#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, Int32, String
from turbojpeg import TurboJPEG
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import os
import threading
import math
import exploration_history
import deadreckoning

HOST_NAME = os.environ["VEHICLE_NAME"]
ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False
STOP_TIMER_RESET_TIME = 60

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = HOST_NAME
        self.jpeg = TurboJPEG()
        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.36
        self.speed = .6
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()

        # handling stopping at stopline
        self.turn_flag = False  # initiate the turning when this is set to true
        self.stop_timer_reset = 0  # 0 is can stop any time, non-zero means wait a period of time and then we look for stop lines
        self.last_seen_apriltag = 169
        self.lock = threading.Lock()  # used to coordinate the subscriber thread and the main thread
        self.controller = deadreckoning.DeadReckoning()  # will handle wheel commands during turning
        self.exploration_history = exploration_history.ExplorationHistory() 

        # Publishers & Subscribers
        if DEBUG:
            self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage,
                                    self.callback, queue_size=1, buff_size="20MB")
        self.general_sub = rospy.Subscriber('/general', String, self.general_callback)

        self.tag_sub = rospy.Subscriber(f'/{HOST_NAME}/detected_tagid', 
                                    Int32, 
                                    self.tag_callback,
                                    queue_size=1)
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)

    def general_callback(self, msg):
        if msg.data == 'shutdown':
            rospy.signal_shutdown('received shutdown message')
        elif msg.data == 'stop':
            self.continue_run = False

    def is_turning(self):
        self.lock.acquire()
        is_turning = self.turn_flag
        self.lock.release()
        return is_turning

    def tag_callback(self, msg):
        id = msg.data
        if id in (58, 62, 169, 133, 153, 162):
            # rospy.loginfo(f'tag seen:{id}')
            self.lock.acquire()
            self.last_seen_apriltag = id
            self.lock.release()

    def callback(self, msg):
        # update stop timer/timer reset and skip the callback if the vehicle is stopped
        self.lock.acquire()
        stop_timer_reset = self.stop_timer_reset
        self.stop_timer_reset = max(0, stop_timer_reset - 1)
        self.lock.release()
        if self.is_turning():
            self.proportional = None
            return

        img = self.jpeg.decode(msg.data)
        if stop_timer_reset == 0:
            # look for stop line
            self.stopline_processing(img)

        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.proportional = cx - int(crop_width / 2) + self.offset
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = -100 # assume off to the right

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def drive(self):
        if self.is_turning():  # TURNING
            self.controller.stop(20)
            self.controller.reset_position()

            self.lock.acquire()
            last_seen_apriltag = self.last_seen_apriltag
            # make a turn
            tagid = last_seen_apriltag
            if tagid == 58:
                possible_turns = [1, 2]
                id_after = [None, 133, 162]
            elif tagid == 133:
                possible_turns = [1, 2]
                id_after = [None, 58, 169]
            elif tagid == 162:
                possible_turns = [2, 0]
                id_after = [62, None, 58]
            elif tagid == 169:
                possible_turns = [2, 0]
                id_after = [153, None, 133]
            elif tagid == 62:
                possible_turns = [0, 1]
                id_after = [162, 153, None]
            elif tagid == 153: # id is 153
                possible_turns = [0, 1]
                id_after = [169, 62, None]
            else:
                rospy.loginfo('unrecognized tag id!')

            turn_idx = -1
            lowest_count = math.inf
            for cur_turn_idx in possible_turns:
                count = self.exploration_history.getCount(tagid, cur_turn_idx)
                if count < lowest_count:
                    lowest_count = count
                    turn_idx = cur_turn_idx

            self.exploration_history.incCount(tagid, turn_idx)
            rospy.loginfo(f'turn idx:{turn_idx}, apriltag seen: {last_seen_apriltag} updated apriltag: {id_after[turn_idx]}')
            last_seen_apriltag = id_after[turn_idx]
            self.last_seen_apriltag = last_seen_apriltag
            self.lock.release()

            self.controller.set_turn_flag(True)
            self.controller.driveForTime(.6, .6, 6)
            if turn_idx == 0:
                self.controller.driveForTime(.58 * self.speed, 1.42 * self.speed, 40)
            elif turn_idx == 1:
                self.controller.driveForTime(1.1 * self.speed, .9 * self.speed, 60)
            elif turn_idx == 2:
                self.controller.driveForTime(1.47 * self.speed, .53 * self.speed, 15)
            self.controller.set_turn_flag(False)

            # reset the detection list since we are out of the intersection after the turn
            self.lock.acquire()
            self.turn_flag = False
            self.lock.release()
        else:  # PID CONTROLLED LANE FOLLOWING
            if self.proportional is None:
                self.twist.omega = 0
            else:
                # P Term
                P = -self.proportional * self.P

                # D Term
                d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                self.last_error = self.proportional
                self.last_time = rospy.get_time()
                D = d_error * self.D

                self.twist.v = self.velocity
                self.twist.omega = P + D
                if DEBUG:
                    self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

            self.vel_pub.publish(self.twist)

    def stopline_processing(self, im):
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        lower_range = np.array([0,70,120])
        upper_range = np.array([5,180,255])
        red_mask = cv2.inRange(hsv, lower_range, upper_range)
        img_dilation = cv2.dilate(red_mask, np.ones((10, 10), np.uint8), iterations=1)
        contours, hierarchy = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # pick the largest contour
        largest_area = 0
        largest_idx = -1

        for i in range(len(contours)):
            ctn = contours[i]
            area = cv2.contourArea(ctn)

            xmin, ymin, width, height = cv2.boundingRect(ctn)
            xmax = xmin + width
            if area > largest_area and area > 3000 and xmax > im.shape[1] * .5 and xmin < im.shape[1] * .5:
                largest_area = area
                largest_idx = i

        contour_y = 0
        if largest_idx != -1:
            largest_ctn = contours[largest_idx]
            xmin, ymin, width, height = cv2.boundingRect(largest_ctn)
            contour_y = ymin + height * 0.5

        if contour_y > 390:
            self.lock.acquire()
            self.stop_timer_reset = STOP_TIMER_RESET_TIME
            self.turn_flag = True
            self.lock.release()

    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()