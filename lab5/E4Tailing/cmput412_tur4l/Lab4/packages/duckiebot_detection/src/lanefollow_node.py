#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32
from turbojpeg import TurboJPEG
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
Intersection_MASK = [(0,0,255),(128,128,255)]
DEBUG = False
ENGLISH = False

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = "csc22913"

        # Publishers & Subscribers
        self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="20MB")
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)
        self.pub_vel = rospy.Publisher("/"+self.veh+"/wheels_driver_node/wheels_cmd",
                                    WheelsCmdStamped,
                                    queue_size=1)
        self.pub_executed_cmd = rospy.Publisher("/"+self.veh+"/wheels_driver_node/wheels_cmd_executed",
                                    WheelsCmdStamped,
                                    queue_size=1)
        self.detection_sub = rospy.Subscriber("/" + self.veh + "/duckiebot_detection_node/detection",
                                              BoolStamped,
                                              self.cb_detection_update,
                                              callback_args="detection")
        self.detection_distance = rospy.Subscriber("/"+self.veh+"/duckiebot_distance_node/distance",
                                                   Float32,
                                                   self.cb_detection_update,
                                                   callback_args ="det_dist")
        self.detection_angle = rospy.Subscriber("/"+self.veh+"/duckiebot_distance_node/angle",
                                                   Float32,
                                                   self.cb_detection_update,
                                                   callback_args ="det_angle")
        

        self.jpeg = TurboJPEG()

        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.25
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()
        self.duck_detected = False
        self.intersection = False
        self.duck_distance = 0.0
        self.duck_angle = 0.0
        self.safe_distance = 0.4 
        self.intersection_crossed = False

        # Shutdown hook
        rospy.on_shutdown(self.hook)

    def cb_detection_update(self,msg,detection):
        if detection == "detection":
            self.duck_detected = msg.data
            print("Is duck detected: ",msg.data)

        if detection == "det_dist":
            self.duck_distance = msg.data
            print("Distance is: ", msg.data)


        if detection == "det_angle":
            self.duck_angle = msg.data 
            print("angle is: ", msg.data)


    def send_msg(self,msg):
        self.pub_vel.publish(msg)
        self.pub_executed_cmd.publish(msg)

    def reset(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub_vel.publish(msg)

        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        


    def callback(self, msg):
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        intersection = cv2.inRange(crop, Intersection_MASK[0],Intersection_MASK[1])
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        akif = np.sum(np.nonzero(intersection))
        if akif == 0:
            self.intersection = False
        else:
            self.intersection = True

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
            self.proportional = None

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def drive(self):
        if self.duck_detected:
            if self.duck_distance > self.safe_distance:
                msg = WheelsCmdStamped()
                msg.header.stamp = rospy.Time.now()
                msg.vel_left = self.velocity + self.duck_angle
                msg.vel_right = self.velocity
                self.send_msg(msg)

            else:
                msg = WheelsCmdStamped()
                msg.header.stamp = rospy.Time.now()
                msg.vel_left = 0
                msg.vel_right = 0
                self.send_msg(msg)


        else:
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
        if node.intersection and not node.intersection_crossed:
            print("At intersection")
            rospy.sleep(1)
            node.intersection_crossed = True

        else:
            node.intersection_crossed = False

        node.drive()
        rate.sleep()