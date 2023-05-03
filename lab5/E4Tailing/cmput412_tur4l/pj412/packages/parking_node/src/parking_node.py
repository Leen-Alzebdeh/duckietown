#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32, String, Float32
from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from geometry_msgs.msg import Transform, Vector3, Quaternion

HOST_NAME = os.environ["VEHICLE_NAME"]
PARKING_1 = 207
PARKING_2 = 226
PARKING_3 = 228
PARKING_4 = 75

class Parking(DTROS):

    def __init__(self, node_name):
        super(Parking, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.PARKING_SLOT = int(rospy.get_param("~parking_slot"))


        #Static Parameters
        self._radius = 0.0318
        self._robot_width = 0.1

        #Subscibers
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/apriltag_detector_node/detections', AprilTagDetectionArray,self.callback)
        self.sub_left_wheel = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback,callback_args="left")
        self.sub_right_wheel = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback,callback_args="right")

        #Publishers
        self.pub = rospy.Publisher(f"/{HOST_NAME}/wheels_driver_node/wheels_cmd", WheelsCmdStamped,queue_size=1)
        self.pub_cmd = rospy.Publisher(f"/{HOST_NAME}/wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, queue_size=1)
        self.genreal_shutdown = rospy.Publisher('/general', String, queue_size = 1)
        
        #Encoder variables
        self.left_tick = None
        self.right_tick = None

        self.delta_left = 0
        self.delta_right = 0

        #Move and Turning variables
        self.left_distance = 0
        self.right_distance = 0
        self.angle = 0

        #Apriltag variables
        self.tr_x = None
        self.tr_z = None

        self.tr_x_helper = None
        self.tr_z_helper = None
        
        self.sign_x = None
        self.sign_z = None
        #Parking variables
        self.parking_slot_id = None
        self.helper_slot_id = None
        self.target_found = False
        self.helper_found = False

    def assign_ids(self):
        if self.PARKING_SLOT == 1:
            self.parking_slot_id = PARKING_1
            self.helper_slot_id = PARKING_3

        elif self.PARKING_SLOT == 2:
            self.parking_slot_id = PARKING_2
            self.helper_slot_id = PARKING_4

        elif self.PARKING_SLOT == 3:
            self.parking_slot_id = PARKING_3
            self.helper_slot_id = PARKING_1

        elif self.PARKING_SLOT == 4:
            self.parking_slot_id = PARKING_4
            self.helper_slot_id = PARKING_1

    def callback(self,msg):
        for i in msg.detections:
            # if i.tag_id == self.parking_slot_id:
            #     self.tr_x = i.transform.translation.x
            #     self.tr_z = i.transform.translation.z
            #     self.target_found = True
            if i.tag_id == self.helper_slot_id:
                self.tr_x_helper = i.transform.translation.x
                self.tr_z_helper = i.transform.translation.z
                self.helper_found = True
            elif i.tag_id == 227:
                self.sign_x = i.transform.translation.x
                self.sign_z = i.transform.translation.z

    def wheel_callback(self,msg,wheel):
        if wheel == "left":
            if self.left_tick == None:
                self.left_tick = abs(msg.data)
            orig_tick = self.left_tick

        elif wheel == "right":
            if self.right_tick == None:
                self.right_tick = abs(msg.data)
            orig_tick = self.right_tick

        delta = 2 * math.pi * self._radius*(abs(msg.data) - orig_tick)/ msg.resolution
        
        if wheel == "left":
            self.left_tick = abs(msg.data)
            self.delta_left = delta
            self.left_distance += abs(delta)

        else:
            self.right_tick = abs(msg.data)
            self.delta_right = delta
            self.right_distance += abs(delta)

    def reset_variables(self):
        self.angle = 0
        self.left_distance = 0
        self.right_distance = 0
    
    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        self.pub_cmd.publish(msg)

    def turn(self,direction,angle=(math.pi),v1=0.3,v2 = 0.3):
        self.reset_variables()

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()

        if direction == "left":
            msg.vel_left = -v1
            msg.vel_right = v2

        else:
            msg.vel_left = v1
            msg.vel_right = -v2

        while self.angle < angle:
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            self.angle += (self.right_distance + self.left_distance)/(self._robot_width)
            rospy.sleep(0.1)

        self.stop()

    def find_apriltag(self,is_target = True):
        if is_target:
            while not self.target_found:
                self.turn("left",math.pi/4)
                rospy.sleep(0.1)

        else:
            while not self.helper_found:
                self.turn("left",math.pi/4)
                rospy.sleep(0.1)

    def move(self,distance,v1=0.3,v2=0.3):
        self.reset_variables()

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v1
        msg.vel_right = v2

        d_traveled = 0

        while d_traveled < distance:
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            d_traveled = (self.left_distance + self.right_distance)/2
            rospy.sleep(0.1)

        self.stop()
        rospy.sleep(1)

    def take_initial_position(self):
        while not self.sign_z or not self.parking_slot_id:
            rospy.sleep(0.1)

        if self.parking_slot_id == PARKING_1 or self.parking_slot_id == PARKING_3:
            distance = 0.25
        elif self.parking_slot_id == PARKING_2 or self.parking_slot_id == PARKING_4:
            distance = 0.55

        while self.sign_z > distance:
            if self.sign_x < 0:
                self.move(0.01,v1 = 0.3,v2 = 0.3 + abs(self.sign_x)*5)
               
            elif self.sign_x > 0:
                self.move(0.01,v1= 0.3 + abs(self.sign_x)*5,v2=0.3)

            print("Z:",self.sign_z)
            print("Distance:",distance)
            
            rospy.sleep(0.1)

        rospy.sleep(1)

    def take_position(self):

        self.reset_variables()
        self.take_initial_position()
        if self.PARKING_SLOT == 1 or self.PARKING_SLOT == 2:
            self.turn("right",(math.pi/2))
        else:
            self.turn("left",(math.pi/2))

        rospy.sleep(1)

    def allign(self):
        self.reset_variables()

        while not self.helper_found:
            rospy.sleep(0.1)

        while self.tr_z_helper < 1.9:
            if self.tr_x_helper < 0:
                self.move(0.01,v1 = -0.3 - abs(self.sign_x)*5,v2 = -0.3)

            else:
                self.move(0.01,v1 = -0.3, v2 = -0.3 - abs(self.sign_x)*5)
            print("Z:",self.tr_z_helper)
            print("X:",self.tr_x_helper)



    def shutdown(self):
        self.genreal_shutdown.publish("shutdown")
        
        rospy.signal_shutdown("Shutting Down ...")

    def main(self):
        self.assign_ids()
        self.take_position()
        self.allign()
        
        self.shutdown()
        """
        Main idea is:
                taking position -> calculate distance using translation matrix -> Do 180 turn -> Use Rotation Matrix to Allign -> Go Backwards
        
        
        """
if __name__ == '__main__':
    parking_node = Parking('parking_node')
    while not rospy.is_shutdown():
        parking_node.main()



