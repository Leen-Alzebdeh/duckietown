#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
import numpy as np


HOST_NAME = os.environ["VEHICLE_NAME"]


class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub_left = rospy.Subscriber(f'{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)
        self.counter = 0
    
    def left_callback(self, msg):
        self.counter += 1
        if self.counter % 2 == 0:
            print(f'left reading:{msg.data}')
    
    def right_callback(self, msg):
        self.counter += 1
        if self.counter % 2 == 0:
            print(f'right reading:{msg.data}')


# if __name__ == '__main__':
#     node = MySubscriberNode('my_subscriber_node')
#     rospy.spin()