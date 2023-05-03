#!/usr/bin/env python3
import math
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Float32


class OdometryNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        # Static parameters 
        self.veh_name = os.environ["VEHICLE_NAME"]
        self._radius = 0.0318

        # Variables
        self.orig_tick_left = None
        self.orig_tick_right = None
        self.left_dir = 1
        self.right_dir = 1

        # Subscribers
        self.sub_encoder_ticks_left = rospy.Subscriber(
            f"/{self.veh_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.cb_encoder_data,
            callback_args="left"
        )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            f"/{self.veh_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.cb_encoder_data,
            callback_args="right"
        )
        self.sub_executed_cmd = rospy.Subscriber(
            f"/{self.veh_name}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped,
            self.cb_executed_commands
        )

        # Publishers
        self.pub_delta_left = rospy.Publisher(
            f"/{self.veh_name}/odometry_node/left_wheel_delta",
            Float32,
            queue_size=10
        )
        self.pub_delta_right = rospy.Publisher(
            f"/{self.veh_name}/odometry_node/right_wheel_delta",
            Float32,
            queue_size=10
        )
    
    def cb_encoder_data(self, msg, wheel):
        """Get the encoder data and publish the delta of distance."""
        # Set the base value of tick
        if self.orig_tick_left is None and wheel == "left":
            self.orig_tick_left = abs(msg.data)
        if self.orig_tick_right is None and wheel == "right":
            self.orig_tick_right = abs(msg.data)
        
        # Compute delta of distance on a wheel
        orig_tick = self.orig_tick_left if wheel == "left" else self.orig_tick_right
        delta_x = 2 * math.pi * self._radius * (abs(msg.data) - orig_tick) / msg.resolution
        delta_x *= self.left_dir if wheel == "left" else self.right_dir
        if wheel == "left":
            delta_x = delta_x * self.left_dir
        else:
            delta_x = delta_x * self.right_dir
                
        # Update variables and publish to the appropriate topics
        if wheel == "left":
            self.orig_tick_left = abs(msg.data)
            self.pub_delta_left.publish(delta_x)
        else:
            self.orig_tick_right = abs(msg.data)
            self.pub_delta_right.publish(delta_x)
    
    def cb_executed_commands(self, msg):
        """Get direction of rotation."""
        self.left_dir = 1 if msg.vel_left >= 0.0 else -1
        self.right_dir = 1 if msg.vel_right >= 0.0 else -1  


if __name__ == "__main__":
    odom_node = OdometryNode("odometry")
    rospy.spin()