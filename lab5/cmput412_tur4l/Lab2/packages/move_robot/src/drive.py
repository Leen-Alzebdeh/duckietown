#!/usr/bin/env python3

"""
TODO: Shutdown nodes (send shutdown request to odometry node).
"""

import math
import os
import subprocess
import time

import rosbag
import rospy
from std_msgs.msg import Float32, String

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped
from duckietown_msgs.srv import ChangePattern


class DriverNode(DTROS):
    def __init__(
        self,
        node_name,
        init_frame_bag_name,
        world_frame_bag_name
    ):
        super(DriverNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        
        # Static variables
        self._veh = os.environ["VEHICLE_NAME"]
        self._rate = rospy.Rate(100)
        self._robot_width_half = 0.05

        # Values to keep track on
        self.total_distance = 0.0   # Total distance progressed by robot
        self.left_distance = 0.0    # Total distance progressed by left wheel
        self.right_distance = 0.0   # Total distance progressed by right wheel

        self.prev_left_dist = 0.0
        
        # Initial robot frame and world frame
        # Initial robot framdt-exec roslaunch duckiebot_detection duckiebot_detection_node.launche
        self.irf_x = 0.0
        self.irf_y = 0.0
        self.irf_t = 0.0
        
        # World frame
        self.wf_x = 0.0
        self.wf_y = 0.0
        self.wf_t = 0.0

        # Rosbag
        self.if_bag = rosbag.Bag(f"/data/bags/{init_frame_bag_name}.bag", 'w')
        self.wf_bag = rosbag.Bag(f"/data/bags/{world_frame_bag_name}.bag", 'w')

        # Service client
        rospy.wait_for_service(f"/{self.veh}/led_emitter_node/set_pattern")
        
        self.srv_led = rospy.ServiceProxy(
            f"/{self.veh}/led_emitter_node/set_pattern",
            ChangePattern
        )

        # Publisher
        self.pub_vel = rospy.Publisher(
            f"{self._veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )
        self.pub_executed_cmd = rospy.Publisher(
            f"/{self._veh}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped,
            queue_size=10
        )
        
        # Subscribers
        self.delta_dist_left = rospy.Subscriber(
            f"{self._veh}/odometry_node/left_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="left"
        )
        self.delta_dist_right = rospy.Subscriber(
            f"{self._veh}/odometry_node/right_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="right"
        )
    
    def reset_variables(self):
        """Reset all tracking variables to 0.
        
        Arguments
        ---------
        None
        
        Returns
        -------
        None
        """
        self.total_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0

    def set_led_color(self, pattern):
        """Set LED pattern to the given desired color.
        
        Aruguments
        ----------
        pattern: str
            String representing the color. By convention, it is fully capitalized.
        """
        msg = String()
        msg.data = pattern 
        self.srv_led(msg)

    def to_init_fram(self,dl,dr):
        """w.r.t. initial robot frame."""
        da = (dl + dr) / 2.
        dt = (dr - dl) / (2 * self._robot_width_half)

        self.irf_x += da * math.cos(self.irf_t)
        self.irf_y += da * math.sin(self.irf_t)
        self.irf_t = (self.irf_t + dt) % (2. * math.pi)

    def to_world_frame(self):
        """Convert the initial robot frame to world frame."""
        self.wf_x = -self.irf_y + 0.32
        self.wf_y = self.irf_x + 0.32
        self.wf_t = (self.irf_t + (math.pi / 2.)) % (2. * math.pi)

    def write_rosbag(self):
        """Write all the coordinates to rosbag."""
        pose = Pose2DStamped()
        pose.header.stamp = rospy.Time.now()

        # Initial frames
        try:
            pose.x = self.irf_x
            pose.y = self.irf_y
            pose.theta = self.irf_t
            self.if_bag.write("initial_frame", pose)

            pose.x = self.wf_x
            pose.y = self.wf_y
            pose.theta = self.wf_t
            self.wf_bag.write("world_frame", pose)
        except:
            print("Could not write as rosbag is closed.")
        
    def cb_param_update(self, msg, wheel):
        """Update distance parameters based on the subscriber's feedback.
        
        Arguments
        ---------
        msg: Float32
            Change of distance on either wheel.
        wheel: str
            Indicator of which wheel has been called. ["left", "right"]
        """
        assert wheel in ["left", "right"]

        if wheel == "right":
            self.right_distance += msg.data
            self.total_distance = (abs(self.left_distance) + abs(self.right_distance)) / 2.
            self.to_init_frame(self.prev_left_dist, msg.data)
            self.to_world_frame()

            # Write to rosbag
            self.write_rosbag()
        else:
            self.left_distance += msg.data
            self.prev_left_dist = msg.data
    
    def send_msg(self, msg):
        """Send the message indicating velocity to suitable topic.
        
        Arguments
        ---------
        msg: WheelsCmdStamped
            Velocity message that we are feeding into a topic.
        """
        self.pub_vel.publish(msg)
        self.pub_executed_cmd.publish(msg)

    def stop(self):
        """Method to stop the robot's movement."""
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = 0.0
        msg.vel_right = 0.0

        self.send_msg(msg)

    def move(self, distance, vel_left=0.4, vel_right=0.4, offset=0):
        """Method to move the robot in straight line for desired distance.z
        
        Arguments
        ---------
        distance: float
            Desired distance for robot to move straight. In meters.
        vel_left: float
            Velocity of left wheel.
        vel_right: float
            Velocity of right wheel.
        """
        # assert (vel_left > 0.0 and vel_right > 0.0) or (vel_left < 0.0 and vel_right < 0.0)
        
        # Reset the variables
        self.reset_variables()
        
        # Construct message
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        
        # Move until distance reaches `distance`
        while abs(self.total_distance) < abs(distance) - offset:
            # print(self.irf_x)
            msg.header.stamp = rospy.Time.now()
            self.send_msg(msg)
            self._rate.sleep()
        
        # Stop after reaching
        self.stop()

    def shutdown_hook(self):
        print("Shutting down a Driver node...")

        # Close rosbag file
        self.if_bag.close()
        self.wf_bag.close()

        # Send signal to kill odometry node
        subprocess.run(["rosnode", "kill", "odometry"])

        self.set_led_color("LIGHT_OFF")

        subprocess.run(["rosnode", "kill", "drive"])


if __name__ == "__main__":
    # Initialize driver node
    driver = DriverNode("drive", "if_bag", "wf_bag")
    # Initialize service proxy

    # driver.set_led_color("RED")
    half_ang = math.pi * driver._robot_width_half * 0.5
    ang = math.pi * driver._robot_width_half

    ### Lab 2 Part 1
    # Straight line task
    # driver.move(1, 0.6, 0.6)
    
    ### Lab 2 Part 2
    
    # Start timer
    st = time.time()
    
    ### Uncomment below for running task for part 2 ###
    # State 1.
    driver.set_led_color("RED")
    time.sleep(5)

    # State 2.
    driver.set_led_color("BLUE")
    driver.move(half_ang, 0.6, -0.6, -half_ang*0.1)
    for i in range(3):
        driver.move(1, 0.6, 0.6)
        if i < 2:
            rat = 0.2
            driver.move(half_ang, -0.45, 0.45, half_ang*rat)

    # State 1.
    driver.set_led_color("RED")
    time.sleep(5)

    # State 3.
    driver.set_led_color("GREEN")
    driver.move(half_ang, -0.55, 0.55, 0.3*half_ang)
    driver.move(1, 0.6, 0.6)
    driver.move(ang, -0.55, 0.55, -0.1*ang)

    # State 1.
    driver.set_led_color("RED")
    time.sleep(5)
    
    # State 4.
    driver.set_led_color("WHITE")
    driver.move(0.05, 0.6, 0.6)
    driver.move(2*math.pi*0.6, 0.7, 0.45)
    
    print(f"Final world frame location, x: {driver.wf_x}, y: {driver.wf_y}, theta: {driver.wf_t}")
    rospy.on_shutdown(driver.shutdown_hook)

    print(f"Total execution time: {time.time() - st}")