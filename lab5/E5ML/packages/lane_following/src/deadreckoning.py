#!/usr/bin/env python3

import time
import math
import rospy
import message_filters
import threading
import wheel_int

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
from tf2_ros import TransformBroadcaster

from tf import transformations as tr
from lane_following.srv import updatepos
import math
import os


HOST_NAME = os.environ["VEHICLE_NAME"]


def clip_npi_pi(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi


class DeadReckoning:
    """Performs deadreckoning.
    The node performs deadreckoning to estimate odometry
    based upon wheel encoder values.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~veh (:obj:`str`): Robot name
        ~publish_hz (:obj:`float`): Frequency at which to publish odometry
        ~encoder_stale_dt (:obj:`float`): Time in seconds after encoders are considered stale
        ~wheelbase (:obj:`float`): Lateral distance between the center of wheels (in meters)
        ~ticks_per_meter (:obj:`int`): Total encoder ticks associated with one meter of travel
        ~debug (:obj: `bool`): Enable/disable debug output

    Publisher:
        ~odom (:obj:`Odometry`): The computed odometry

    Subscribers:
        ~left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`): Encoder ticks for left wheel
        ~right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`): Encoder ticks for right wheel
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.veh = HOST_NAME
        self.publish_hz = 10.0
        self.encoder_stale_dt = 1.0
        self.ticks_per_meter = 656
        self.wheelbase = 0.1
        self.origin_frame = "world"
        self.target_frame = "odometry"
        self.debug = False

        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None

        # Current pose, forward velocity, and angular rate
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_z = 0.0
        self.base_yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.turn_flag = False

        # tf broadcaster for odometry TF
        self._tf_broadcaster = TransformBroadcaster()

        # Used for debugging
        self.x_trajectory = []
        self.y_trajectory = []
        self.yaw_trajectory = []
        self.time = []

        self.total_dist = 0
        self.wheel_integration = wheel_int.WheelPositionIntegration(33, 0, 0, 0)

        # Setup publishers
        self.wheel_pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.pub = rospy.Publisher("~odom", Odometry, queue_size=10)

        # Setup subscribers
        self.sub_left = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)

        def myhook():
            self.send_stop_command()
        rospy.on_shutdown(myhook)

        self.speed = 0.67
        self.rate = rospy.Rate(60)
        
        # Setup update position service
        self.s = rospy.Service('update_pos', updatepos, self.update_service)

        rospy.loginfo("Initialized")
        
    def set_turn_flag(self, turn_flag):
        self.lock.acquire()
        self.turn_flag = turn_flag
        self.lock.release()

    def update_service(self, req):
        self.lock.acquire()
        if not self.turn_flag:
            # print("updatepos received request:", req.x, req.y, req.z)
            self.base_x = req.x
            self.base_y = req.y
            self.base_z = req.z
            self.base_yaw = req.yaw
            self.wheel_integration.reset_position()
        self.lock.release()
        return True

    def left_callback(self, msg):
        self.wheel_integration.update_left(msg.data, rospy.get_rostime())

    def right_callback(self, msg):
        self.wheel_integration.update_right(msg.data, rospy.get_rostime())

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()  # Ideally, should be encoder time
        odom.header.frame_id = self.origin_frame
        byaw = self.base_yaw
        pose_x, pose_y, yaw = self.wheel_integration.get_state_meters()
        pose_x, pose_y = \
            self.base_x + pose_x * math.cos(byaw) + pose_y * math.sin(byaw), \
            self.base_y + pose_y * math.cos(byaw) - pose_y * math.sin(byaw)
        q = tr.quaternion_from_euler(0, 0, self.angle_clamp(yaw + self.base_yaw))
        odom.pose.pose = Pose(Point(pose_x, pose_y, self.base_z), Quaternion(*q))
        odom.child_frame_id = self.target_frame
        odom.twist.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        self.pub.publish(odom)

        self._tf_broadcaster.sendTransform(
            TransformStamped(
                header=odom.header,
                child_frame_id=self.target_frame,
                transform=Transform(
                    translation=Vector3(pose_x, pose_y, self.base_z), rotation=Quaternion(*q)
                ),
            )
        )

    def reset_position(self):
        # print('resetting position')
        rx, ry, yaw = self.calc_relative_position()
        self.base_x += rx
        self.base_y += ry
        self.base_yaw += yaw
        self.wheel_integration.reset_position()
    
    def calc_relative_position(self):
        byaw = self.base_yaw
        pose_x, pose_y, yaw = self.wheel_integration.get_state_meters()
        pose_x, pose_y = \
            pose_x * math.cos(byaw) + pose_y * math.sin(byaw), \
            pose_y * math.cos(byaw) - pose_y * math.sin(byaw)
        return pose_x, pose_y, yaw

    def send_stop_command(self):
        self.drive(0, 0)

    def drive(self, left_speed, right_speed):

        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.wheel_pub.publish(msg)
        self.publish_odometry()

    def stop(self, stop_time=16):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        for i in range(stop_time):
            self.drive(0., 0.)
            self.rate.sleep()
    
    def driveForTime(self, left, right, time):
        for i in range(time):
            self.drive(left, right)
            self.rate.sleep()
    
    def driveToPoint(self, x, y):
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state_meters()
            dx, dy = x - curx, y - cury
            # rospy.loginfo(f'to target location: {x}, {y} from {curx} {cury}, {curtheta}')
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 0.05:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = clip_npi_pi(to_target - curtheta)
            OFF_THRESHOLD = 0.9
            if abs(to_adjust) > OFF_THRESHOLD:
                self.stop()
                self.adjustRotation(to_adjust)
                self.stop()
            else:
                speed = self.speed
                q = to_adjust / OFF_THRESHOLD * 0.5
                self.drive(speed * (1 - q), speed * (1 + q))
                self.rate.sleep()
    
    def adjustRotation(self, to_adjust):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta - 0.21:
                self.drive(-self.speed, self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        else:
            while curtheta > target_theta + 0.21:
                self.drive(self.speed, -self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        self.drive(0, 0)
    
    def adjustRotationWhileDriving(self, to_adjust):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta:
                self.drive(self.speed * 0.5, self.speed * 1.5)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        else:
            while curtheta > target_theta:
                self.drive(self.speed * 1.5, self.speed * 0.5)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        self.drive(0, 0)
    
    def adjustToTargetRotation(self, adjust_target_radian):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        to_adjust = ((adjust_target_radian - curtheta) + math.pi) % (math.pi * 2) - math.pi
        if abs(to_adjust) > 0.05:
            self.adjustRotation(to_adjust)

    def driveForDistance(self, distance):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()

        targetx, targety = curx + distance * math.cos(curtheta), cury + distance * math.sin(curtheta)
        rospy.loginfo(f'driving to {targetx} {targety}')
        self.driveToPoint(targetx, targety)
        self.stop()
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * math.pi:
            return theta - 2 * math.pi
        elif theta < -2 * math.pi:
            return theta + 2 * math.pi
        else:
            return theta


if __name__ == "__main__":
    # create node
    dead_reckoning = DeadReckoning()
