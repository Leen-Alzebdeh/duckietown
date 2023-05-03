import rospy
from duckietown_msgs.msg import WheelsCmdStamped
import os
import math

from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Pose2DStamped
from std_msgs.msg import String, Int8, Float32, ColorRGBA
import math
import wheel_int

HOST_NAME = os.environ["VEHICLE_NAME"]


def clip_0_2pi(rad):
    return rad % (2 * math.pi)


def clip_npi_pi(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi


class KineticController:
    def __init__(self):
        self.sub_left = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rate = rospy.Rate(60)  # in Hz
        self.wheel_integration = wheel_int.WheelPositionIntegration(33, 0, 0, math.pi / 2)
        def myhook():
            self.send_stop_command()
        rospy.on_shutdown(myhook)

        self.speed = 0.67
    
    def send_stop_command(self):
        self.drive(0, 0)

    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)

    def left_callback(self, msg):
        self.wheel_integration.update_left(msg.data, rospy.get_rostime())

    def right_callback(self, msg):
        self.wheel_integration.update_right(msg.data, rospy.get_rostime())

    def stop(self, stop_time=16):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        for i in range(stop_time):
            self.drive(0., 0.)
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
    
