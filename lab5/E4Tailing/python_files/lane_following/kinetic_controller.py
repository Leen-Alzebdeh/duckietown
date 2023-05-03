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
import time


HOST_NAME = os.environ["VEHICLE_NAME"]
P_DECAY_FACTOR = 0.5
I_DECAY_FACTOR = 0.02
D_DECAY_FACTOR = 0.05

STATE_TOO_CLOSE = 0
STATE_WAITING_FOR_TURN = 1
STATE_DRIVING = 2
STATE_TURNING = 3


def clip_0_2pi(rad):
    return rad % (2 * math.pi)


def clip_npi_pi(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi


class KineticController:
    def __init__(self, angle_coeffs, position_coeffs):
        """
        cp, ci and cd are all need to be smaller than 0
        cp < 0
        ci < 0
        cd > 0
        """
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        def myhook():
            self.stop()
        rospy.on_shutdown(myhook)

        self.angle_coeffs = angle_coeffs
        self.position_coeffs = position_coeffs

        self.angle_error_i = 0  # integrated
        self.angle_error_p = 0
        self.angle_error = 0
        self.angle_error_d = 0
        self.position_error_i = 0
        self.position_error_p = 0
        self.position_error = 0
        self.position_error_d = 0
        self.adjustment = 0

        self.turn_timer = 0
        self.actions_queue = []


        self.sub_left = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.pose_pub = rospy.Publisher(f'/{HOST_NAME}/pose2d', Pose2DStamped, queue_size=10)
        self.rate = rospy.Rate(60)  # in Hz
        self.wheel_integration = wheel_int.WheelPositionIntegration(33, 0, 0, math.pi / 2)
        self.speed = 0.6
        self.count = 0
        self.LEFT_TICKS = None
        self.RIGHT_TICKS = None
        self.initial_left = None
        self.initial_right = None
    
    def stop(self):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        self.drive(0, 0)

    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)
    
    def actionQueueIsEmpty(self):
        return len(self.actions_queue) == 0

    def getCurrentState(self):
        if len(self.actions_queue) == 0:
            return None
        else:
            state = self.actions_queue[0][3]
            return state

    def update(self, is_car_too_close):
        if len(self.actions_queue) == 0:
            self.drive(0., 0.)
        else:
            left_speed, right_speed, time, state = self.actions_queue[0]
            if (left_speed == 0 and right_speed == 0) or not is_car_too_close:
                self.drive(left_speed, right_speed)
                if time <= 1:
                    self.actions_queue.pop(0)
                else:
                    self.actions_queue[0] = (left_speed, right_speed, time - 1, state)
            else:
                self.drive(0., 0.)

    def driveForTime(self, left_speed, right_speed, ntime_step, state):
        self.actions_queue.append((left_speed, right_speed, ntime_step, state))
    
    def update_error(self, angle_error, position_error):
        self.angle_error_i = (self.angle_error_i + angle_error) * (1 - I_DECAY_FACTOR)
        self.position_error_i = (self.position_error_i + position_error) * (1 - I_DECAY_FACTOR)

        self.angle_error_d = self.angle_error_d * (1 - D_DECAY_FACTOR) + (angle_error - self.angle_error) * D_DECAY_FACTOR
        self.position_error_d = self.position_error_d * (1 - D_DECAY_FACTOR) + (position_error - self.position_error) * D_DECAY_FACTOR

        self.angle_error = angle_error
        self.position_error = position_error

        self.angle_error_p = self.angle_error_p * (1 - P_DECAY_FACTOR) + angle_error * P_DECAY_FACTOR
        self.position_error_p = self.position_error_p * (1 - P_DECAY_FACTOR) + position_error * P_DECAY_FACTOR

        ap, ai, ad = self.angle_coeffs
        pp, pi, pd = self.position_coeffs

        angle_adjustment = self.angle_error_i * ai + self.angle_error_p * ap + self.angle_error_d * ad
        position_adjustment = self.position_error_i * pi + self.position_error_p * pp + self.position_error_d * pd
        self.adjustment = angle_adjustment + position_adjustment

    def get_adjustment(self):
        return self.adjustment



    def left_callback(self, msg):
        self.wheel_integration.update_left(msg.data, rospy.get_rostime())

    def right_callback(self, msg):
        self.wheel_integration.update_right(msg.data, rospy.get_rostime())

    def stop_momentum(self):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        self.driveForTime(0, 0, 16, STATE_TURNING)
    
    def driveToPoint(self, x, y):
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state_meters()
            dx, dy = x - curx, y - cury
            rospy.loginfo(f'to target location: {x}, {y} from {curx} {cury}, {curtheta}')
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 0.05:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = clip_npi_pi(to_target - curtheta)
            OFF_THRESHOLD = 0.9
            if abs(to_adjust) > OFF_THRESHOLD:
                self.stop_momentum()
                self.adjustRotation(to_adjust)
                self.stop_momentum()
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
        self.stop_momentum()
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation
    
