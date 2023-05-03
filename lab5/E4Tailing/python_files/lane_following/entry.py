import numpy as np
import os
import math
import cv2

import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, ColorRGBA
import threading
from dt_apriltags import Detector
import rospkg
import yaml

import kinetic_controller
import detection_info

HOST_NAME = os.environ["VEHICLE_NAME"]
PUBLISH_IMAGE = True
PUBLISH_IMAGE_TYPE = 'red'
PROCESSING_RATE = 20
TURN_CENTERS = ((260, 120), (320, 100), (380, 120))
IGNORE_DISTANCE = .5

STATE_TOO_CLOSE = 0
STATE_WAITING_FOR_TURN = 1
STATE_DRIVING = 2
STATE_TURNING = 3


class LaneFollowingNode:
    def __init__(self):
        # Initialize an instance of Renderer giving the model in input.
        self.count = 0
        self.image_lock = threading.Lock()
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback, queue_size=1)
        if PUBLISH_IMAGE:
            self.pub = rospy.Publisher(f'/{HOST_NAME}/lane_following/compressed', CompressedImage, queue_size=10)
        self.detection_manager = detection_info.DetectionManager()
        self.image = None
        self.seq = 0
        
        ANGLE_MULT = 0.
        POSITION_MULT = 1.
        self.controller = kinetic_controller.KineticController(
            (ANGLE_MULT * .3, ANGLE_MULT * .002, ANGLE_MULT * -3), 
            (POSITION_MULT * .005, POSITION_MULT * .00005, POSITION_MULT * -.0))
        
        self.max_speed = 0.56  # top speed when driving in a single lane
        self.speed = self.max_speed  # current speed
        self.correct_x = 1
        self.timer = 0

        self.turn_flag = False
        self.stop_timer_default = PROCESSING_RATE * .25  # time before stopping after seeing a red line
        self.stop_timer = self.stop_timer_default  # current timer, maxed out at self.stop_timer_default
        self.cur_pattern = None

        self.continue_run = True
        self.last_angle_error = 0.
        self.last_position_error = 0.

        rospack = rospkg.RosPack()
        self.intrinsic = self.readYamlFile(rospack.get_path('lane_following') + '/src/camera_intrinsic.yaml')
        self.detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.last_seen_apriltag = 201

        rospy.Subscriber('/general', String, self.general_callback)

    def change_pattern(self, pattern):
        pass

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        self.count += 1
        if self.count % 2 == 0:
            compressed_image = np.frombuffer(msg.data, np.uint8)
            im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
            self.image_lock.acquire()
            self.image = im
            self.image_lock.release()

    def general_callback(self, msg):
        if msg.data == 'stop':
            self.continue_run = False
        strs = msg.data.split()
        if len(strs) == 4:
            cp, ci, cd = float(strs[1]), float(strs[2]), float(strs[3])
            if strs[0] == 'position':
                self.controller.position_coeffs = (cp, ci, cd)
            elif strs[0] == 'angle':
                self.controller.angle_coeffs = (cp, ci, cd)
            else:
                pass
        elif len(strs) % 3 == 0:
            # received wheel commands
            for i in range(len(strs) // 3):
                left, right, time = float(strs[i * 3]), float(strs[i * 3 + 1]), float(strs[i * 3 + 2])
                self.controller.driveForTime(left, right, time, STATE_DRIVING)

    def run(self):
        rate = rospy.Rate(PROCESSING_RATE)  # in Hz
        for i in range(10):
            self.controller.drive(0, 0)
            rate.sleep()

        while not rospy.is_shutdown():
            if not self.continue_run:
                self.controller.drive(0, 0)
                break

            self.image_lock.acquire()
            im = self.image
            self.image_lock.release()
            if im is not None:
                self.timer += 1

                self.stopline_processing(im)
                self.update_controller(im)
                self.controller.update(self.detection_manager.isCarTooClose())
                if self.timer % 5 == 0:
                    self.update_apriltag_detection(im)
            rate.sleep()


    def update_apriltag_detection(self, im):
        camera_matrix = np.array(self.intrinsic["camera_matrix"]["data"]).reshape(3,3)
        distort_coeff = np.array(self.intrinsic["distortion_coefficients"]["data"]).reshape(5,1)
        fx = camera_matrix[0][0].item()
        fy = camera_matrix[1][1].item()
        cx = camera_matrix[0][2].item()
        cy = camera_matrix[1][2].item()
        tag_size = 0.065  # in meters

        width = self.image.shape[1]
        height = self.image.shape[0]

        newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distort_coeff, (width,height), 1, (width,height))
        undistort_im = cv2.undistort(self.image, camera_matrix, distort_coeff, None, newmatrix)
        input_image = cv2.cvtColor(undistort_im, cv2.COLOR_BGR2GRAY)
        detected_tags = self.detector.detect(input_image, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tag_size)
        for det in detected_tags:
            id = det.tag_id
            
            ihom_pose = det.pose_t
            if np.linalg.norm(ihom_pose) > IGNORE_DISTANCE:
                continue
            self.last_seen_apriltag = id

    
    def update_controller(self, im):
        publish_flag = PUBLISH_IMAGE and PUBLISH_IMAGE_TYPE == 'yellow'
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        lower_range = np.array([22,100,150])
        upper_range = np.array([30,255,255])

        yellow_mask = cv2.inRange(hsv, lower_range, upper_range)
        yellow_mask[:260, 330:] = 0
        img_dilation = cv2.dilate(yellow_mask, np.ones((35, 35), np.uint8), iterations=1)

        contours, hierarchy = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # pick the largest contour
        largest_area = 0
        largest_idx = -1
        refx, refy = im.shape[1] * 0.5, 130.5
        for i in range(len(contours)):
            ctn = contours[i]
            xmin, ymin, width, height = cv2.boundingRect(ctn)
            midx, midy = xmin + .5 * width, ymin + .5 * height
            if midy < 190 or midx + midy < 290:  # crop top half
                continue
            area = cv2.contourArea(ctn)
            if area > largest_area:
                largest_area = area
                largest_idx = i

        vx, vy, cosref, sinref = 1, 0, 1, 0
        position_ref = 0
        contour_y = 0
        contour_x = 0
        if largest_idx != -1:
            largest_ctn = contours[largest_idx]
            if publish_flag:
                im = cv2.drawContours(im, contours, largest_idx, (0,255,0), 3)
            [vx,vy,x,y] = cv2.fitLine(largest_ctn, cv2.DIST_L2,0,0.01,0.01)
            vx, vy = vx[0], vy[0]
            if vx + vy > 0:
                vx, vy = -vx, -vy
            angle = math.atan2(vy, vx)

            xmin, ymin, width, height = cv2.boundingRect(largest_ctn)
            contour_x, contour_y = xmin + width * 0.5, ymin + height * 0.5
            ref_angle = math.atan2(refy - contour_y, refx - contour_x)     
            cosref, sinref = math.cos(ref_angle), math.sin(ref_angle)
            angle_error = (ref_angle - angle + math.pi) % (2 * math.pi) - math.pi
            if contour_y >= 420 or (contour_x - refx) ** 2 + (contour_y - refy) ** 2 < 155 ** 2:
                angle_error = 0.

            down_right_pt_x = 320. + 120. * self.correct_x
            position_line_ref = np.cross(
                np.array((im.shape[1] * 0.5, 130.5, 1.)), 
                np.array((70., down_right_pt_x, 1.)))
            position_line_ref /= position_line_ref[0]
            position_ref = -position_line_ref[2].item() - contour_y * position_line_ref[1].item()
            position_error = position_ref - contour_x

            if contour_x <= 10 or contour_y >= 420 or (contour_x - refx) ** 2 + (contour_y - refy) ** 2 < 155 ** 2:
                angle_error = self.last_angle_error
            
            self.last_angle_error = angle_error
            self.last_position_error = position_error
        else:
            angle_error = self.last_angle_error
            position_error = self.last_position_error
        
        position_error = max(position_error, -280.)
        
        if self.controller.actionQueueIsEmpty():
            if self.detection_manager.isCarTooClose():
                if self.turn_flag:
                    self.controller.driveForTime(0., 0., 1, STATE_WAITING_FOR_TURN)
                else:
                    self.controller.driveForTime(0., 0., 1, STATE_TOO_CLOSE)
            else:
                self.controller.update_error(angle_error, position_error)
                adjust = self.controller.get_adjustment()

                adjust = max(min(adjust, .9), -.9)
                left_speed = self.speed * (1 - adjust)
                right_speed = self.speed * (1 + adjust)
                self.controller.driveForTime(left_speed, right_speed, 1, STATE_DRIVING)

        if publish_flag:
            ARROW_LENGTH = 50
            if largest_idx !=-1:
                if angle_error != 0:
                    cv2.arrowedLine(im,
                        (int(contour_x), int(contour_y)), 
                        (int(contour_x + vx * ARROW_LENGTH), int(contour_y + vy * ARROW_LENGTH)), 
                        (255, 0, 0), 3)
                    cv2.arrowedLine(im,
                        (int(contour_x), int(contour_y)), 
                        (int(contour_x + cosref * ARROW_LENGTH), int(contour_y + sinref * ARROW_LENGTH)), 
                        (0, 255, 0), 3)
                cv2.arrowedLine(im,
                    (int(contour_x + position_error), int(contour_y)), 
                    (int(contour_x + position_error), int(contour_y - ARROW_LENGTH)), 
                    (0, 0, 255), 3)
            msg = CompressedImage()
            msg.header.seq = self.seq
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            ret, buffer = cv2.imencode('.jpg', im)
            if not ret:
                print('failed to encode image!')
            else:
                msg.data = np.array(buffer).tostring()
                self.pub.publish(msg)
                self.seq += 1
    
    def stopline_processing(self, im):
        publish_flag = PUBLISH_IMAGE and PUBLISH_IMAGE_TYPE == 'red'
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

            if area > largest_area and area > 4000 and xmax > im.shape[1] * .5 and xmin < im.shape[1] * .5:
                largest_area = area
                largest_idx = i

        contour_y = 0
        if largest_idx != -1:
            largest_ctn = contours[largest_idx]

            # if publish_flag:
            #     im = cv2.drawContours(im, contours, largest_idx, (0,255,0), 3)

            xmin, ymin, width, height = cv2.boundingRect(largest_ctn)
            contour_y = ymin + height * 0.5

        if publish_flag:
            last_observed_x, last_observed_y = self.detection_manager.getCenter()
            cv2.arrowedLine(im,
                (int(last_observed_x), int(last_observed_y)), 
                (int(last_observed_x), int(last_observed_y + 10)), 
                (255, 255, 0), 3)
            for i in range(len(TURN_CENTERS)):
                last_observed_x, last_observed_y = TURN_CENTERS[i]
                cv2.arrowedLine(im,
                    (int(last_observed_x), int(last_observed_y)), 
                    (int(last_observed_x), int(last_observed_y + 10)), 
                    (255, 255, 0), 3)
        if self.turn_flag:
            if self.detection_manager.isSafeToTurn():
                if self.controller.actionQueueIsEmpty():
                    self.controller.wheel_integration.reset_position()
                    # make a turn
                    tagid = self.last_seen_apriltag

                    if tagid in (201, 200, 58):
                        possible_turns = [1, 2]
                        id_after = [None, 133, 162]
                    elif tagid in (133, 94, 93):
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
                    else: # id is 153
                        possible_turns = [0, 1]
                        id_after = [169, 62, None]

                    turn_idx = possible_turns[0]
                    best_distance_square = math.inf
                    last_observed_x, last_observed_y = self.detection_manager.getCenter()

                    for i in range(len(possible_turns)):
                        cur_turn_idx = possible_turns[i]
                        cur_turn_center_x, cur_turn_center_y = TURN_CENTERS[cur_turn_idx]
                        cur_distance_square = (cur_turn_center_x - last_observed_x) ** 2 + (cur_turn_center_y - last_observed_y) ** 2
                        if cur_distance_square < best_distance_square:
                            best_distance_square = cur_distance_square
                            turn_idx = cur_turn_idx

                    self.speed = self.max_speed
                    self.last_seen_apriltag = id_after[turn_idx]

                    rospy.loginfo(f'turn idx:{turn_idx}, apriltag seen: {self.last_seen_apriltag}')
              
                    if turn_idx == 0:
                        self.controller.driveForDistance(.46)
                        self.controller.adjustToTargetRotation(math.pi *.6)
                        self.controller.driveForDistance(.5)
                    elif turn_idx == 1:
                        self.controller.driveForDistance(.5)
                    elif turn_idx == 2:
                        self.controller.driveForDistance(.3)
                        self.controller.adjustToTargetRotation(-math.pi *.5)
                        self.controller.driveForDistance(.3)

                    # reset the detection list since we are out of the intersection after the turn
                    self.turn_flag = False
                    self.stop_timer = self.stop_timer_default + PROCESSING_RATE * 2.8

        self.correct_x = (contour_y - 330) / (390 - 330)
        self.correct_x = 1 - min(1, max(0, self.correct_x))

        if self.stop_timer <= self.stop_timer_default and \
            (contour_y > 390 or (contour_y > 380 and self.stop_timer < self.stop_timer_default)):
            self.speed = 0
            self.stop_timer = self.stop_timer_default + 99999
            self.turn_flag = True

            self.controller.driveForTime(0., 0., PROCESSING_RATE * .75, STATE_WAITING_FOR_TURN)
        else:  # not approaching stop line
            if self.stop_timer > self.stop_timer_default:
                self.stop_timer = max(self.stop_timer - 1, self.stop_timer_default)
            else:
                self.stop_timer = min(self.stop_timer + 1, self.stop_timer_default)
                

        if publish_flag:
            im = cv2.drawContours(im, contours, -1, (0,255,0), 3)

            msg = CompressedImage()
            msg.header.seq = self.seq
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            ret, buffer = cv2.imencode('.jpg', im)
            if not ret:
                print('failed to encode image!')
            else:
                msg.data = np.array(buffer).tostring()
                self.pub.publish(msg)
                self.seq += 1


def entry():
    ar_node = LaneFollowingNode()
    ar_node.run()
