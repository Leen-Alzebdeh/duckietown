#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
import yaml
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from dt_apriltags import Detector
import rospkg
import math

from std_msgs.msg import Int32, String
from duckietown.dtros import DTROS, TopicType, NodeType
import digit_crop
import detect_history
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Transform, Vector3, Quaternion
import tf


HOST_NAME = os.environ["VEHICLE_NAME"]
IGNORE_DISTANCE_MAX = .6
IGNORE_DISTANCE_MIN = .34
DEBUG = True


def _matrix_to_quaternion(r):
    T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)


def send_compressed(pub, seq, frame_id, im):
    msg = CompressedImage()
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    ret, buffer = cv2.imencode('.jpg', im)
    if not ret:
        print('failed to encode image!')
    else:
        msg.data = np.array(buffer).tostring()
        pub.publish(msg)
    return ret


class MLNode(DTROS):

    def __init__(self, node_name):
        super(MLNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospack = rospkg.RosPack()
        self._tf_bcaster = tf.TransformBroadcaster()
        self.seq = 0
        self.intrinsic = self.readYamlFile(rospack.get_path('ml_node') + '/src/camera_intrinsic.yaml')
        self.detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.detect_history = detect_history.DetectHistory()
        self.timer = 0

        self.continue_run = True
        self.camera_sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.general_sub = rospy.Subscriber('/general', String, self.general_callback)
        self.digit_sub = rospy.Subscriber(f'/{HOST_NAME}/april_tag_node/detected_digit', String, self.digit_callback)

        self.general_pub = rospy.Publisher(f'/general', String, queue_size=10)
        self.tag_pub = rospy.Publisher(f'/{HOST_NAME}/detected_tagid', Int32, queue_size=10)
        self.digit_pub = rospy.Publisher(f'/{HOST_NAME}/cropped_digit/compressed', CompressedImage, queue_size=10)
        self.rviz_pub = rospy.Publisher(f'/{HOST_NAME}/detection_visualization/compressed', CompressedImage, queue_size=10)
        self.detections_pub = rospy.Publisher(f'/{HOST_NAME}/apriltag_detector_node/detections', AprilTagDetectionArray,
                                    queue_size=2)
    
    def digit_callback(self, msg):
        strs = msg.data.split()
        seq = int(strs[0]) - 1
        digit = int(strs[1])
        tagid = self.detect_history.add_recognition(seq, digit)
        if self.detect_history.is_all_digit_recognized():
            rospy.loginfo(f'all tags detected, shutting down the container...')
            self.general_pub.publish(String('shutdown'))
        # rospy.loginfo(f'digit recognized:{digit} on tag {tagid} with im seq={seq}')

    def general_callback(self, msg):
        if msg.data == 'shutdown':
            rospy.signal_shutdown('received shutdown message')
        elif msg.data == 'stop':
            self.continue_run = False

    def crop_and_send_digit(self, undistort_im, detected_tags, roi):
        if DEBUG:
            dst = np.copy(undistort_im)

        for det in detected_tags:
            ymin = int(np.min(det.corners[:, 1]).item())
            x, y = det.center
            x, y = int(x), int(y)

            # ignore tags that are too close
            ihom_pose = det.pose_t
            distance = np.linalg.norm(ihom_pose)
            if IGNORE_DISTANCE_MAX < distance or IGNORE_DISTANCE_MIN > distance:
                continue

            # broadcast tag id
            id = det.tag_id
            self.tag_pub.publish(Int32(id))
            predy = ymin * 2 - y
            rect, digit_im = digit_crop.get_cropped_digit(undistort_im, (x, predy))

            if digit_im is None:
                continue  # no digit detected
            else:
                # published the cropped digit image
                self.detect_history.add_detection(self.seq, id, 1)
                ret = send_compressed(self.digit_pub, self.seq, f'{HOST_NAME}/camera_optical_frame', digit_im)
                if ret:
                    self.seq += 1
                
                # draw borders and center for each tag if in debugging mode
                # if DEBUG:
                #     hom_pose = np.zeros((4, 1), dtype=np.float32)
                #     hom_pose[0:3, :] = ihom_pose
                #     hom_proj_pose_t = camera_proj_mat @ hom_pose
                #     ihom_proj_pose_t = hom_proj_pose_t[0:2, :] / hom_proj_pose_t[2, 0]
                #     proj_x, proj_y = int(ihom_proj_pose_t[0, 0].item()), int(ihom_proj_pose_t[1, 0].item())

                    # cv2.drawMarker(dst, (proj_x, proj_y), (0, 0, 255))
                    # dst = cv2.putText(dst, str(id), (proj_x, proj_y), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 1, cv2.LINE_AA)
                    # cv2.polylines(dst, np.array([det.corners], dtype=np.int32), True, (255, 0, 0))
                    # ihom_center = np.sum(np.array(det.corners, dtype=np.float32), axis=0) / 4
                    # centerx, centery = int(ihom_center[0].item()), int(ihom_center[1].item())
                    # cv2.drawMarker(dst, (centerx, centery), (0, 255, 0))
                
                if DEBUG:
                    # draw the bound box and most likely digit for each tag
                    bound_x, bound_y, bound_w, bound_h = rect
                    corners = np.array(((
                        (bound_x, bound_y), 
                        (bound_x + bound_w, bound_y),
                        (bound_x + bound_w, bound_y + bound_h),
                        (bound_x, bound_y + bound_h)), ), dtype=np.int32)
                    cv2.polylines(dst, corners, True, (255, 0, 0))
                    dst = cv2.putText(dst, str(self.detect_history.get_most_likely_digit(id)), 
                                      (x, predy), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 1, cv2.LINE_AA)

        if DEBUG:
            x,y,w,h = roi
            dst = dst[y:y+h, x:x+w]
            send_compressed(self.rviz_pub, self.seq, f'{HOST_NAME}/camera_optical_frame', dst)
    
    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        self.timer += 1
        if self.timer % 8 == 0:
            compressed_image = np.frombuffer(msg.data, np.uint8)
            im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)

            camera_matrix = np.array(self.intrinsic["camera_matrix"]["data"]).reshape(3,3)
            camera_proj_mat = np.concatenate((camera_matrix, np.zeros((3, 1), dtype=np.float32)), axis=1)
            distort_coeff = np.array(self.intrinsic["distortion_coefficients"]["data"]).reshape(5,1)
            fx = camera_matrix[0][0].item()
            fy = camera_matrix[1][1].item()
            cx = camera_matrix[0][2].item()
            cy = camera_matrix[1][2].item()
            tag_size = 0.065  # in meters

            width = im.shape[1]
            height = im.shape[0]

            newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distort_coeff, (width,height), 1, (width,height))
            undistort_im = cv2.undistort(im, camera_matrix, distort_coeff, None, newmatrix)
            input_image = cv2.cvtColor(undistort_im, cv2.COLOR_BGR2GRAY)
            detected_tags = self.detector.detect(input_image, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tag_size)

            # pack detections into a message
            tags_msg = AprilTagDetectionArray()
            tags_msg.header.stamp = msg.header.stamp
            tags_msg.header.frame_id = msg.header.frame_id
            for tag in detected_tags:
                # turn rotation matrix into quaternion
                q = _matrix_to_quaternion(tag.pose_R)
                p = tag.pose_t.T[0]
                # create single tag detection object
                detection = AprilTagDetection(
                    transform=Transform(
                        translation=Vector3(x=p[0], y=p[1], z=p[2]),
                        rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                    ),
                    tag_id=tag.tag_id,
                    tag_family=str(tag.tag_family),
                    hamming=tag.hamming,
                    decision_margin=tag.decision_margin,
                    homography=tag.homography.flatten().astype(np.float32).tolist(),
                    center=tag.center.tolist(),
                    corners=tag.corners.flatten().tolist(),
                    pose_error=tag.pose_err,
                )
                # add detection to array
                tags_msg.detections.append(detection)
                # publish tf
                self._tf_bcaster.sendTransform(
                    p.tolist(),
                    q.tolist(),
                    msg.header.stamp,
                    "tag/{:s}".format(str(tag.tag_id)),
                    msg.header.frame_id,
                )
            # publish detections
            self.detections_pub.publish(tags_msg)

            self.crop_and_send_digit(undistort_im, detected_tags, roi)

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

    def onShutdown(self):
        super(MLNode, self).onShutdown()


if __name__ == '__main__':
    ml_node = MLNode('ml_node')
    rospy.spin()
    #ml_node.run()


