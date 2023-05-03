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
import onnx


HOST_NAME = os.environ["VEHICLE_NAME"]
IGNORE_DISTANCE = 1.0
DEBUG = True

class MLNode(DTROS):

    def __init__(self, node_name):
        super(MLNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospack = rospkg.RosPack()
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
        self.timer = 0

        self.continue_run = True
        def general_callback(msg):
            if msg.data == 'shutdown':
                rospy.signal_shutdown('received shutdown message')
            elif msg.data == 'stop':
                self.continue_run = False
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.tag_pub = rospy.Publisher(f'/{HOST_NAME}/detected_tagid', Int32, queue_size=10)
        rospy.Subscriber('/general', String, general_callback)

        if DEBUG:
            self.pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/compressed', CompressedImage, queue_size=10)
            self.seq = 0

    
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
            dst = undistort_im
            x,y,w,h=0,0,0,0

            for det in detected_tags:
                bottomR, _, _, _ = det.corners 
                x,y = det.center
                x, y = int(x), math.floor(y) 
                w = (x-int(bottomR[0]))
                h = (math.ceil((bottomR[1])) - y)
                y = y - 2*h

                # ignore tags that are too close
                ihom_pose = det.pose_t
                if np.linalg.norm(ihom_pose) > IGNORE_DISTANCE:
                    continue

                # broadcast tag id
                id = det.tag_id
                self.tag_pub.publish(Int32(id))
                
                
                # draw borders and center for each tag if in debugging mode
                # if DEBUG:
                #     hom_pose = np.zeros((4, 1), dtype=np.float32)
                #     hom_pose[0:3, :] = ihom_pose
                #     hom_proj_pose_t = camera_proj_mat @ hom_pose
                #     ihom_proj_pose_t = hom_proj_pose_t[0:2, :] / hom_proj_pose_t[2, 0]
                #     proj_x, proj_y = int(ihom_proj_pose_t[0, 0].item()), int(ihom_proj_pose_t[1, 0].item())

                #     cv2.drawMarker(dst, (proj_x, proj_y), (0, 0, 255))
                #     dst = cv2.putText(dst, str(id), (proj_x, proj_y), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 1, cv2.LINE_AA)
                #     cv2.polylines(dst, np.array([det.corners], dtype=np.int32), True, (255, 0, 0))
                    # ihom_center = np.sum(np.array(det.corners, dtype=np.float32), axis=0) / 4
                    # centerx, centery = int(ihom_center[0].item()), int(ihom_center[1].item())
                    # cv2.drawMarker(dst, (centerx, centery), (0, 255, 0))
                
                # TODO: ptopR, topL, bottomL, bottomR = det.coublish tag ids and detect digits with trained ML model

            if DEBUG:
                #x,y,w,h = roi 
                dst = dst[y-h:y+h, x-w:x+w]
                dst = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)



                msg = CompressedImage()
                msg.header.seq = self.seq
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = f'{HOST_NAME}/camera_optical_frame'
                msg.format = 'jpeg'
                ret, buffer = cv2.imencode('.jpg', dst)
                if not ret:
                    print('failed to encode image!')
                else:
                    msg.data = np.array(buffer).tostring()
                    self.pub.publish(msg)
                    self.seq += 1

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


