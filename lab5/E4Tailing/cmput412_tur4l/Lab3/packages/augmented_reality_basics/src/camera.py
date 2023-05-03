#!/usr/bin/env python3

"""
TODO: Shutdown nodes (send shutdown request to odometry node).
"""

import math
import os
import subprocess
import time

import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image,CompressedImage
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped
from duckietown_msgs.srv import ChangePattern


class Augmenter(DTROS):
    def __init__(
        self,
        node_name,
    ):
        super(Augmenter, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        
        # Static variables
        self._veh = os.environ["VEHICLE_NAME"]
        self._rate = rospy.Rate(100)
        self._map_name = ""

        # Variables
        self.image

        # Publisher
        self.pub_image = rospy.Publisher(
            f"{self._veh}/augmented_reality_basics_node/{self.map_file}/image/compressed",
            CompressedImage
        )

        # Subscribers
        self.get_image= rospy.Subscriber(
            f"{self._veh}/camera_node/image/compressed",CompressedImage,self.cb_image
        )

        def cb_image(self,msg):
            self.image = msg
            

        def readYamlFile(self,fname):
            """
            Reads the YAML file in the path specified by 'fname'.
            E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
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
            
        def process_image(self,msg):
            raise("NotImplementedError")
        
        def ground2pixel(self,msg):
            raise("NotImplementedError")

        def draw_segment(self, image, pt_x, pt_y, color):
            defined_colors = {
                'red': ['rgb', [1, 0, 0]],
                'green': ['rgb', [0, 1, 0]],
                'blue': ['rgb', [0, 0, 1]],
                'yellow': ['rgb', [1, 1, 0]],
                'magenta': ['rgb', [1, 0 , 1]],
                'cyan': ['rgb', [0, 1, 1]],
                'white': ['rgb', [1, 1, 1]],
                'black': ['rgb', [0, 0, 0]]}
            color_type, [r, g, b] = defined_colors[color]
            cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
            return image

        def render_segments(self,msg):
            raise("NotImplementedError")
            
    
if __name__ == "__main__":
    # Initialize driver node
    driver = Augmenter("augmented_reality_basics_node")