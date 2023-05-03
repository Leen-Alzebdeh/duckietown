#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
import matplotlib.pyplot as plt
import tensorflow as tf

import rospy
from duckietown.dtros import DTROS, TopicType, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import rospkg

HOST_NAME = os.environ["VEHICLE_NAME"]


class Digit_Detection(DTROS):

    def __init__(self,node_name):

        super(Digit_Detection, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.sub = rospy.Subscriber(f'/{HOST_NAME}/cropped_digit/compressed', CompressedImage, self.callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/detected_digit', String, queue_size=10)

        self.image = None
        self.model = None
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path("digit_detection_node")
        self.trained_model = str(self.path) + "/src/digit_detection.model"
        self.prediction = None


    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        msg.header.seq
        compressed_image = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
        im = im[:,:,0]
        self.image = im
        self.predict(msg.header.seq)

    def load_model(self):
        self.model = tf.keras.models.load_model(self.trained_model)

    def predict(self,sequence):
        tf.keras.utils.normalize(self.image,axis=0)
        prediction = self.model.predict(self.image[np.newaxis, :, :], verbose=0)
        print(f"The detected digit is: {np.argmax(prediction)}")
        self.pub.publish(String(str(sequence) + " " + str(np.argmax(prediction))))

if __name__ == '__main__':
    digit_node = Digit_Detection("digit_detection_node")
    digit_node.load_model()
    rospy.spin()