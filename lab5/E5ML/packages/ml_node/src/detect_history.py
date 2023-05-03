import rospy
from threading import Lock
import numpy as np


class DetectHistory:
    def __init__(self):
        self.lock = Lock()
        self.tagid_list = (153, 94, 93, 62, 133, 169, 200, 201, 58, 162)
        self.detection_history = {}  # maps seq numbers to tagid
        # maps tagid to an array of weight a digit is recognized
        self.tag_history = { id: np.zeros(10) for id in self.tagid_list }
        self.count = 0

    def add_detection(self, seq, tagid, weight):
        self.lock.acquire()
        self.detection_history[seq] = (tagid, weight)
        self.lock.release()
    
    def add_recognition(self, seq, digit):
        self.lock.acquire()
        tagid, weight = self.detection_history[seq]
        if tagid not in self.tagid_list:
            self.lock.release()
            return  # do nothing
        self.tag_history[tagid][digit] += weight
        self.lock.release()
        return tagid
    
    def get_most_likely_digit(self, tagid):
        if tagid not in self.tagid_list:
            return -1
        self.lock.acquire()
        best_digit = -1
        most_occurred = -1
        for i in range(10):
            occurrence = self.tag_history[tagid][i]
            if occurrence > most_occurred:
                most_occurred = occurrence
                best_digit = i
        self.lock.release()
        return best_digit
    
    def get_history_for_tag(self, tagid):
        self.lock.acquire()
        if tagid not in self.tagid_list:
            self.lock.release()
            return None
        history = np.copy(self.tag_history[tagid])
        self.lock.release()
        return history
    
    def is_all_digit_recognized(self):
        self.lock.acquire()
        self.count += 1
        flag = True
        if self.count % 10 == 0:
            rospy.loginfo('detection matrix')
        for tagid in self.tagid_list:
            if self.count % 10 == 0:
                rospy.loginfo(str(tagid) + str(self.tag_history[tagid]))
            if np.sum(self.tag_history[tagid]) < 3:
                flag = False
        self.lock.release()
        return flag


