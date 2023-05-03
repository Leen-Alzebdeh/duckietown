#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import importlib
from duckietown.dtros import DTROS, TopicType, NodeType


def callback(msg):
    if msg.data == 'shutdown':
        rospy.signal_shutdown('received shutdown message')
    elif msg.data == 'pkg_reload':
        import entry_detection as entry
        importlib.reload(entry)
        entry.entry()


if __name__ == '__main__':
    node = DTROS('duckiebot_detection', node_type=NodeType.PERCEPTION)
    rospy.Subscriber('/general', String, callback)
    rospy.spin()

