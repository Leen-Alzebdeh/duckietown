#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import importlib
from duckietown.dtros import DTROS, TopicType, NodeType


def callback(msg):
    if msg.data == 'shutdown':
        rospy.signal_shutdown('received shutdown message')
    elif msg.data == 'pkg_reload':
        import entry
        importlib.reload(entry)
        entry.entry()


if __name__ == '__main__':
    node = DTROS('lane_following_node', node_type=NodeType.GENERIC)
    rospy.Subscriber('/general', String, callback)
    rospy.spin()

