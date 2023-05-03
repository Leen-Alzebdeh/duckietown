#!/usr/bin/env python3

from std_msgs.msg import String
import rospy
from duckietown.dtros import DTROS, NodeType
import os
from gh_download import download_from_github
from github import Github
from git_token import TOKEN
import rospkg


"""
When the container starts, none of the packages except this file (reloader) will run. This file will download all the newest code on github. 
When the programmer issues a 'start' string message to the /general topic, and notify all the other packages to start running after the 
download is complete
"""

HOST_NAME = os.environ["VEHICLE_NAME"]


class ReloaderNode(DTROS):
    def __init__(self, node_name, github):
        super(ReloaderNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(f'/general', String, queue_size=10)
        rospack = rospkg.RosPack()

        def reloader_callback(msg):
            nonlocal github
            if msg.data == 'start':
                owner = github.get_user('CMPUT412-Friday-Lab-Team')
                print(f'current working folder:{os.getcwd()}')
                for package_name in ('lane_following', 'duckiebot_detection'):
                    target_folder = rospack.get_path(package_name) + f'/src'
                    download_from_github(owner, f'E4Tailing', 'main', target_folder, f'python_files/{package_name}')
                    print(f'resulting files:')
                    for file in os.listdir(target_folder):
                        print('    ', file)
                self.pub.publish('pkg_reload')
            elif msg.data == 'shutdown':
                rospy.signal_shutdown('received shutdown message in general')
        self.sub = rospy.Subscriber(f'/general', String, reloader_callback)


if __name__ == '__main__':
    github = Github(TOKEN)

    node = ReloaderNode('reloader_node', github)
    rospy.spin()
