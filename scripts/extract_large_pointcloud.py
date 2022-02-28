#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import threading


class ExtractLargePointcloud(object):
    def __init__(self):
        rospy.Subscriber(
            '/tilt_laser_listener/output_cloud', PointCloud2,
            self.cloud_callback)
        self.pub = rospy.Publisher(
            '~output', PointCloud2, queue_size=1)
        duration = rospy.get_param('~duration', 30)
        rospy.Timer(rospy.Duration(duration), self.timer_callback)
        self.cloud_msg = None
        self.cloud_len = 0
        self.lock = threading.Lock()

    def cloud_callback(self, msg):
        self.lock.acquire()
        if self.cloud_len < len(msg.data):
            self.cloud_len = len(msg.data)
        self.cloud_msg = msg
        self.lock.release()

    def timer_callback(self, event):
        self.lock.acquire()
        if self.cloud_msg is not None:
            self.pub.publish(self.cloud_msg)
            self.cloud_msg = None
            self.cloud_len = 0
        self.lock.release()


if __name__ == '__main__':
    rospy.init_node('extract_large_pointcloud')
    ExtractLargePointcloud()
    rospy.spin()
