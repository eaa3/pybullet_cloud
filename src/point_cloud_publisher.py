#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
import struct
import threading
import signal

g_mutex = threading.Lock()

def RGB2Uint(rgb):
    rgb_hex = struct.pack('BBB',*rgb).encode('hex')
    return int(rgb_hex, 16)

class PointCloudPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher("sim_cloud", PointCloud2, queue_size=1)

        self.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                       PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        self.points = [[0.0, 0.0, 0.0, RGB2Uint((0,0,0))]]
        self.frame_name = "bullet_camera"

    def run_thread(self):
        threading.Thread(target=self.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    def update(self, xyz_rgb):
        global g_mutex
        g_mutex.acquire()
        self.points = []
        for i in range(xyz_rgb.shape[0]):
            R, G, B = int(xyz_rgb[i,3]), int(xyz_rgb[i,4]), int(xyz_rgb[i,5])
            point = [xyz_rgb[i,0], xyz_rgb[i,1], xyz_rgb[i,2], RGB2Uint((R,G,B))]
            self.points.append(point)
        g_mutex.release()

    def loop(self):
        r = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_name

            global g_mutex
            g_mutex.acquire()
            pcl_data = pcl2.create_cloud(header, self.fields, self.points)
            g_mutex.release()

            self.pub.publish(pcl_data)

            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
