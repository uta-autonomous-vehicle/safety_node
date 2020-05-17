#!/usr/bin/env python
import rospy
from datetime import datetime
from sensor_msgs.msg import LaserScan
from std_msgs import msg
import os, cv2 as cv, pdb, message_filters, numpy as np, time


class SensorNode:
    def __init__(self):
        rospy.init_node("ros_node")
        rospy.loginfo("Starting SensorNode.")

        self.initiate_listening()

    def scan_message(self, data):
        threshold = 1
        intensities = np.array(data.intensities)
        ranges = np.array(data.ranges)

        range_min = data.range_min
        range_max = data.range_max

        angle_min = data.angle_min
        angle_max = data.angle_max


        # print "range is from {} to {}".format(range_min, range_max)
        # print "angle is from {} to {}".format(angle_min, angle_max)

        range_n = ranges.shape[0]
        range_n /=2
        ranges = ranges[range_n - 50:range_n + 50]

        ranges = ranges < 1
        if np.any(ranges):
            # print("Not safe")
            self.message_pub.publish(False)
        else:
            # print("Safe")
            self.message_pub.publish(True)
    
        # print "{} {}".format(intensities.shape, ranges.shape)
        # print "ranges any is at {}".format(np.any(ranges))
        # print "ranges min is at {}".format(np.min(ranges))

        return
    
    def initiate_listening(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_message)
        self.message_pub = rospy.Publisher("/safety_node/safety", msg.Bool, queue_size=10)