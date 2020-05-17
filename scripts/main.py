#!/usr/bin/env python
import rospy
from sensors import SensorNode

if __name__ == "__main__":
    ros_node = SensorNode()
    rospy.spin()