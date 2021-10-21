#!/usr/bin/env python
import rospy

class PlotImuCalibration(object):
    def __init__(self):
        rospy.loginfo("toto")

    def plot(self, data):
        print(data)
