#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy

from sensor_msgs.msg import Imu
from bpf_localization.plot_imu_calibration import *

class PlotImuCalibrationROS(PlotImuCalibration):
    def __init__(self):
        PlotImuCalibration.__init__(self)
        rospy.Subscriber("/imu", Imu, self.callback)

    def callback(self, data):
        self.feed(data)

if __name__ == '__main__':
    rospy.init_node('py_plot_imu', anonymous = True)

    duration = 10.
    plot_imu = PlotImuCalibrationROS()

    while 1: plot_imu.plot()

    #rospy.spin()
