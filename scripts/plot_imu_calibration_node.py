#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy

from sensor_msgs.msg import Imu
from bpf_localization.plot_imu_calibration import *

class PlotImuCalibrationROS(PlotImuCalibration):
    def __init__(self, method, name):
        PlotImuCalibration.__init__(self, method, name)
        rospy.Subscriber("/imu", Imu, self.callback)

    def callback(self, data):
        self.feed(data)

if __name__ == '__main__':
    rospy.init_node('py_plot_imu', anonymous = True)

    name = "arronax" 
    plot_imu  = PlotImuCalibrationROS(PlotMethods.TIME_DEPENDENT, name)
    plot_imu1 = PlotImuCalibrationROS(PlotMethods.DENSITY_DISTRIBUTION, name)
    plot_imu2 = PlotImuCalibrationROS(PlotMethods.MID_MEAN_DISTANCE, name)
    plot_imu3 = PlotImuCalibrationROS(PlotMethods.LB_UB, name)

    while 1: 
        plot_imu.plot()
        plot_imu1.plot()
        plot_imu2.plot()
        plot_imu3.plot()

    #rospy.spin()
