#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np

class CallbackTriggeredPlotting(object):
    def __init__(self, i, j):
        plt.ion()
        self.fig, self.ax = plt.subplots(i, j, figsize=(10, 8))
        self.data_size = 0

    def __del__(self):
        plt.close(self.fig)

    def plotVector(self, vector, nb):
        for i in range(len(vector)):
            self.ax[nb, i].set_xlabel(self.labels[nb][i])
            self.ax[nb, i].set_ylim([min(vector[i]), max(vector[i])])
            line, = self.ax[nb, i].plot(vector[i], 'x', color='blue')
            line.set_ydata(vector[i])

    def plot(self):
        if len(self.linear_acceleration[0]) > self.data_size:
            self.plotVector(self.angular_velocity, 0)
            self.plotVector(self.linear_acceleration, 1)
            self.fig.canvas.flush_events()
            self.fig.canvas.draw()
            self.data_size = len(self.linear_acceleration[0])

    def feed(self, data):
        raise("Feed method has to be implemented")


class PlotImuCalibration(CallbackTriggeredPlotting):
    def __init__(self):
        CallbackTriggeredPlotting.__init__(self, 2, 3)
        self.labels = [["angular_velocity x",
                        "angular_velocity y",
                        "angular_velocity z",],
                        ["linear_acceleration x",
                         "linear_acceleration y",
                         "linear_acceleration z",]]
        self.angular_velocity    = [[],[],[]] 
        self.linear_acceleration = [[],[],[]] 

    def feed(self, data):
        self.angular_velocity[0]    += [data.angular_velocity.x]
        self.angular_velocity[1]    += [data.angular_velocity.y]
        self.angular_velocity[2]    += [data.angular_velocity.z]
        self.linear_acceleration[0] += [data.linear_acceleration.x]
        self.linear_acceleration[1] += [data.linear_acceleration.y]
        self.linear_acceleration[2] += [data.linear_acceleration.z]

