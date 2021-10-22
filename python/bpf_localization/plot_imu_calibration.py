#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plt
import numpy as np

class PlotImuCalibration():
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 3, figsize=(10, 8))
        self.x = [0.]
        self.y = [0.]
        self.phase = 0.
        self.x_size = 0

    def plot(self):
        if len(self.x) > self.x_size:
            self.ax[0,0].set_xlabel("angular_velocity x")
            self.ax[0,0].set_xlim([-2., 2.])
            self.ax[0,0].set_ylim([-.1., .1])
            line, = self.ax[0,0].plot(self.x, self.y, 'b-')
            line.set_xdata(self.x)
            line.set_ydata(self.y)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.x_size = len(self.x)

    def __del__(self):
        plt.close(self.fig)

    def feed(self, data):
        self.phase += 1.
        self.x.append(self.phase)
        self.y.append(self.phase)
