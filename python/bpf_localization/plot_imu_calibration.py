#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
from enum import Enum

class Methods(Enum):
    TIME_DEPENDENT = 1
    DENSITY_DISTRIBUTION = 2

class CallbackTriggeredPlotting(object):
    def __init__(self, i, j, method):
        plt.ion()
        self.fig, self.ax = plt.subplots(i, j, figsize=(10, 8))
        self.data_size = 0
        self.vectors = None
        self.method = method

    def __del__(self):
        plt.close(self.fig)

    def timeDependent(self, vector, i, u):
        self.ax[i, u].set_xlabel(self.labels[i][u])
        self.ax[i, u].set_ylim([min(vector), max(vector)])
        line, = self.ax[i, u].plot(vector, 'x', color='blue')

    def densityDistribution(self, vector, i, u):
        self.ax[i, u].set_xlabel(self.labels[i][u])
        lb = min(vector)
        ub = max(vector)
        self.ax[i, u].set_xlim([lb, ub])
        bins = 20 
        line = self.ax[i, u].hist(vector, bins)

    def plotMethod(self, vector, i, u):
        if self.method == Methods.TIME_DEPENDENT : 
            self.timeDependent(vector, i, u)
        elif self.method == Methods.DENSITY_DISTRIBUTION : 
            self.densityDistribution(vector, i, u)
        else : raise("Bad plot method") 

    def plot(self):
        if self.vectors == None: raise("vectors has to be initialized")
        if len(self.vectors[0][0]) > self.data_size:
            for i in range(len(self.vectors)):
                for u in range(len(self.vectors[i])):
                    self.plotMethod(self.vectors[i][u], i, u)
            self.fig.canvas.flush_events()
            self.fig.canvas.draw()
            self.data_size = len(self.vectors[0][0])

    def feed(self, data):
        raise("Feed method has to be implemented")

class PlotImuCalibration(CallbackTriggeredPlotting):
    def __init__(self, method):
        CallbackTriggeredPlotting.__init__(self, 2, 3, method)
        self.labels = [["angular_velocity x",
                        "angular_velocity y",
                        "angular_velocity z",],
                        ["linear_acceleration x",
                         "linear_acceleration y",
                         "linear_acceleration z",]]
        self.angular_velocity    = [[],[],[]] 
        self.linear_acceleration = [[],[],[]]
        self.vectors = [self.angular_velocity, self.linear_acceleration]

    def feed(self, data):
        self.angular_velocity[0]    += [data.angular_velocity.x]
        self.angular_velocity[1]    += [data.angular_velocity.y]
        self.angular_velocity[2]    += [data.angular_velocity.z]
        self.linear_acceleration[0] += [data.linear_acceleration.x]
        self.linear_acceleration[1] += [data.linear_acceleration.y]
        self.linear_acceleration[2] += [data.linear_acceleration.z]

