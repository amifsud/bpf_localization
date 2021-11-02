#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
from enum import Enum

class PlotMethods(Enum):
    TIME_DEPENDENT          = 1
    DENSITY_DISTRIBUTION    = 2
    MID_MEAN_DISTANCE       = 3
    LB_UB                   = 4

class CallbackTriggeredPlotting(object):
    def __init__(self, i, j, method):
        plt.ion()
        self.fig, self.ax = plt.subplots(i, j, figsize=(10, 8))
        self.data_size = 0
        self.vectors        = self.initDataContainers(i, j) 
        self.mid_points     = self.initDataContainers(i, j)
        self.means          = self.initDataContainers(i, j)
        self.lb_evolution   = self.initDataContainers(i, j)
        self.ub_evolution   = self.initDataContainers(i, j)
        self.method = method
        self.lb = 
            [[-0.0255413819104, -0.0489543154836, 0.0329909510911],
             [0.143652096391, -0.0383072271943, 9.53849983215]]
        self.ub = 
            [[-0.00532112130895,-0.0329909510911, 0.0489543154836],
             [0.339976638556, 0.210689753294, 9.9359369278]]

    def __del__(self):
        plt.close(self.fig)

    def initDataContainers(self, i, u):
        vect = []
        for ii in range(i):
            vect += [[]]
            for uu in range(u):
                vect[ii] += [[]]
        return vect

    def timeDependent(self, vector, i, u):
        self.ax[i, u].set_xlabel(self.labels[i][u])
        self.ax[i, u].set_ylim(
            [self.lb_evolution[i][u][-1], 
             self.ub_evolution[i][u][-1]])
        line, = self.ax[i, u].plot(vector, 'x', color='blue')

    def densityDistribution(self, vector, i, u):
        r = 0.9
        bins = 100
        self.ax[i, u].cla()
        self.ax[i, u].set_xlabel(self.labels[i][u])
        self.ax[i, u].set_xlim(
            [self.lb_evolution[i][u][-1], 
             self.ub_evolution[i][u][-1]])
        line = self.ax[i, u].hist(vector, bins, color='orange')
        ylim = self.ax[i, u].get_ylim()
        self.ax[i, u].vlines(self.means[i][u][-1], r*ylim[0], r*ylim[1])
        self.ax[i, u].vlines(self.mid_points[i][u][-1], r*ylim[0], r*ylim[1], color='blue')

    def meadMinDistance(self, vector, i, u):
        self.ax[i, u].cla()
        self.ax[i, u].set_xlabel(self.labels[i][u])
        self.ax[i, u].plot(np.array(self.means[i][u])-np.array(self.mid_points[i][u]))

    def lb_ub(self, vector, i, u):
        self.ax[i, u].cla()
        self.ax[i, u].set_xlabel(self.labels[i][u])
        self.ax[i, u].plot(self.lb_evolution[i][u])
        self.ax[i, u].plot(self.ub_evolution[i][u])

    def plotMethod(self, vector, i, u):
        if self.method == PlotMethods.TIME_DEPENDENT : 
            self.timeDependent(vector, i, u)
        elif self.method == PlotMethods.DENSITY_DISTRIBUTION : 
            self.densityDistribution(vector, i, u)
        elif self.method == PlotMethods.MID_MEAN_DISTANCE : 
            self.meadMinDistance(vector, i, u)
        elif self.method == PlotMethods.LB_UB : 
            self.lb_ub(vector, i, u)
        else : raise("Bad plot method")

    def update(self):
        for i in range(len(self.vectors)):
            for u in range(len(self.vectors[i])):
                vector = self.vectors[i][u]
                lb = min(vector)
                ub = max(vector)
                self.lb_evolution[i][u] += [lb]
                self.ub_evolution[i][u] += [ub]
                self.means[i][u]        += [np.array(vector).mean()]
                self.mid_points[i][u]   += [(ub-lb)/2.+lb]

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
    def __init__(self, method, name):
        CallbackTriggeredPlotting.__init__(self, 2, 3, method)
        self.name = name
        self.labels = 
            [["angular_velocity x",
              "angular_velocity y",
              "angular_velocity z"],
             ["linear_acceleration x",
              "linear_acceleration y",
              "linear_acceleration z"]]
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
        self.update()

