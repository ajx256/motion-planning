
from cmath import cos, sin
from math import atan2
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

import math
import sys

def on_close(event):
    plt.close()
    exit()

def plot2DPoint(x, y):
    plt.plot([x], [y], 'ro')
    #plt.pause(0.001)
    
def plot2DLine(x1, y1, x2, y2, color):
    plt.plot([x1, x2], [y1, y2], color)
    #plt.pause(0.001)

def plot2DCurve(x1, y1, speedX, speedY, steeringAngle, angularAcc, color):
    theta = steeringAngle
    s = angularAcc
    x = x1
    y = y1
    dx = speedX
    dy = speedY

    for t in range(1,21):
        dt = 1.0/20.0
        thetaTot = math.atan2(dy, dx) + theta
        dx = dx + (dt * s * math.cos(thetaTot))
        dy = dy + (dt * s * math.sin(thetaTot))
        x = x + (dt * dx)
        y = y + (dt * dy)
        plt.plot([x1, x], [y1, y], color)
        x1 = x
        y1 = y
    #plt.pause(0.001)

try:
    # Define plot
    fig = plt.figure()
    fig.canvas.mpl_connect('close_event', on_close)
    ax = plt.axes(projection='3d')
    ax.set_zlim(0, 10)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.xaxis.set_ticks(np.arange(0, 10, 1))
    ax.yaxis.set_ticks(np.arange(0, 10, 1))
    ax.zaxis.set_ticks(np.arange(0, 10, 1))

    # Read in from stdin
    trajectoryNodes = []
    readingSolution = False
    for line in sys.stdin:
        vals = line.rstrip().split(' ')
        if len(vals) == 1 and not readingSolution:
            readingSolution = True
        elif len(vals) == 1 and readingSolution:
            readingSolution = False

        if len(vals) > 1 and readingSolution:
            trajectoryNodes.append(vals)
        if not readingSolution and len(vals) == 4:
            plot2DLine(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), '-k')
            #print("GOT LINE: (" + vals[0] + "," + vals[1] + ") -> (" + vals[2] + "," + vals[3] + ")")
        if not readingSolution and len(vals) == 6:
            #print("GOT CURVE: " + line)
            plot2DCurve(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), float(vals[4]), float(vals[5]), '-k')

    lastNode = []
    for node in trajectoryNodes:
        if len(node) == 2:
            plot2DPoint(float(node[0]), float(node[1]))
            #print("GOT TRAJECTORY POINT: (" + node[0] + "," + node[1] + ")")
            if len(lastNode) != 0:
                plot2DLine(float(lastNode[0]), float(lastNode[1]), float(node[0]), float(node[1]), '-r')
            lastNode = node
        if len(node) == 6:
            plot2DPoint(float(node[0]), float(node[1]))
            plot2DCurve(float(node[0]), float(node[1]), float(node[2]), float(node[3]), float(node[4]), float(node[5]), '-r')

    plt.show()
except:
    exit()