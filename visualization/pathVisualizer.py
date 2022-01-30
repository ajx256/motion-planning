
from cmath import cos, sin
from math import atan2
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation  as animation

import math
import sys

def on_close(event):
    plt.close()
    exit()

def plot2DPoint(x, y, points):
    points[0].append(x)
    points[1].append(y)
    points[2].append(0.0)
    
def plot2DLine(x1, y1, x2, y2, lines):
    lines.append([(x1, y1, 0.0), (x2, y2, 0.0)])

def plot2DCurve(x1, y1, speedX, speedY, steeringAngle, angularAcc, lines):
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
        lines.append([(x1, y1, 0.0), (x, y, 0.0)])
        x1 = x
        y1 = y

try:
    # Read in from stdin
    solution = []
    treeLines = []
    trajectoryLines = []
    solutionPoints = [[], [], []]
    readingSolution = False
    # Build up the lines for the search tree, and set aside the solution points for later...
    for line in sys.stdin:
        vals = line.rstrip().split(' ')
        if len(vals) == 1 and not readingSolution:
            readingSolution = True
        elif len(vals) == 1 and readingSolution:
            readingSolution = False
        elif len(vals) > 1 and readingSolution:
            solution.append(vals)
        elif not readingSolution and len(vals) == 4:
            plot2DLine(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), treeLines)
        elif not readingSolution and len(vals) == 6:
            plot2DCurve(float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), float(vals[4]), float(vals[5]), treeLines)

    # Build up both the solution trajectory as well as the points along it...
    lastNode = []
    for node in solution:
        if len(node) == 2:
            plot2DPoint(float(node[0]), float(node[1]), solutionPoints)
            if len(lastNode) != 0:
                plot2DLine(float(lastNode[0]), float(lastNode[1]), float(node[0]), float(node[1]), trajectoryLines)
            lastNode = node
        if len(node) == 6:
            plot2DPoint(float(node[0]), float(node[1]), solutionPoints)
            plot2DCurve(float(node[0]), float(node[1]), float(node[2]), float(node[3]), float(node[4]), float(node[5]), trajectoryLines)

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
    
    ax.add_collection(mplot3d.art3d.Line3DCollection(treeLines, color='k', lw=1))
    ax.add_collection(mplot3d.art3d.Line3DCollection(trajectoryLines, color='r', lw=3))

    plt.show()
except:
    exit()