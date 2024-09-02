#!/usr/bin/env python3
import matplotlib.pyplot as plt
from collections import deque
from queue import PriorityQueue
from numpy import arange
from occupancy_grid import OccupancyGridManager
import rospy



def connect_waypoints(waypoints):
    x_coords = [point[0] for point in waypoints]
    y_coords = [point[1] for point in waypoints]

    plt.plot(x_coords, y_coords, marker='o')
    plt.plot(x_coords, y_coords, linestyle='-')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Connect Waypoints')
    plt.grid(True)
    plt.show()

polygon_vertices = [(2.0, 4.0), (5.5, 5.0), (7.0, 4.0), (4.00 ,1.5),(2.0, 4.0)]
connect_waypoints(polygon_vertices)
