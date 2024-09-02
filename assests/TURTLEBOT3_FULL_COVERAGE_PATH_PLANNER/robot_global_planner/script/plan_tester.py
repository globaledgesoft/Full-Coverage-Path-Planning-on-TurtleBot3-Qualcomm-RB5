#!/usr/bin/env python3
import matplotlib.pyplot as plt
from collections import deque
from numpy import arange
from occupancy_grid import OccupancyGridManager
import rospy
from shapely.geometry import Point as SPoint, Polygon as SPolygon

def point_filter(point, polygon_coordinates):
    point = SPoint(point)
    polygon = SPolygon(polygon_coordinates)
    return polygon.contains(point)

def find_path(matrix, start, end):
    rows = len(matrix)
    cols = len(matrix[0])
    visited = set()
    queue = deque([(start, [])])

    while queue:
        current, path = queue.popleft()
        if current == end:
            return path + [end]

        visited.add(current)
        row, col = current

        # Define possible movements: up, down, left, right
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < rows and 0 <= new_col < cols and matrix[new_row][new_col] != 1 and (new_row, new_col) not in visited:
                queue.append(((new_row, new_col), path + [current]))


    return None  # Path not found


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

rospy.init_node("planner_tester")
exploration_point_dist=0.5
ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                               subscribe_to_updates=False)  

# polygon_vertices = [(-0.52, 1.72), (1.78, 8.72), (1.78, -3.00), (-0.52, -3.00)]
polygon_vertices = [(2.0, 4.0), (5.5, 5.0), (7.0, 4.0), (4.00 ,1.5),(2.0, 4.0)]
# polygon_vertices = [(0.0, 5.00), (3.0, 5.0), (3.0, 3.0), (0.0, 3.0)]
x_values = [vertex[0] for vertex in polygon_vertices]
y_values = [vertex[1] for vertex in polygon_vertices]
min_x=min(x_values)
max_x=max(x_values)
min_y=min(y_values)
max_y=max(y_values)
print(min_x,max_x,min_y,max_y)
goal_list = list()
reverse=1
# polygon_vertices = [(-5.3, 3.25), (-5.5,-2.75), (8.5, -2.75), (8.5, 7.05), (6.00,7.05), (6.00, 3.10)]
count=0
matrix=[]
cost_matrix=[]
last_row=int((polygon_vertices[1][0]-polygon_vertices[0][0])/exploration_point_dist)
polygon=SPolygon(polygon_vertices)
for x in arange(min_x,max_x,exploration_point_dist):
    #polygon_vertices[0][0],polygon_vertices[1][0],exploration_point_dist):
    _goal_list = list()
    _goal_matrix = list()
    _cost_matrix = list()
    for y in arange(min_y,max_y,0.25):
        #polygon_vertices[2][1],polygon_vertices[0][1],exploration_point_dist):
        try:
            cost = ogm.get_cost_from_world_x_y(x, y)
            _point=(x,y)
            if(cost == 0 and point_filter(_point,polygon)):
                _goal_matrix.append([x,y])
                _goal_list.append([x,y])
                _cost_matrix.append(0)
            else:
                _cost_matrix.append(1)
                _goal_matrix.append(None)
        except Exception as ex:
            pass
    print(matrix)
    matrix.extend(_goal_matrix)
    cost_matrix.append(_cost_matrix)

i=0
full_path=[]
column_len=len(cost_matrix[0])
start_point=None
while(i<len(matrix)):
    while(True):
        row=i//column_len
        column=i%column_len
        if(row%2!=0):
            j=(row*column_len)+(column_len-column)
        else:
            j=i
        # print(j)
        if(i>=len(matrix)):
            break
        if(j>=len(matrix)):
            i+=1
            continue
        if(isinstance(matrix[j],list)):
            break

        i+=1
    row=j//column_len
    column=j%column_len
    if(start_point==None):
        start_point = (row,column)
        i+=1
        print("start intialized")
        continue
    end_point=(row,column)
    path = find_path(cost_matrix, start_point, end_point)
    # print(path)
    if(path!=None):
        full_path.extend(path)
    start_point=(row,column)
    i+=1
goals_with_angle=[]
directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
for i in range(0,len(full_path)):
    if(i<len(full_path)-1):
        x,y=full_path[i][0]-full_path[i+1][0],full_path[i][1]-full_path[i+1][1]
    if(x==1 and y==0):
        _point=[full_path[i][0],full_path[i][1],1.57]
    elif(x==-1 and y==0):
        _point=[full_path[i][0],full_path[i][1],-1.57]
    elif(x==0 and y==1):
        _point=[full_path[i][0],full_path[i][1],0]
    elif(x==0 and y==-1):
        _point=[full_path[i][0],full_path[i][1],3.14]
    if(i==0):
        goals_with_angle.append(_point)
        continue
    # goals_with_angle.append(_point)
    else:
        if(goals_with_angle[-1][0]==_point[0] and goals_with_angle[-1][1]==_point[1] and goals_with_angle[-1][2]==_point[2]):
            continue
        if(goals_with_angle[-1][2]!=_point[2]):
            goals_with_angle.append([_point[0],_point[1],goals_with_angle[-1][2]])
            goals_with_angle.append(_point)        
        else:
            goals_with_angle.append(_point)        

# print(goals_with_angle)
connect_waypoints(full_path)
