#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from occupancy_grid import OccupancyGridManager
from geometry_msgs.msg import Point
import math
import tf2_ros
import tf
from tf.transformations import quaternion_from_euler
from robot_global_planner.srv import GetPlan,GetPlanResponse
from robot_global_planner.msg import WayPoint
from numpy import arange
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionResult
import actionlib
from shapely.geometry import Point as SPoint, Polygon as SPolygon
import numpy as np
from math import radians,degrees
import dynamic_reconfigure.client
from geometry_msgs.msg import Quaternion,Vector3
from std_srvs.srv import SetBool,SetBoolRequest

def output_to_rviz(array, scale, color, current_goal=False):
    # global publisher_waypoints, publisher_current_goal
    global rviz_id 
    rviz_id = 0 

    # make MarkerArray message
    markerArray = MarkerArray()

    # loop throgh all instances of the array
    for index in range(len(array)):
        marker = Marker()
        marker.id = rviz_id
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.text = str(index)
        marker.action = marker.ADD
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        duration = -1
        marker.lifetime = rospy.Duration(duration)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 0, 0, 1.0)        # x and y are inverted due to nature of the map
        marker.pose.position.x = array[index][1]
        marker.pose.position.y = array[index][0]

        markerArray.markers.append(marker)
        rviz_id = rviz_id + 1

        publisher_waypoints.publish(markerArray)
        rospy.sleep(0.1)

def show_waypoints(goals,color=ColorRGBA(1.0, 0.0, 0.0, 0.80)):

    if not rospy.is_shutdown():
        rospy.loginfo('Inside show_waypoints')
        color = ColorRGBA()
        dimention = 0.15
        scale = Point(dimention * 2, dimention * 2, dimention * 2)
        # print(scale,color)
        output_to_rviz(goals, scale, color)

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(a, b, c):
    # Calculate the angle using the law of cosines
    angle_rad = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
    angle_deg = math.degrees(angle_rad)
    return angle_rad

def calculate_triangle_angles(x1, y1, x2, y2, x3, y3):
    # Calculate the lengths of the sides
    a = calculate_distance(x2, y2, x3, y3)
    b = calculate_distance(x1, y1, x3, y3)
    c = calculate_distance(x1, y1, x2, y2)

    # Calculate the angles using the law of cosines
    angle_A = calculate_angle(b, c, a)
    angle_B = calculate_angle(a, c, b)
    angle_C = calculate_angle(a, b, c)

    return angle_A, angle_B, angle_C

def point_filter(point, polygon_coordinates):
    """
    Check if a point is inside a polygon.

    Parameters:
    - point: Tuple representing the (x, y) coordinates of the point.
    - polygon_coordinates: List of tuples representing the vertices of the polygon.

    Returns:
    - True if the point is inside the polygon, False otherwise.
    """
    point = SPoint(point)
    polygon = SPolygon(polygon_coordinates)
    
    return polygon.contains(point)

def get_angle(pt1,pt2):
    return math.atan2(pt1[0]-pt2[0],pt1[1]-pt2[1])

def create_goals():
    goals = []
    # Subscribe to the nav_msgs/OccupancyGrid topic
    ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                               subscribe_to_updates=False)  # default False
    # tf_broadcaster = tf.TransformBroadcaster()
    size_x = 40
    size_y = 40
    count=0
    # Define the polygon vertices in counter-clockwise order
    # polygon_vertices = [(8.5, -2.75), (-5.40,-2.75), (-5.40, 3.25), (8.5, 3.25)]
    polygon_vertices = [(-0.52, 1.72), (1.78, 1.72), (1.78, -3.00), (-0.52, -3.00)]
    goal_list = list()
    reverse=1
    # polygon_vertices = [(-5.3, 3.25), (-5.5,-2.75), (8.5, -2.75), (8.5, 7.05), (6.00,7.05), (6.00, 3.10)]
    count=0
    last_row=int((polygon_vertices[1][0]-polygon_vertices[0][0])/exploration_point_dist)
    for x in arange(polygon_vertices[0][0],polygon_vertices[1][0],exploration_point_dist):
        _goal_list = list()
        for y in arange(polygon_vertices[2][1],polygon_vertices[0][1],exploration_point_dist):

            try:
                cost = ogm.get_cost_from_world_x_y(x, y)
                if(cost == 0):
                    _goal_list.append([y,x])
            except Exception as ex:
                pass

        if(len(_goal_list)==0):
                continue
        if(reverse):
            for i in range(len(_goal_list)):
                _goal_list[i].append(0)
            # if(count==0):
            #     _goal_list.insert(0,_goal_list[0])
            #     print("adding extras")
            #     _goal_list[0][2]=radians(90)
            # else:
            #     _goal_list.append(_goal_list[-1])
            #     # print("adding extras")
            #     _goal_list[-1][2]=radians(90)
            # _goal_list.insert(0,_goal_list[0])
            # _goal_list[0][2]=radians(90)
            if(count!=0):
                goal_list.append([_goal_list[-1][0],_goal_list[-1][1],radians(90)])
            goal_list.extend(_goal_list[::-1])
            goal_list.append([_goal_list[0][0],_goal_list[0][1],radians(90)])

        else:
            for i in range(len(_goal_list)):
                _goal_list[i].append(radians(180))
            goal_list.append([_goal_list[0][0],_goal_list[0][1],radians(90)])
            goal_list.extend(_goal_list)
            goal_list.append([_goal_list[-1][0],_goal_list[-1][1],radians(90)])
        if(count==last_row-1):
            goal_list.pop()

            # _goal_list.append(_goal_list[-1])
            # _goal_list[-1][2]=radians(90)
            # _goal_list.insert(0,_goal_list[0])
            # _goal_list[0][2]=radians(90)
            
        
        reverse=not(reverse)
        count+=1
    path_angle=math.atan2(goal_list[1][0] - goal_list[0][0],goal_list[1][1] - goal_list[0][1])
    compensated_angle= path_angle #+ radians(90)
    # print(goal_list)
    for i in range(len(goal_list)):
        goal_list[i][2]+=compensated_angle
        print(goal_list[i][1],goal_list[i][0],goal_list[i][2])
        
    return goal_list

def get_robot_pose():
    transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
    # Extract the position
    position = transform.transform.translation
    orientation = transform.transform.rotation

def waypoint_callback(srv):
    res=GetPlanResponse()
    for i in goal_list:
        goal=WayPoint()
        goal.pose.position.x=i[1]
        goal.pose.position.y=i[0]
        _,_,z,w=quaternion_from_euler(0.0,0.0,i[2])
        goal.pose.orientation.z=z
        goal.pose.orientation.w=w
        res.pose_list.append(goal)
    # print(res)
    return res

def publish_tf(timer):
    for i in range(len(goal_list)):
            # count+=1
            translation = (goal_list[i][1], goal_list[i][0], 0.0)
            rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_list[i][2])  # Roll, Pitch, Yaw in radians

            # Publish the transform
            tf_broadcaster.sendTransform(
                translation,
                rotation,
                rospy.Time.now(),
                f"waypoint_{i}",
                "map"
            )

def clean_room():
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(60))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_list[0][1]
    goal.target_pose.pose.position.y = goal_list[0][0]
    _,_,z,w=tf.transformations.quaternion_from_euler(0.0, 0.0, goal_list[0][2])  
    goal.target_pose.pose.orientation.z=z
    goal.target_pose.pose.orientation.w=w
    move_base.send_goal_and_wait(goal)
    while True:
        result=move_base.wait_for_result()
        if(result):
            break
    # params = {'sim_time' : 0.1}
    # movebase_client.update_configuration(params)
    cleaner_plan_service.call(SetBoolRequest(data=True))
    rospy.sleep(10)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_list[-1][1]
    goal.target_pose.pose.position.y = goal_list[-1][0]
    _,_,z,w=tf.transformations.quaternion_from_euler(0.0, 0.0, goal_list[-1][2])  
    goal.target_pose.pose.orientation.z=z
    goal.target_pose.pose.orientation.w=w
    move_base.send_goal_and_wait(goal)
    while True:
        result=move_base.wait_for_result()
        if(result):
            break
        # for i in goal_list:
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = 'map'
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position.x = i[1]
        # goal.target_pose.pose.position.y = i[0]
        # _,_,z,w=tf.transformations.quaternion_from_euler(0.0, 0.0, i[2])  
        # goal.target_pose.pose.orientation.z=z
        # goal.target_pose.pose.orientation.w=w
        # move_base.send_goal_and_wait(goal)
        # while True:
        #     result=move_base.wait_for_result()
        #     if(result):
        #         break
    return True

if __name__ == "__main__":
    global publisher_waypoints,exploration_point_dist,clear_costmap
    goal_list=[]
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.init_node("room_explorer")
    rospy.Service("/get_cleaning_waypoint",GetPlan,waypoint_callback)
    cleaner_plan_service=rospy.ServiceProxy("move_base/RobotGlobalPlanner/plan_cleaning",SetBool)
    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the transform from the base_link to the map frame
    try:
        tf_buffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get initial transform. Make sure the transform is being published.")
    
    client = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer", timeout = 10)
    # movebase_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout = 10)
    parameters = client.get_configuration()
    # print(parameters)
    params = {'inflation_radius' : 0.2}
    config = client.update_configuration(params)
    rospy.sleep(2)

    exploration_point_dist=rospy.get_param("exploration_point_dist",0.5)
    
    publisher_waypoints = rospy.Publisher(
            'visualization_marker_array_waypoints', MarkerArray, queue_size=10000)
    goal_list=create_goals()
    rospy.Timer(rospy.Duration(1),publish_tf)
    # clean_room()
    rospy.spin()