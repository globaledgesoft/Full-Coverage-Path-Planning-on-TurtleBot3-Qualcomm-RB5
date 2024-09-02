#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,PoseStamped

def main():
    rospy.init_node("Intial_pose_estimator")
    cmd_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    cleaning_starter=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)
    cmd_vel=Twist()
    cmd_vel.angular.z=2.0
    # rospy.sleep(5)
    start_time=rospy.Time.now().secs
    rate=rospy.Rate(10)
    while(True):
        cmd_pub.publish(cmd_vel)
        current_time=rospy.Time.now().secs
        if(current_time-start_time>=20):
            break
        rate.sleep()
    # rospy.sleep(20)
    cmd_vel.angular.z=0.0
    cmd_pub.publish(cmd_vel)
    rospy.sleep(2)

    #start cleaner planing
    goal_cmd=PoseStamped()
    goal_cmd.header.frame_id = 'map'
    goal_cmd.header.stamp = rospy.Time.now()
    goal_cmd.pose.orientation.w=1.0
    cleaning_starter.publish(goal_cmd)
    
if __name__=="__main__":
    main()