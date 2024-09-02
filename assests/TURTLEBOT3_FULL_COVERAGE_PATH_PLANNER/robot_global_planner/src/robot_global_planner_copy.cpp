#include <pluginlib/class_list_macros.h>
#include "robot_global_planner.h"
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>

PLUGINLIB_EXPORT_CLASS(robot_global_planner::RobotGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace robot_global_planner
{
RobotGlobalPlanner::RobotGlobalPlanner ():
bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
world_model_(NULL),initialized_(false)
{}

RobotGlobalPlanner::RobotGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
     {
        initialize(name, costmap_ros);
     }
bool RobotGlobalPlanner::plan_cleaning(std_srvs::SetBoolRequest  &req,
                                        std_srvs::SetBoolResponse  &res)
{
    ROS_INFO_STREAM("plan_cleaning_called with: "<<req.data);
    is_active_cleaning=req.data;
    std::vector<geometry_msgs::PoseStamped> bgp_plan,np_stamped_list;
    std::vector<geometry_msgs::Pose> next_pose_list;
    geometry_msgs::PoseStamped np_stamped,last_np_stamped;
    int list_size,global_plan_start_index;
    robot_global_planner::GetPlan srv;
    // srv.request.start = start;
    if (plan_srv.call(srv))
    {
        pose_list_.pose_list = srv.response.pose_list;
        list_size = pose_list_.pose_list.size();
    }
    if(is_active_cleaning)
    {   
            if (list_size > 0)
            {
            last_np_stamped.pose = pose_list_.pose_list[0].pose;
            int i = 1;
            for (; i < list_size; i++)
                {
                    double dist=distance(last_np_stamped.pose,pose_list_.pose_list[i].pose);
                    if(dist<=0.05){
                        break;
                    }
                }
            for (; i < list_size; i++)
                {
                    geometry_msgs::Pose next_pose;
                    next_pose = pose_list_.pose_list[i].pose;
                    // next_pose_list.push_back(next_pose);
                    np_stamped.header.stamp = ros::Time::now();
                    np_stamped.header.frame_id = "map";
                    np_stamped.pose.position.x = next_pose.position.x;
                    np_stamped.pose.position.y = next_pose.position.y;
                    np_stamped.pose.orientation.x = next_pose.orientation.x;
                    np_stamped.pose.orientation.y = next_pose.orientation.y;
                    np_stamped.pose.orientation.z = next_pose.orientation.z;
                    np_stamped.pose.orientation.w = next_pose.orientation.w;
                    np_stamped_list.push_back(np_stamped);
                    if((abs(pose_list_.pose_list[i].pose.position.x-pose_list_.pose_list[i-1].pose.position.x)>0.5) or 
                    ((abs(pose_list_.pose_list[i].pose.position.y-pose_list_.pose_list[i-1].pose.position.y)>0.5)))
                    {
                        bgp->makePlan(last_np_stamped, np_stamped, bgp_plan);
                        for (int j = 0; j< bgp_plan.size(); j++)
                        {
                            cleaning_plan.push_back(bgp_plan[j]);
                        }
                    }
                    else
                    {
                        straightPlan(last_np_stamped,next_pose,cleaning_plan);
                    }
                }
                last_np_stamped = np_stamped;
                
            }
            else
            {
                cout << "Received empty plan. Giving starting pose as goal\n";
                cleaning_plan.clear();
            }
            // for(auto i : plan)
            // {
            //     cleaning_plan.push_back(i);
            // }
            publishcleanerPlan(cleaning_plan);
    }
    res.success=true;
    return true;

}

double quaternionToEuler(const double z,const double w) {
    double yaw;
    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z);
    double cosy_cosp = 1.0 - 2.0 * (z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}
void eulerToQuaternion(const double yaw, double& z, double& w) {

    w = std::cos(yaw * 0.5);
    z = std::sin(yaw * 0.5);
    // double cp = std::cos(0.0);
    // double sp = std::sin(0.0);
    // double cr = std::cos(0.0);
    // double sr = std::sin(0.0);

    // q.w = cy;
    // q.z = sy * cp * cr - cy * sp * sr;

    return ;
}
void RobotGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
     {
        if (!initialized_){
            global_costmap = costmap_ros;
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            cleaner_plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleanerplan", 1);
            cleaning_plan_service=private_nh.advertiseService("plan_cleaning",plan_cleaning);
            plan_srv = private_nh.serviceClient<robot_global_planner::GetPlan>("/get_cleaning_waypoint");
            std::string global_planner;
            // cleaning_plan.empty();
            global_planner = "global_planner/GlobalPlanner";
            is_active_cleaning=false;
            is_start_point_reached=false;
            try
            {
                bgp = bgp_loader_.createInstance(global_planner);
                bgp->initialize(bgp_loader_.getName(global_planner), costmap_ros);
            }
            catch (pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            }
            try
            {
                world_model_ = new base_local_planner::CostmapModel(*global_costmap->getCostmap());
            }
            catch (bool error)
            {
                ROS_ERROR("World Model failed to load for some reason");
            }
            initialized_ = true;
         }
     }
double RobotGlobalPlanner::distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
     {
         return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
     }     

void RobotGlobalPlanner::straightPlan(geometry_msgs::PoseStamped& start, geometry_msgs::Pose& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        double vector[2],mag;
        mag = distance(start.pose,goal);
        vector[0] = (goal.position.x - start.pose.position.x) / mag ;
        vector[1] = (goal.position.y - start.pose.position.y) / mag ;
        while(distance(start.pose,goal) > 0.01 ){
            plan.push_back(start);
            start.pose.position.x += 0.01 * vector[0];
            start.pose.position.y += 0.01 * vector[1];
            start.header.stamp = ros::Time::now();
            start.pose.orientation.x = 0;
            start.pose.orientation.y = 0;
            start.pose.orientation.z = goal.orientation.z;
            start.pose.orientation.w = goal.orientation.w;
       }
       double d_angle= quaternionToEuler(start.pose.orientation.z,start.pose.orientation.w)-quaternionToEuler(goal.orientation.z,goal.orientation.w);
       int sign= d_angle < 0 ? -1 : 1; 
       ROS_INFO("dangle______________%f",d_angle);
       while(abs(d_angle)>0.0){
            start.header.stamp = ros::Time::now();
            double d_yaw = quaternionToEuler(start.pose.orientation.z,start.pose.orientation.w);
            d_yaw-=(0.01 * sign);
            double z;
            double w;
            eulerToQuaternion(d_yaw,z,w);
            start.pose.orientation.x = 0;
            start.pose.orientation.y = 0;
            start.pose.orientation.z = z;
            start.pose.orientation.w = w;
            d_angle+=(0.01 * sign);
            ROS_INFO("dvalue %d %f",sign, d_yaw);
            break;
       }
       
}

bool RobotGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {   
        
        publishPlan(plan); 
        return true;       
                 
    }

 void RobotGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "/map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}
 void RobotGlobalPlanner::publishcleanerPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "/map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    cleaner_plan_pub_.publish(gui_path);
}
};
