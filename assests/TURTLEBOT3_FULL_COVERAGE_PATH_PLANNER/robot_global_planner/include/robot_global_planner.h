#ifndef _ROBOTGLOBALPLANNER_H
#define _ROBOTGLOBALPLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/SetBool.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <robot_global_planner/PoseList.h>
#include <robot_global_planner/GetPlan.h>
#include <robot_global_planner/GetPlanResponse.h>
#include <iostream>
#include <vector>
#include <move_base/move_base.h>
#include <tf2/utils.h>

#include <pluginlib/class_loader.h>

bool is_active_cleaning;
namespace robot_global_planner
{
    class RobotGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        RobotGlobalPlanner();
        RobotGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

        ros::Publisher plan_pub_;
        ros::ServiceClient plan_srv;
        ros::ServiceServer cleaning_plan_service;
        bool is_start_point_reached;
        std::vector<geometry_msgs::PoseStamped> cleaning_plan;

    private:
        boost::shared_ptr<nav_core::BaseGlobalPlanner> bgp;
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        robot_global_planner::GetPlanResponse pose_list_;
        bool bgp_goal, align_goal;
        void straightPlan(geometry_msgs::PoseStamped &start, geometry_msgs::Pose &goal, std::vector<geometry_msgs::PoseStamped> &plan);
        static bool plan_cleaning(std_srvs::SetBoolRequest  &req,
                                        std_srvs::SetBoolResponse  &res);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
        bool initialized_;
        costmap_2d::Costmap2DROS *global_costmap;
        base_local_planner::CostmapModel *world_model_;
    };
};
#endif