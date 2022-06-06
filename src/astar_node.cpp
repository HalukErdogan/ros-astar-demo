/******************************************************************************
* MIT License
* 
* Copyright (c) 2022 Haluk Erdogan
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
******************************************************************************/

#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "astar/occupancy_map.h"
#include "astar/astar.h"

#define SUB_TOPIC_NAME_OCCUPANCY_MAP "/map"
#define SUB_TOPIC_NAME_START_STATE   "/initialpose"
#define SUB_TOPIC_NAME_TARGET_STATE  "/move_base_simple/goal"
#define PUB_TOPIC_NAME_INFLATED_MAP  "/inflated_map"
#define PUB_TOPIC_NAME_OPTIMAL_PATH  "/optimal_path"
#define DEFAULT_MAP_THRESHOLD_VALUE  50
#define DEFAULT_MAP_INFLATION_RADIUS 0.3  // meter


class GridWorld{
public:
    GridWorld();
    void ProcessQuery();

private:
    bool map_flag_ = false;
    bool source_point_flag_ = false;
    bool target_point_flag_ = false;
    bool start_planning_flag_ = false;
    tf2::Vector3 source_point_ = {0, 0, 0};
    tf2::Vector3 target_point_ = {0, 0, 0};
    astar::Astar planner_;
    std::shared_ptr<occupancy_map::OccupancyMap> map_ptr_;
    ros::NodeHandle node_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_source_point_;
    ros::Subscriber sub_target_point_;
    ros::Publisher pub_path_;
    ros::Publisher pub_inflated_map_;
    void CallbackMap_(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void CallbackSourcePoint_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void CallbackTargetPoint_(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void PublishPath_(const std::vector<tf2::Vector3> &path_w);
};

GridWorld::GridWorld(){
    // Initilize subscribers and publishers
    sub_map_          = node_.subscribe(SUB_TOPIC_NAME_OCCUPANCY_MAP, 1, &GridWorld::CallbackMap_, this);
    sub_source_point_ = node_.subscribe(SUB_TOPIC_NAME_START_STATE, 1, &GridWorld::CallbackSourcePoint_, this);
    sub_target_point_ = node_.subscribe(SUB_TOPIC_NAME_TARGET_STATE, 1, &GridWorld::CallbackTargetPoint_, this);
    pub_inflated_map_ = node_.advertise<nav_msgs::OccupancyGrid>(PUB_TOPIC_NAME_INFLATED_MAP, 1, this);
    pub_path_         = node_.advertise<nav_msgs::Path>(PUB_TOPIC_NAME_OPTIMAL_PATH, 1, this);

    // Get parameters
    ros::NodeHandle node_prv("~");  // private node for private params
    int occupancy_map_threshold;
    double occupancy_map_inflatiion_radius;
    node_prv.param<int>("occupancy_map_threshold", occupancy_map_threshold, DEFAULT_MAP_THRESHOLD_VALUE);
    node_prv.param<double>("occupancy_map_inflation_radius", occupancy_map_inflatiion_radius, DEFAULT_MAP_INFLATION_RADIUS);
                         
    // Create an occupancy map object with parameters
    map_ptr_ = std::make_shared<occupancy_map::OccupancyMap>(occupancy_map::OccupancyMap(occupancy_map_threshold, occupancy_map_inflatiion_radius));

    // Initiate planner
    planner_ = astar::Astar(map_ptr_);
}

void GridWorld::ProcessQuery(){
    if(start_planning_flag_ == true){
        std::vector<tf2::Vector3> path_w;
        std::chrono::_V2::system_clock::time_point start, stop;

        // Get beginning of planning time
        start = std::chrono::high_resolution_clock::now();

        // Plan
        planner_.Plan(source_point_, target_point_, path_w);

        // Get end of planning time
        stop = std::chrono::high_resolution_clock::now();
        
        // Calculate duration of planning
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        // Inform the user
        ROS_INFO("Planning is done in %.4f secs.", (double) duration.count() / 1e6);

        // Publish the path
        PublishPath_(path_w);
        
        // Set flags
        start_planning_flag_ = false;
    }
}

void GridWorld::CallbackMap_(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    // Update the map object
    map_ptr_->UpdateMap(msg);

    // Update planner 
    planner_.InitializeGraph();

    // Publish inflated map for rviz to visualize
    nav_msgs::OccupancyGrid inflated_map_msg;
    inflated_map_msg.header.frame_id = map_ptr_->GetFrameID();
    inflated_map_msg.header.stamp = ros::Time::now();
    map_ptr_->GetInflatedMapMsg(&inflated_map_msg);
    pub_inflated_map_.publish(inflated_map_msg);
    
    // Set flags
    map_flag_ = true;
    source_point_flag_ = false;
    target_point_flag_ = false;
    start_planning_flag_ = false;
}

void GridWorld::CallbackSourcePoint_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    source_point_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};

    // Set flags
    source_point_flag_ = true;
    if(map_flag_ && source_point_flag_ && target_point_flag_){
        start_planning_flag_ = true;
    }
}

void GridWorld::CallbackTargetPoint_(const geometry_msgs::PoseStamped::ConstPtr &msg){
    target_point_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

    // Set flags
    target_point_flag_ = true;
    if(map_flag_ && source_point_flag_ && target_point_flag_){
        start_planning_flag_ = true;
    }
}

inline void GridWorld::PublishPath_(const std::vector<tf2::Vector3> &path_w){
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_ptr_->GetFrameID();
    path_msg.header.stamp = ros::Time::now();
    for(auto &point_w : path_w){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point_w[0];
        pose.pose.position.y = point_w[1];
        pose.pose.position.z = point_w[2];
        path_msg.poses.push_back(pose);
    }
    pub_path_.publish(path_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "astar_node");
    GridWorld env;
    ros::Rate rate(20);
    while(ros::ok()){
        env.ProcessQuery();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
