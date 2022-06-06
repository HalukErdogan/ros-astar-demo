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

#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <string>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

namespace occupancy_map{

class OccupancyMap{
private:
    int nrows_= 0;
    int ncols_ = 0;
    double resolution_ = 0;
    char threshold_ = 0;
    double inflation_radius_ = 0.0;
    std::string frame_id_ = "";
    cv::Mat1b map_image_;
    cv::Mat1b inflated_map_image_;
    tf2::Vector3 map_frame_position_w_;
    tf2::Quaternion map_frame_orientation_w_;
    tf2::Transform transform_world_to_map_;
    tf2::Transform transform_map_to_world_;
    
public:
    OccupancyMap(){}
    OccupancyMap(int occupancy_map_threshold, double occupancy_map_inflation_radius);
    void UpdateMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void GetInflatedMapMsg(nav_msgs::OccupancyGrid* msg);
    tf2::Vector3 MapToWorld(const tf2::Vector3 &point_m);
    tf2::Vector3 WorldToMap(const tf2::Vector3 &point_w);
    tf2::Vector3 MapIndexToWorld(int id);
    tf2::Vector3 MapIndicesToWorld(int row, int col);
    int WorldToMapIndex(const tf2::Vector3 &point_w);
    std::pair<int,int> WorldToMapIndices(const tf2::Vector3 &point_w);
    bool IsMapIndicesValid(const int &row, const int &col);
    bool IsMapIndexValid(const int &id);
    bool IsMapPointValid(const tf2::Vector3 &point_m);
    bool IsWorldPointValid(const tf2::Vector3 &point_w);
    std::string GetFrameID();
    int GetNumRows();
    int GetNumCols();
};

OccupancyMap::OccupancyMap(int occupancy_map_threshold, double occupancy_map_inflation_radius){
    threshold_        = occupancy_map_threshold;
    inflation_radius_ = occupancy_map_inflation_radius;
}

void OccupancyMap::UpdateMap(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    // Set frame id
    frame_id_ = msg->header.frame_id;

    // Set map metadata
    nrows_ = msg->info.height;
    ncols_ = msg->info.width;
    resolution_ = msg->info.resolution;
    map_frame_position_w_ = {
        msg->info.origin.position.x,
        msg->info.origin.position.y, 
        msg->info.origin.position.z};
    map_frame_orientation_w_ = {
        msg->info.origin.orientation.x, 
        msg->info.origin.orientation.y,
        msg->info.origin.orientation.z,
        msg->info.origin.orientation.w};
 
    // Set the transformation matrices
    transform_map_to_world_ = tf2::Transform(map_frame_orientation_w_, map_frame_position_w_);
    transform_world_to_map_ = transform_map_to_world_.inverse();

    // Check whether map image size is preserved
    if(map_image_.rows == nrows_ && map_image_.cols == ncols_){
        // If so set map image`s elements to zero
        map_image_.setTo(cv::Scalar(0));
        inflated_map_image_.setTo(cv::Scalar(0));
    }
    else{
        // Otherwise create another one from scratch
        map_image_ = cv::Mat(nrows_, ncols_, CV_8UC1, cv::Scalar(0));
        inflated_map_image_ = cv::Mat(nrows_, ncols_, CV_8UC1, cv::Scalar(0));
    }

    // Fill up the map image (Assumes unexplored areas are free)
    std::transform(
        msg->data.begin(), 
        msg->data.end(),
        map_image_.begin(), 
        [this](int8_t x) { return (x >= this->threshold_ || x == -1) ? true : false; });
    
    // Inflate the map image
    int kernel_radius = ceil(inflation_radius_/resolution_);
    cv::Mat kernel = cv::getStructuringElement( 
        cv::MORPH_ELLIPSE,
        cv::Size( 2*kernel_radius + 1, 2*kernel_radius + 1 ),
        cv::Point( kernel_radius, kernel_radius ) );
    cv::dilate(map_image_, inflated_map_image_, kernel);
}

void OccupancyMap::GetInflatedMapMsg(nav_msgs::OccupancyGrid* msg){
    // Info
    msg->info.height = nrows_;
    msg->info.width  = ncols_;
    msg->info.resolution = resolution_;
    msg->info.origin.position.x = map_frame_position_w_[0];
    msg->info.origin.position.y = map_frame_position_w_[1];
    msg->info.origin.position.z = map_frame_position_w_[2];
    msg->info.origin.orientation.w = map_frame_orientation_w_.getW();
    msg->info.origin.orientation.x = map_frame_orientation_w_.getX();
    msg->info.origin.orientation.y = map_frame_orientation_w_.getY();
    msg->info.origin.orientation.z = map_frame_orientation_w_.getZ();
    
    // Data
    msg->data.resize(inflated_map_image_.rows*inflated_map_image_.cols);
    std::transform(
        inflated_map_image_.begin(),
        inflated_map_image_.end(), 
        msg->data.begin(), [](uchar x) -> int8_t { return (x==true) ? 100 : 0;});
}

inline tf2::Vector3 OccupancyMap::WorldToMap(const tf2::Vector3 &point_w){
    return transform_world_to_map_*point_w;
}

inline tf2::Vector3 OccupancyMap::MapToWorld(const tf2::Vector3 &point_m){
    return transform_map_to_world_*point_m;
}

inline int OccupancyMap::WorldToMapIndex(const tf2::Vector3 &point_w){
    auto indices = WorldToMapIndices(point_w);
    return indices.first*ncols_ + indices.second;
}

inline std::pair<int,int> OccupancyMap::WorldToMapIndices(const tf2::Vector3 &point_w){
    tf2::Vector3 point_m = WorldToMap(point_w);
    if(std::abs(point_m[2]) > 1e-5){
        throw "Map point cannot have a value other then zero in third dimenson!";
    }

    // Get indices
    int col = point_m[0] / resolution_;     // cols of map allign with x direction of image
    int row = point_m[1] / resolution_;     // rows of map allign with y direction of image

    return {row, col};
}

inline tf2::Vector3 OccupancyMap::MapIndicesToWorld(int row, int col){
    double x = col * resolution_ + resolution_ / 2; // x alligns with cols
    double y = row * resolution_ + resolution_ / 2; // y alligns with rows
    tf2::Vector3 point_m = {x, y, 0};
    return MapToWorld(point_m);
}

inline tf2::Vector3 OccupancyMap::MapIndexToWorld(int id){
    int row = id / ncols_;
    int col = id - row*ncols_;
    return MapIndicesToWorld(row, col);
}

inline bool OccupancyMap::IsMapIndexValid(const int &id){
    int row = id / ncols_;
    int col = id - row * ncols_;
    return IsMapIndicesValid(row, col);
}

inline bool OccupancyMap::IsMapIndicesValid(const int &row, const int &col){
    return (row >= 0 
        && col >= 0
        && row < nrows_ 
        && col < ncols_
        && inflated_map_image_.at<bool>(row, col) == false);
}

inline bool OccupancyMap::IsMapPointValid(const tf2::Vector3 &point_m){
    int col = point_m[0] / resolution_;     // cols of map allign with x direction of image
    int row = point_m[1] / resolution_;     // rows of map allign with y direction of image
    if(std::abs(point_m[2]) > 1e-5){
        throw "Map point cannot have a value other then zero in third dimenson!";
    }

    return IsMapIndicesValid(row, col);
}

inline bool OccupancyMap::IsWorldPointValid(const tf2::Vector3 &point_w){
    tf2::Vector3 point_m = WorldToMap(point_w);
    return IsMapPointValid(point_m);
}

inline std::string OccupancyMap::GetFrameID(){
    return frame_id_;
}

inline int OccupancyMap::GetNumRows(){
    return nrows_;
}

inline int OccupancyMap::GetNumCols(){
    return ncols_;
}

}  // namespace occupancy_map
#endif // OCCUPANCY_H