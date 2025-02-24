// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
// #include <std_msgs/msg/detail/bool__struct.hpp>
#include "std_msgs/msg/bool.hpp"


namespace kiss_icp_ros {

class OdometryServer : public rclcpp::Node {
public:
    /// OdometryServer constructor
    OdometryServer() = delete;
    explicit OdometryServer(const rclcpp::NodeOptions &options);
    ~OdometryServer();

private:
    /// Register new frame
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    /// Stream the estimated pose to ROS
    void PublishOdometry(const Sophus::SE3d &kiss_pose, const std_msgs::msg::Header &header);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const std::vector<Eigen::Vector3d> frame,
                       const std::vector<Eigen::Vector3d> keypoints,
                       const std_msgs::msg::Header &header);

    void PublishIMUOdometry(const Sophus::SE3d &transform, const std_msgs::msg::Header &header);
    
    void VehicleMovingCallback(const std_msgs::msg::Bool::SharedPtr msg);

private:
    /// Tools for broadcasting TFs.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    bool invert_odom_tf_;
    bool publish_odom_tf_;
    bool publish_debug_clouds_;
    
    /// Data subscribers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr vel_est_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vehicle_moving_sub_;
    
    
    /// Data publishers.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr IMU_odom_publisher_;
    
    /// KISS-ICP
    std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
    
    /// Global/map coordinate frame.
    std::string lidar_odom_frame_{"hesai_lidar"};
    std::string base_frame_{"base_link"};
    std::string pointcloud_topic_{"/lidar_points"};

    /// Covariance diagonal
    double position_covariance_;
    double orientation_covariance_;

    // Odom->Base_link transformations
    Sophus::SE3d last_odom_to_base_link_pose_;

    bool is_vehicle_moving_{false};

    bool profiling_enabled_;
};

}  // namespace kiss_icp_ros
