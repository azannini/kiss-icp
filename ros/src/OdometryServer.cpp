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
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <easy/profiler.h>

#include <filesystem> 
#include <fstream>

namespace {
Sophus::SE3d LookupTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
    std::string err_msg;
    if (tf2_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "Failed to find tf. Reason=%s",
                err_msg.c_str());
    // default construction is the identity
    return Sophus::SE3d();
}
}  // namespace

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("kiss_icp_node", options) {
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    lidar_odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", lidar_odom_frame_);
    pointcloud_topic_ = declare_parameter<std::string>("PC_config.pointcloud_topic", pointcloud_topic_);
    publish_odom_tf_ = declare_parameter<bool>("ICP_config.publish_odom_tf", publish_odom_tf_);
    invert_odom_tf_ = declare_parameter<bool>("ICP_config.invert_odom_tf", invert_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("PC_config.publish_debug_clouds", publish_debug_clouds_);
    position_covariance_ = declare_parameter<double>("ICP_config.position_covariance", 0.1);
    orientation_covariance_ = declare_parameter<double>("ICP_config.orientation_covariance", 0.1);
    profiling_enabled_= declare_parameter<bool>("profiling_enabled", profiling_enabled_);
    
    kiss_icp::pipeline::KISSConfig config;
    config.max_range = declare_parameter<double>("ICP_config.max_range", config.max_range);
    config.min_range = declare_parameter<double>("ICP_config.min_range", config.min_range);
    config.deskew = declare_parameter<bool>("PC_config.deskew", config.deskew);
    config.voxel_size = declare_parameter<double>("ICP_config.voxel_size", config.max_range / 100.0);
    config.max_points_per_voxel =
        declare_parameter<int>("ICP_config.max_points_per_voxel", config.max_points_per_voxel);
    config.initial_threshold =
        declare_parameter<double>("ICP_config.initial_threshold", config.initial_threshold);
    config.min_motion_th = declare_parameter<double>("ICP_config.min_motion_th", config.min_motion_th);
    config.max_num_iterations =
        declare_parameter<int>("ICP_config.max_num_iterations", config.max_num_iterations);
    config.convergence_criterion =
        declare_parameter<double>("ICP_config.convergence_criterion", config.convergence_criterion);
    config.max_num_threads = declare_parameter<int>("ICP_config.max_num_threads", config.max_num_threads);
    if (config.max_range < config.min_range) {
        RCLCPP_WARN(get_logger(),
                    "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config.min_range = 0.0;
    }
    //TODO(Andrea): check if this makes sense or not (Do I need this KISS_CONFIG?)
    // config.initial_guess_mode = static_cast<kiss_icp::pipeline::KISSConfig::InitialGuess>(declare_parameter<int>("ICP_config.initial_guess_mode", config.initial_guess_mode));
    config.profiling_enabled_ = profiling_enabled_;

    // Construct the main KISS-ICP odometry node
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);

    // Initialize subscribers
    // pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "pointcloud_topic", rclcpp::SensorDataQoS(),
    //     std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(), // other topic: /perception/lidar_motion_compensator/compensated_pc
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    
    vel_est_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "tf", rclcpp::SystemDefaultsQoS(),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            for (const auto &transform : msg->transforms) {
                if (transform.header.frame_id == "odom" && transform.child_frame_id == base_frame_) {
                    // Process the transform
                    // RCLCPP_INFO(this->get_logger(), "Received transform from odom to base_link");
                      this->PublishIMUOdometry(tf2::transformToSophus(transform), transform.header);
                }
            }
        });
    
    vehicle_moving_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/moving", 10,
        std::bind(&OdometryServer::VehicleMovingCallback, this, std::placeholders::_1));
    
    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("kiss/odometry", qos);
    IMU_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("kiss/IMU_odometry", qos);
    if (publish_debug_clouds_) {
        frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/frame", qos);
        kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/keypoints", qos);
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/local_map", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    /** \brief Store static transformations as variable for efficiency
     * LookupTransform(target_frame, source_frame, tf2_buffer_)
     * \param target_frame (frame_id) The frame to which data should be transformed
     * \param source_frame (child_frame_id) The frame where the data originated
    */
    //TODO(Andrea): check if with new rosbags it's correct to do it here or not
    // base_link_to_hesai_static_transform_ = LookupTransform("map", "hesai_lidar_static" , tf2_buffer_);
    
    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");

    if(profiling_enabled_){
        EASY_PROFILER_ENABLE;
        // profiler::startListen();
        RCLCPP_INFO(this->get_logger(), "Easy Profiler started");
    } else {
        RCLCPP_INFO(this->get_logger(), "Easy Profiler disabled --> profiling_enabled set to False");
    }
    
}

OdometryServer::~OdometryServer() {
    // Save profiler and CSV trajectory for KPIs and debugging
    // - kiss_icp_ws
    //      - run_2025-02-20_13-24-38
    //      |--- profiler.prof
    //      |--- trajectory.csv
    //      |--- launch_params_hesai.yaml
    if(profiling_enabled_){   
        // Determine output directories based on current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm *tm_now = std::localtime(&time_now);
        
        // TODO(Andrea): this approach works only if you start node from the root of the workspace
        std::ostringstream run_dir_stream;
        run_dir_stream << std::filesystem::current_path().string() << "/results/run_"
                << std::put_time(tm_now, "%Y-%m-%d_%H-%M-%S");
        std::string run_dir = run_dir_stream.str();
        std::filesystem::create_directories(run_dir);
        
        // Define file paths for profiler and trajectory data
        std::string profiler_file = run_dir + "/profiler.prof";
        std::string trajectory_file = run_dir + "/trajectory.txt";
        std::string ICP_iterations_file = run_dir + "/ICP_iterations.csv";
        std::string ICP_thresholds_file = run_dir + "/ICP_thresholds.csv";
        // std::string launch_params_file = run_dir + "/launch_params_hesai.yaml";
        std::string launch_params_file = run_dir + "/launch_params_ouster.yaml";

        
        // Save profiling data to file
        profiler::dumpBlocksToFile(profiler_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Profiling data saved to %s", profiler_file.c_str());
        
        // Save trajectory CSV data to file
        std::ofstream file(trajectory_file);
        file << "# timestamp x y z q_x q_y q_z q_w\n";
        for (const auto &pose : trajectory_) {
            file << std::fixed << std::setprecision(4) << pose[0] << " "; // Print timestamp with full precision
            for (size_t i = 1; i < pose.size(); ++i) {
            file << pose[i];
            if (i < pose.size() - 1) {
                file << " ";
            }
            }
            file << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory data saved to %s", trajectory_file.c_str());

      
        // Save ICP iterations to a CSV file
        std::ofstream icp_iterations_file(ICP_iterations_file);
        icp_iterations_file << "# mean_iterations: " << kiss_icp_->mean_ICP_iterations() << "\n";
        icp_iterations_file << "iteration,num_iterations\n";
        const auto &iterations = kiss_icp_->num_iterations_ICP();
        for (size_t i = 0; i < iterations.size(); ++i) {
            icp_iterations_file << i << "," << iterations[i] << "\n";
        }
        icp_iterations_file.close();

        RCLCPP_INFO(this->get_logger(), "ICP iterations saved to %s", ICP_iterations_file.c_str());
        
        
        // Save ICP thresholds to a CSV file
        std::ofstream icp_thresholds_file(ICP_thresholds_file);
        icp_thresholds_file << "iteration,threshold\n";
        const auto &thresholds = kiss_icp_->max_distance_ICP();
        for (size_t i = 0; i < thresholds.size(); ++i) {
            icp_thresholds_file << i << "," << thresholds[i] << "\n";
        }
        icp_thresholds_file.close();
        RCLCPP_INFO(this->get_logger(), "ICP thresholds saved to %s", ICP_thresholds_file.c_str());


        // Save launch_params_hesai.yaml to the run directory
        std::filesystem::copy("install/kiss_icp/share/kiss_icp/launch/config/launch_params_hesai.yaml", launch_params_file);
        RCLCPP_INFO(this->get_logger(), "Launch parameters saved to %s", launch_params_file.c_str());
    }else{
        RCLCPP_INFO(this->get_logger(), "Easy Profiler disabled. No profiling data has been saved.");
    }

    std::cout << "ICP mean iterations before convergence: " << kiss_icp_->mean_ICP_iterations() << std::endl;
}

void OdometryServer::VehicleMovingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // Once set to true, do not change
    is_vehicle_moving_ = is_vehicle_moving_ || msg->data;
};

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    //TODO(Andrea): we should move this with the initializer, but problems with rosbag (tf not found at the beginning)
    // base_link_to_hesai_static_transform_ = LookupTransform("map", "hesai_lidar_static" , tf2_buffer_);
    // hesai_to_base_link_static_transform_ = base_link_to_hesai_static_transform_.inverse();
   
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = GetTimestamps(msg);

    // Retrieve delta_pose (current - previous odom_to_base_link)
    const auto odom_to_base_link_pose = LookupTransform("odom", base_frame_, tf2_buffer_);
    const auto delta_pose_odom_frame = last_odom_to_base_link_pose_.inverse() * odom_to_base_link_pose;
    const auto delta_pose_hesai_frame = hesai_to_base_link_static_transform_ * delta_pose_odom_frame * base_link_to_hesai_static_transform_;
    last_odom_to_base_link_pose_ = odom_to_base_link_pose;
    // const auto delta_pose_hesai_frame = hesai_to_base_link_static_transform_ * delta_pose_odom_frame * base_link_to_hesai_static_transform_;
    
    // const auto delta_pose_odom_frame = last_odom_to_base_link_pose_.inverse() * odom_to_base_link_pose;


    // Register frame, main entry point to KISS-ICP pipeline
    // frame: original voxelized map
    // keypoints(source): features used to update ICP [subsample of frame_downsample]
    // delta_pose:
    const auto &[frame, keypoints] = kiss_icp_->RegisterFrame(is_vehicle_moving_, delta_pose_hesai_frame, points, timestamps);
    
    // Extract the last hesai_lidar_static->hesai_lidar KISS-ICP pose
    const Sophus::SE3d kiss_pose = kiss_icp_->pose();

    // Eigen::Quaterniond quaternion = kiss_pose.unit_quaternion();
    // Eigen::Vector4d coeffs = quaternion.coeffs();

    // RCLCPP_INFO(get_logger(), "Kiss_pose_quaternion: [%.4f, %.4f, %.4f, %.4f]",
    //             coeffs.x(), coeffs.y(), coeffs.z(), coeffs.w());
    // RCLCPP_INFO(get_logger(), "Kiss_pose_translation: [%.4f, %.4f, %.4f]",
    //             kiss_pose.translation().x(), kiss_pose.translation().y(), kiss_pose.translation().z());


    // Spit the current estimated pose to ROS msgs handling the desired target frame
    PublishOdometry(kiss_pose, msg->header);

    // Publishing these clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header);
    }
}


void OdometryServer::PublishIMUOdometry(const Sophus::SE3d &transform, const std_msgs::msg::Header &header){
    // TODO(Andrea): use variables instead of strings here
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(transform);

    odom_msg.pose.covariance.fill(0.0);
    IMU_odom_publisher_->publish(std::move(odom_msg));
}


void OdometryServer::PublishOdometry(const Sophus::SE3d &kiss_pose,
                                     const std_msgs::msg::Header &header) {
    // Now publishing on /tf2 map->odom instead of map->base_link. map->base_link is published as kiss/odometry
    // map -> bl = map -> hesai_lidar_static * hesai_lidar_static -> hesai_lidar * hesai_lidar -> bl
    const auto map_to_base_link_pose = base_link_to_hesai_static_transform_ * kiss_pose * hesai_to_base_link_static_transform_;
    // const auto map_to_base_link_pose = kiss_pose;
    
    // Compute the map -> odom correction: map->odom = map->base_link · (odom->base_link)⁻¹ so that map->base_link = map->odom * odom->base_link
    const auto map_to_odom_pose = map_to_base_link_pose * last_odom_to_base_link_pose_.inverse();

    // Broadcast the map -> odom transform on /tf2
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = header.stamp;
    map_to_odom_tf.header.frame_id = "map";
    map_to_odom_tf.child_frame_id = "odom";
    map_to_odom_tf.transform = tf2::sophusToTransform(map_to_odom_pose);
    RCLCPP_DEBUG(get_logger(), "Broadcasting transform from %s to %s",
                 map_to_odom_tf.header.frame_id.c_str(), map_to_odom_tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(map_to_odom_tf);

    
    // publish KISS-ICP odometry msg on topic /kiss/odometry
    nav_msgs::msg::Odometry map_to_bl_msg;
    map_to_bl_msg.header.stamp = header.stamp;
    map_to_bl_msg.header.frame_id = "map";
    map_to_bl_msg.child_frame_id = base_frame_;
    map_to_bl_msg.pose.pose = tf2::sophusToPose(map_to_base_link_pose);
    map_to_bl_msg.pose.covariance.fill(0.0);
    map_to_bl_msg.pose.covariance[0] = position_covariance_;
    map_to_bl_msg.pose.covariance[7] = position_covariance_;
    map_to_bl_msg.pose.covariance[14] = position_covariance_;
    map_to_bl_msg.pose.covariance[21] = orientation_covariance_;
    map_to_bl_msg.pose.covariance[28] = orientation_covariance_;
    map_to_bl_msg.pose.covariance[35] = orientation_covariance_;
    odom_publisher_->publish(std::move(map_to_bl_msg));

    // Save pose data to trajectory_ for KPI CSV export.
    // Only the following values are saved: time_sec, time_nsec, x, y, z, qx, qy, qz, qw
    trajectory_.emplace_back(std::array<double, 8>{
        static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) / 1e9,
        map_to_bl_msg.pose.pose.position.x,
        map_to_bl_msg.pose.pose.position.y,
        map_to_bl_msg.pose.pose.position.z,
        map_to_bl_msg.pose.pose.orientation.x,
        map_to_bl_msg.pose.pose.orientation.y,
        map_to_bl_msg.pose.pose.orientation.z,
        map_to_bl_msg.pose.pose.orientation.w
    });
}

void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> frame,
                                   const std::vector<Eigen::Vector3d> keypoints,
                                   const std_msgs::msg::Header &header) {
    const auto kiss_map = kiss_icp_->LocalMap();
    
    // Publish frame and keypoints in the sensor's frame ("hesai_lidar")
    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, header)));

    // Publish the local map in the "hesai_lidar_static" frame because the map represents the global reference.
    auto local_map_header = header;
    local_map_header.frame_id = lidar_odom_frame_;
    map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, local_map_header)));
}
}  // namespace kiss_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_icp_ros::OdometryServer)
