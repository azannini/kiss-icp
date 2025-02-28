# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from launch.actions import LogInfo
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch_testing


# PARAMS --> only change this from Ouster to Hesai
# ouster = True

# The above automatically sets the following:
# topic = "ouster/points" if ouster else "/lidar_points"
# lidar_frame = "os_lidar" if ouster else "hesai_lidar"
# rviz_config = "kiss_icp_ouster.rviz" if ouster else "kiss_icp_hesai.rviz"

# This configuration parameters are not exposed thorught the launch system, meaning you can't modify
# those through the ros launch CLI. If you need to change these values, you could write your own
# launch file and modify the 'parameters=' block from the Node class.
class config:
    # Preprocessing
    max_range: float = 100.0
    min_range: float = 0.0
    deskew: bool = True

    #  Mapping parameters
    voxel_size: float = max_range / 100.0
    max_points_per_voxel: int = 20

    # Adaptive threshold
    initial_threshold: float = 2.0
    min_motion_th: float = 0.1

    # Registration
    max_num_iterations: int = 500
    convergence_criterion: float = 0.0001
    max_num_threads: int = 12

    # Covariance diagonal values
    position_covariance: float = 0.1
    orientation_covariance: float = 0.1


def generate_launch_description():
    # Change this from terminal 
    lidar_model = LaunchConfiguration("lidar_model", default="hesai")
    
    # ROS configuration
    visualize = LaunchConfiguration("visualize", default="true")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # Additional parameters
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame", default="hesai_lidar")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic", default="/perception/lidar_motion_compensator/compensated_pc")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default="true")
    invert_odom_tf = LaunchConfiguration("invert_odom_tf", default="false")
    max_range = LaunchConfiguration("max_range", default="100.0")
    min_range = LaunchConfiguration("min_range", default="0.0")
    deskew = LaunchConfiguration("deskew", default="false")
    max_points_per_voxel = LaunchConfiguration("max_points_per_voxel", default="20")
    voxel_size = LaunchConfiguration("voxel_size", default="1.0")
    initial_threshold = LaunchConfiguration("initial_threshold", default="2.0")
    min_motion_th = LaunchConfiguration("min_motion_th", default="0.1")
    max_num_iterations = LaunchConfiguration("max_num_iterations", default="500")
    convergence_criterion = LaunchConfiguration("convergence_criterion", default="0.0001")
    max_num_threads = LaunchConfiguration("max_num_threads", default="0")
    position_covariance = LaunchConfiguration("position_covariance", default="0.1")
    orientation_covariance = LaunchConfiguration("orientation_covariance", default="0.1")
    publish_debug_clouds = LaunchConfiguration("publish_debug_clouds", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    profiling_enabled = LaunchConfiguration("profiling_enabled", default="true")



    # KISS-ICP node variables
    package_name = "kiss_icp"
    executable_name = "kiss_icp_node"
    node_name = "kiss_icp_node"

    param_file_path = PathJoinSubstitution([
        FindPackageShare("kiss_icp"),
        "launch",
        "config",
        PythonExpression(["'launch_params_' + '", lidar_model, "' + '.yaml'"]),
    ])

    log_param_file = LogInfo(msg=["Using configuration file: ", param_file_path])

    # KISS-ICP node
    kiss_icp_node = Node(
        package=package_name,
        executable=executable_name,
        name=node_name,
        output="screen",
        parameters=[param_file_path],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("kiss_icp"), "rviz", "kiss_icp.rviz"]),
        ],
        condition=IfCondition(visualize),
    )

    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--rate", "1", bagfile, "--clock", "1000.0"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )

    return LaunchDescription(
        [
            log_param_file,
            kiss_icp_node,
            rviz_node,
            bagfile_play,
        ]
    )


