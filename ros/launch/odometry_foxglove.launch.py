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



import os
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
# from launch import LaunchDescription
# from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    
    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")


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

    visualization = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch/foxglove_bridge_launch.xml",
            )
        )
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
            visualization,
            bagfile_play,
        ]
    )
