# Copyright (c) 2019 AutonomouStuff, LLC
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import yaml


def launch_setup(context, *args, **kwargs):
    reader_params_path = LaunchConfiguration('reader_params_path').perform(context)
    with open(reader_params_path, 'r') as f:
        reader_params = yaml.safe_load(f)['/**']['ros__parameters']
    writer_params_path = LaunchConfiguration('writer_params_path').perform(context)
    with open(writer_params_path, 'r') as f:
        writer_params = yaml.safe_load(f)['/**']['ros__parameters']
    driver_params_path = LaunchConfiguration('driver_params_path').perform(context)
    with open(driver_params_path, 'r') as f:
        driver_params = yaml.safe_load(f)['/**']['ros__parameters']
    container = ComposableNodeContainer(
        name='pacmod3_with_kvaser',
        namespace='pacmod',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='kvaser_interface',
                plugin='kvaser_interface::KvaserReaderNode',
                name='kvaser_reader',
                namespace='pacmod',
                parameters=[reader_params]),
            ComposableNode(
                package='kvaser_interface',
                plugin='kvaser_interface::KvaserWriterNode',
                name='kvaser_writer',
                namespace='pacmod',
                parameters=[writer_params]),
            ComposableNode(
                package='pacmod3',
                plugin='pacmod3::PACMod3Node',
                name='pacmod3_driver',
                namespace='pacmod',
                parameters=[driver_params])
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    pacmod3_share_dir = get_package_share_directory('pacmod3')
    kvaser_share_dir = get_package_share_directory('kvaser_interface')
    return LaunchDescription([
        DeclareLaunchArgument('driver_params_path',
                              default_value=os.path.join(pacmod3_share_dir,
                                                         'launch',
                                                         'driver_params.yaml')),
        DeclareLaunchArgument('reader_params_path',
                              default_value=os.path.join(kvaser_share_dir,
                                                         'config',
                                                         'reader_params.yaml')),
        DeclareLaunchArgument('writer_params_path',
                              default_value=os.path.join(kvaser_share_dir,
                                                         'config',
                                                         'writer_params.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
