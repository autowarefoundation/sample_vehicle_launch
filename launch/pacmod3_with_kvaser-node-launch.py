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

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState, matches_node_name

import lifecycle_msgs.msg


def generate_launch_description():
    pacmod3_share_dir = get_package_share_directory('pacmod3')
    kvaser_share_dir = get_package_share_directory('kvaser_interface')

    pacmod3_driver_node = LifecycleNode(package='pacmod3',
                                        executable='pacmod3_driver',
                                        name='pacmod3_driver',
                                        namespace='pacmod',
                                        parameters=[LaunchConfiguration('driver_params_path')],
                                        output='screen')

    kvaser_reader_node = LifecycleNode(package='kvaser_interface',
                                       executable='kvaser_reader_node_exe',
                                       name='kvaser_reader',
                                       namespace='pacmod',
                                       parameters=[LaunchConfiguration('reader_params_path')],
                                       output='screen')

    kvaser_reader_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/pacmod/kvaser_reader'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    kvaser_reader_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/pacmod/kvaser_reader'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    kvaser_writer_node = LifecycleNode(package='kvaser_interface',
                                       executable='kvaser_writer_node_exe',
                                       name='kvaser_writer',
                                       namespace='pacmod',
                                       parameters=[LaunchConfiguration('writer_params_path')],
                                       output='screen')

    kvaser_writer_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/pacmod/kvaser_writer'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    kvaser_writer_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/pacmod/kvaser_writer'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('driver_params_path',
                              default_value=os.path.join(pacmod3_share_dir,
                                                         'launch',
                                                         'driver_params.yaml')),
        DeclareLaunchArgument('reader_params_path',
                              default_value=os.path.join(kvaser_share_dir,
                                                         'launch',
                                                         'reader_params.yaml')),
        DeclareLaunchArgument('writer_params_path',
                              default_value=os.path.join(kvaser_share_dir,
                                                         'launch',
                                                         'writer_params.yaml')),
        pacmod3_driver_node,
        kvaser_reader_node,
        kvaser_reader_configure_trans_event,
        kvaser_reader_activate_trans_event,
        kvaser_writer_node,
        kvaser_writer_configure_trans_event,
        kvaser_writer_activate_trans_event,
    ])
