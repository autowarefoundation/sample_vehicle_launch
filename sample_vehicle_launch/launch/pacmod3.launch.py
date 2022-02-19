# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def on_output(event):
    card_info = yaml.safe_load(event.text.decode())
    if "Card 0" in card_info.keys():
        return IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                [FindPackageShare("pacmod3"), "/launch/pacmod3.launch.xml"]
            ),
            launch_arguments={
                "use_kvaser": "true",
                "kvaser_hardware_id": str(card_info["Card 0"]["S/N"]),
            }.items(),
        )


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kvaser_interface",
                executable="list_channels",
                name="list_channels",
            ),
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=on_output,
                )
            ),
        ]
    )
