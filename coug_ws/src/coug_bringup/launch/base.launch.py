# Copyright (c) 2026 BYU FRoSt Lab
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    multiagent_viz = LaunchConfiguration("multiagent_viz", default="false")
    bluerov_viz = LaunchConfiguration("bluerov_viz", default="false")

    coug_gui_dir = get_package_share_directory("coug_gui")
    coug_gui_launch_dir = os.path.join(coug_gui_dir, "launch")

    coug_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_gui_launch_dir, "mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "multiagent_viz": multiagent_viz,
            "bluerov_viz": bluerov_viz,
        }.items(),
    )

    coug_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_gui_launch_dir, "rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "multiagent_viz": multiagent_viz,
            "bluerov_viz": bluerov_viz,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (HoloOcean) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "multiagent_viz",
            default_value="false",
            description="Use multi-agent visualization config if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "bluerov_viz",
            default_value="false",
            description="Load BlueROV specific viz config if true",
        )
    )
    ld.add_action(coug_mapviz_cmd)
    ld.add_action(coug_rviz_cmd)

    return ld
