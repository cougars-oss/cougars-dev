# Copyright (c) 2026 BYU FROST Lab
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    multiagent_viz = LaunchConfiguration("multiagent_viz")
    bluerov2_viz = LaunchConfiguration("bluerov2_viz")

    pkg_share = get_package_share_directory("coug_rqt")

    rqt_perspective_file = os.path.join(pkg_share, "rqt", "rqt.perspective")
    diagnostics_params_file = PythonExpression(
        [
            "'",
            os.path.join(pkg_share, "rqt", "bluerov2_diagnostics_params.yaml"),
            "' if '",
            bluerov2_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "rqt", "multi_diagnostics_params.yaml"),
            "' if '",
            multiagent_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "rqt", "diagnostics_params.yaml"),
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "multiagent_viz",
                default_value="false",
                description="Use multi-agent visualization config if true",
            ),
            DeclareLaunchArgument(
                "bluerov2_viz",
                default_value="false",
                description="Load BlueROV2 specific viz config if true",
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                parameters=[
                    diagnostics_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                name="rqt_gui",
                arguments=["--perspective-file", rqt_perspective_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
