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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    multiagent_viz = LaunchConfiguration("multiagent_viz")
    bluerov_viz = LaunchConfiguration("bluerov_viz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_plotjuggler = LaunchConfiguration("use_plotjuggler")
    use_rqt = LaunchConfiguration("use_rqt")

    pkg_share = get_package_share_directory("coug_gui")

    plotjuggler_layout_file = os.path.join(pkg_share, "plotjuggler", "plotjuggler.xml")
    rqt_perspective_file = os.path.join(pkg_share, "rqt", "rqt.perspective")

    mapviz_config_file = PythonExpression(
        [
            "'",
            os.path.join(pkg_share, "mapviz", "bluerov2_mapviz_config.mvc"),
            "' if '",
            bluerov_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "mapviz", "multi_mapviz_config.mvc"),
            "' if '",
            multiagent_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "mapviz", "mapviz_config.mvc"),
            "'",
        ]
    )

    rviz_config_file = PythonExpression(
        [
            "'",
            os.path.join(pkg_share, "rviz", "bluerov2_rviz_config.rviz"),
            "' if '",
            bluerov_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "rviz", "multi_rviz_config.rviz"),
            "' if '",
            multiagent_viz,
            "' == 'true' else '",
            os.path.join(pkg_share, "rviz", "rviz_config.rviz"),
            "'",
        ]
    )

    diagnostics_params_file = PythonExpression(
        [
            "'",
            os.path.join(pkg_share, "rqt", "bluerov2_diagnostics_params.yaml"),
            "' if '",
            bluerov_viz,
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
                "bluerov_viz",
                default_value="false",
                description="Load BlueROV specific viz config if true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 if true",
            ),
            DeclareLaunchArgument(
                "use_mapviz",
                default_value="true",
                description="Launch Mapviz if true",
            ),
            DeclareLaunchArgument(
                "use_plotjuggler",
                default_value="true",
                description="Launch PlotJuggler if true",
            ),
            DeclareLaunchArgument(
                "use_rqt",
                default_value="true",
                description="Launch RQT if true",
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="coug_gui",
                executable="odom_to_path_node.py",
                name="odom_to_path_node",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(use_mapviz),
                package="mapviz",
                executable="mapviz",
                name="mapviz",
                parameters=[
                    {"config": mapviz_config_file},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                condition=IfCondition(use_mapviz),
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                remappings=[
                    ("fix", "/origin"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(use_mapviz),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_origin_transform",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "origin",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(use_plotjuggler),
                package="plotjuggler",
                executable="plotjuggler",
                name="plotjuggler",
                arguments=["--layout", plotjuggler_layout_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(use_rqt),
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                parameters=[
                    diagnostics_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                condition=IfCondition(use_rqt),
                package="rqt_gui",
                executable="rqt_gui",
                name="rqt_gui",
                arguments=["--perspective-file", rqt_perspective_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
