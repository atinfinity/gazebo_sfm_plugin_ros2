#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Ported to Gazebo Harmonic (gz-sim) / ROS 2 Jazzy.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory("gazebo_sfm_plugin_ros2")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # The actor skin/animation must be referenced by an absolute path: gz
    # mis-resolves relative actor mesh paths, which breaks skeleton animation.
    # So substitute the bundled mesh's absolute path into the world template.
    mesh_file = os.path.join(pkg_share, "meshes", "walk.dae")
    template = os.path.join(pkg_share, "worlds", "sfm.sdf.in")
    with open(template, "r") as f:
        world_content = f.read().replace("@MESH_FILE@", mesh_file)

    world = os.path.join(tempfile.gettempdir(), "sfm_generated.sdf")
    with open(world, "w") as f:
        f.write(world_content)

    # Make the compiled plugins discoverable by gz-sim. They are installed to
    # <prefix>/lib/<pkg>; the install prefix is the parent of the share dir.
    install_prefix = os.path.dirname(os.path.dirname(pkg_share))
    plugin_path = os.path.join(install_prefix, "lib",
                               "gazebo_sfm_plugin_ros2")

    set_plugin_path = AppendEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r"]}.items(),
    )

    return LaunchDescription(
        [
            set_plugin_path,
            gz_sim,
        ]
    )
