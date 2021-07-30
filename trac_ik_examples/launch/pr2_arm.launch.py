# Copyright 2021 Abrar Rahman Protyasha
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_samples = LaunchConfiguration('num_samples', default_value=1000)
    chain_start = LaunchConfiguration('chain_start', default_value='torso_lift_link')
    chain_end = LaunchConfiguration('chain_end', default_value='r_wrist_roll_link')
    timeout = LaunchConfiguration('timeout', default_value=0.005)

    pkg_share = FindPackageShare('trac_ik_examples').find('trac_ik_examples')
    urdf_file = os.path.join(pkg_share, 'launch', 'pr2_arm', '.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    ik_test_params = {
        'robot_description': robot_desc,
    }

    return LaunchDescription([
        DeclareLaunchArgument('num_samples', default_value=1000),
        DeclareLaunchArgument('chain_start', default_value='torso_lift_link'),
        DeclareLaunchArgument('chain_end', default_value='r_wrist_roll_link'),
        DeclareLaunchArgument('timeout', 0.005),
        Node(package='trac_ik_examples',
             executable='ik_tests',
             output='screen',
             parameters=[rsp_params.update(
                 {'num_samples': num_samples,
                  'chain_start': chain_start,
                  'chain_end': chain_end,
                  'timeout': timeout,}
             )]),
    ])
