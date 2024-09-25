# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# this is an example to launch resize node.
# if you want to run the test node you should build test node,
# and as the follow creat two new launch files to launch the #TestPubNode and #TestSubNode.

def generate_launch_description():
    composable_nodes = [
#ResizeNode
        ComposableNode(
            package='qrb_ros_image_resize',
            plugin='qrb_ros::resize::ResizeNode',
            name='qrb_ros_image_resize',
            parameters=[{
                #'calculate_enable':True,
                'use_scale': False,
                'height': 400,
                'width': 400,
                #'use_scale': True,
                #'height_scale': 0.25,
                #'width_scale': 0.25,
            }]),
'''
#TestPubNode
        ComposableNode(
            package='qrb_ros_image_resize',
            plugin='qrb_ros::resize::TestPubNode',
            name='TestPubNode',
            ),
#TestSubNode
        ComposableNode(
            package='qrb_ros_image_resize',
            plugin='qrb_ros::resize::TestSubNode',
            name='TestSubNode',
            ),
'''
    ]

    container = ComposableNodeContainer(
#ResizeNode
        name='resize',
'''
#TestPubNode
        name='pub',
#TestPubNode
        name='sub',
'''
        namespace='container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    return launch.LaunchDescription([container])
