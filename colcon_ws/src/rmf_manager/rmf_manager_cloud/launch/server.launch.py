#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='rmf_manager_cloud',
        executable='cloud_server_node.py',
        name='rmf_manager_cloud',
        output='screen',
        emulate_tty=True,
        # The node consumes this CLI arg and exports ROS_DOMAIN_ID internally
        arguments=['--ros-domain-id', '41'],
        parameters=[{
            # --- UI (three UIs are served by the node) ---
            'ui_host': '0.0.0.0',
            'ui_port': 5005,          # Task Manager UI
            'ui_viz_host': '0.0.0.0',
            'ui_viz_port': 8080,      # Map Visualizer UI

            # --- Map/graph files (leave empty if not used) ---
            'param_yaml_path': '',
            'building_yaml_path': '',

            # --- Multi-robot via Zenoh (comma-separated namespaces) ---
            # 'robot_namespaces': 'vboxuser_Ubuntu22,robot02,robot03,robot04,robot05',
            'robot_namespaces': 'vboxuser_Ubuntu22',

            # --- Zenoh connectivity (match your router/peer) ---
            'zenoh_mode': 'peer',                         # peer|client|router
            'zenoh_connect': 'tcp/192.168.196.48:7447',
            'zenoh_listen': 'tcp/0.0.0.0:7448',
        }],
    )

    return LaunchDescription([node])
