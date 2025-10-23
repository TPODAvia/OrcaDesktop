#!/usr/bin/env python3
# Minimal launch: start the multi-robot RMF dashboard with inline params.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmf_manager_cloud',
            # If your setup installs an entrypoint named "dashboard_node" (no .py),
            # change the next line to: executable='dashboard_node',
            executable='cloud_server_node.py',
            name='rmf_dashboard_multi',
            output='screen',
            respawn=False,  # set True if you want it to auto-restart
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                # ---- edit these defaults as you like ----
                'ui_host': '0.0.0.0',
                'ui_port': 5080,
                'ui_viz_host': '0.0.0.0',
                'ui_viz_port': 8080,
                'viz_update_hz': 1.0,
            }],
        ),
        Node(
            package='rmf_manager_cloud',
            # If your setup installs an entrypoint named "dashboard_node" (no .py),
            # change the next line to: executable='dashboard_node',
            executable='rmf_influx_exporter.py',
            name='rmf_influx_exporter',
            output='screen',
            respawn=False,  # set True if you want it to auto-restart
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
