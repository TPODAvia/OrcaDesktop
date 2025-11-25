#!/usr/bin/env python3
# Launch RMF dashboard + Influx exporter + cam_noise_node (6 noisy cams to Influx)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    influx_url     = LaunchConfiguration('influx_url')
    influx_bucket  = LaunchConfiguration('influx_bucket')
    influx_org     = LaunchConfiguration('influx_org')
    influx_token   = LaunchConfiguration('influx_token')
    robot_name     = LaunchConfiguration('robot_name')
    influx_udp_port= LaunchConfiguration('influx_udp_port')  # for reference; node uses INFLUX_URL host:port and UDP 8094

    return LaunchDescription([
        # ---- Launch args (edit defaults here or pass via CLI) ----
        DeclareLaunchArgument('influx_url',      default_value='http://localhost:8086'),
        DeclareLaunchArgument('influx_bucket',   default_value='rmf'),
        DeclareLaunchArgument('influx_org',      default_value='org'),
        DeclareLaunchArgument('influx_token',    default_value='token'),
        DeclareLaunchArgument('robot_name',      default_value='vboxuser_Ubuntu22'),
        DeclareLaunchArgument('influx_udp_port', default_value='8094'),

        # ---- Original nodes (unchanged) ----
        Node(
            package='rmf_manager_cloud',
            executable='cloud_server_node.py',  # if installed entrypoint exists, use: executable='dashboard_node'
            name='rmf_dashboard_multi',
            output='screen',
            respawn=False,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'ui_host': '0.0.0.0',
                'ui_port': 5080,
                'ui_viz_host': '0.0.0.0',
                'ui_viz_port': 8080,
                'viz_update_hz': 1.0,
                # go2rtc integration
                # mount go2rtc.yaml into this container at /config/go2rtc.yaml
                'go2rtc_config_path': os.path.join(get_package_share_directory('rmf_manager_cloud'), 'launch', 'go2rtc.yaml'),
                # leave empty to auto-use http://<dashboard_host>:1984
                'go2rtc_base_url': '',
            }],
        ),

        # Node(
        #     package='rmf_manager_cloud',
        #     executable='rmf_influx_exporter.py',
        #     name='rmf_influx_exporter',
        #     output='screen',
        #     respawn=False,  # set True if you want it to auto-restart
        #     arguments=['--ros-args', '--log-level', 'info'],
        # ),
    ])
