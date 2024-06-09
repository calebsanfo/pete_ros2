from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, ExecuteProcess
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    gps_node = Node(
        package='gps_publisher_pkg',
        executable='gps_publisher',
        name='gps_publisher',
        output='screen'
    )

    imu_node = Node(
        package='imu_publisher_pkg',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )

    return LaunchDescription([
        gps_node,
        imu_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gps_node,
                on_exit=[
                    LogInfo(msg="GPS Publisher node crashed, restarting..."),
                    ExecuteProcess(cmd=['ros2', 'run', 'gps_publisher_pkg', 'gps_publisher'], output='screen')
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_node,
                on_exit=[
                    LogInfo(msg="IMU Publisher node crashed, restarting..."),
                    ExecuteProcess(cmd=['ros2', 'run', 'imu_publisher_pkg', 'imu_publisher'], output='screen')
                ]
            )
        ),
    ])