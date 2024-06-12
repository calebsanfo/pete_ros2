import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 
                 '/topic1', 
                 '/topic2',
                 '/image'],
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
