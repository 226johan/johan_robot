import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_bring_dir=get_package_share_directory('robot_bringup')
    ydlidar_ros2_dir=get_package_share_directory('ydlidar')

    urdf2tf=launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_bring_dir,'/launch','/urdf2tf.launch.py']))
    
    odom2tf=launch_ros.actions.Node(
        package='robot_bringup',
        executable='odom2tf',
        output='screen'
    )

    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4','--port','8888'],
        output='screen'
    )

    ros_serial2wifi=launch_ros.actions.Node(
        package='ros_serail2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        output='screen'
    )

    ydlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_ros2_dir,'/launch','/ydlidar_launch.py']
    ))

    ydlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ydlidar])

    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        microros_agent,
        ros_serial2wifi,
        ydlidar_delay
    ])