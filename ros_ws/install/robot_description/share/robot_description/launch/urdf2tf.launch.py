import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_tutorial_path=get_package_share_directory('robot_description')
    robot_model_path=urdf_tutorial_path + '/urdf/robot.urdf'

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(robot_model_path),description='Path to robot model file')
    
    robot_description=launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ',launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node=launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])


