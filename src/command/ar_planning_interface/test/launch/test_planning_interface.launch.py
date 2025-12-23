from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    # Load test configs
    test_config_dir = PathJoinSubstitution([
        FindPackageShare('ar_planning_interface'),
        'test', 'config'
    ])
    
    moveitcpp_config = PathJoinSubstitution([test_config_dir, 'moveitcpp.yaml'])
    controller_config = PathJoinSubstitution([test_config_dir, 'moveit_controller.yaml'])
    
    # Planning interface test node
    test_node = Node(
        package='ar_planning_interface',
        executable='ar_planning_interface_test_node',
        name='ar_planning_interface_test',
        output='screen',
        parameters=[
            moveitcpp_config,
            controller_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'planning_group': 'Arm',  # Match your SRDF
            },
        ],
        emulate_tty=True,  # For interactive input
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        test_node,
    ])
