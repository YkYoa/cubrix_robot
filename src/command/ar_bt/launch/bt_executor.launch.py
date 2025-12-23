from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='simple_motion_test.xml',
        description='Name of the behavior tree XML file (in trees/ directory)'
    )
    
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='Arm',
        description='Planning group name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # BT Executor Node
    bt_executor = Node(
        package='ar_bt',
        executable='bt_executor_node',
        name='ar_bt_executor',
        output='screen',
        parameters=[{
            'tree_file': LaunchConfiguration('tree_file'),
            'planning_group': LaunchConfiguration('planning_group'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    return LaunchDescription([
        tree_file_arg,
        planning_group_arg,
        use_sim_time_arg,
        bt_executor,
    ])
