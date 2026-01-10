from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Launch the Parameter Editor UI only.
    RViz will be launched when user clicks Run Robot/Simulation button.
    """
    return LaunchDescription(
        [
            # Launch Parameter Editor UI only
            ExecuteProcess(
                cmd=["ros2", "run", "ar_ui", "param_editor"],
                output="screen",
            ),
        ]
    )
