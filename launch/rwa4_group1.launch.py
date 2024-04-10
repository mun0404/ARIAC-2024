from launch import LaunchDescription  # Import LaunchDescription class from launch module
from launch_ros.actions import Node  # Import Node action from launch_ros.actions module
from launch.actions import TimerAction  # Import TimerAction for introducing delay


def generate_launch_description():

    # Create a LaunchDescription object
    ld = LaunchDescription()

    competitor_control_system = Node(
        package="rwa4_group1",
        executable="competition_node.py",
        emulate_tty=True,
    )

    order = Node(
        package="rwa4_group1",
        executable="kitting_node.py",
        emulate_tty=True,
    )

    # Add actions to the launch description
    ld.add_action(order)
    ld.add_action(TimerAction(
        period=2.0,  # delay for 2 seconds
        actions=[
            competitor_control_system  # Add competition_start action after the delay
        ]
    ))

    return ld # Return the launch description object