from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Common gains (tweak as needed)
    KP = '400.0'
    KD = '2.5'

    ak70 = Node(
        package='YOUR_PKG_NAME',  # <-- change
        executable='joystick_turntable',  # if installed via entry point; else use Python path
        name='ak70_joystick',
        output='screen',
        parameters=[
            {'motor_ns': 'ak70'},
            {'kp': KP}, {'kd': KD},
            {'control_hz': 200.0},
            # Left stick
            {'axis_x': 0}, {'axis_y': 1},
            {'invert_x': 1.0}, {'invert_y': -1.0},
            # Same direction as stick
            {'direction': 1.0},
            # Smoothing + hysteresis
            {'pos_lpf_hz': 10.0},
            {'deadzone_enter': 0.15}, {'deadzone_exit': 0.10},
            # Unlimited turns
            {'enable_pos_clamp': False},
        ]
    )

    ak80 = Node(
        package='YOUR_PKG_NAME',  # <-- change
        executable='joystick_turntable',
        name='ak80_joystick',
        output='screen',
        parameters=[
            {'motor_ns': 'ak80'},
            {'kp': KP}, {'kd': KD},
            {'control_hz': 200.0},
            # Right stick (common Xbox mapping)
            {'axis_x': 3}, {'axis_y': 4},
            {'invert_x': 1.0}, {'invert_y': -1.0},
            # Flip if direction feels reversed on AK80
            {'direction': 1.0},
            # Smoothing + hysteresis
            {'pos_lpf_hz': 10.0},
            {'deadzone_enter': 0.15}, {'deadzone_exit': 0.10},
            {'enable_pos_clamp': False},
        ]
    )

    return LaunchDescription([ak70, ak80])
