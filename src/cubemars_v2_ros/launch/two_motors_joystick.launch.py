from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=str(default), description=desc)

def motor(name, can_id, motor_type, iface, control_hz, auto_start):
    return Node(
        package='cubemars_v2_ros',
        executable='motor_node',
        name=name,
        namespace=name,
        output='screen',
        parameters=[{
            'can_interface': iface,
            'can_id': can_id,
            'motor_type': motor_type,
            'control_hz': control_hz,
            'joint_name': name,
            'auto_start': False,     # controller will send start
        }],
    )

def generate_launch_description():
    iface = LaunchConfiguration('can_interface')
    hz    = LaunchConfiguration('control_hz')

    m1_id   = LaunchConfiguration('motor1_id')
    m1_type = LaunchConfiguration('motor1_type')
    m1_name = LaunchConfiguration('motor1_name')

    m2_id   = LaunchConfiguration('motor2_id')
    m2_type = LaunchConfiguration('motor2_type')
    m2_name = LaunchConfiguration('motor2_name')

    return LaunchDescription([
        arg('can_interface', 'can0', 'SocketCAN interface'),
        arg('control_hz', '200.0', 'Motor control rate'),
        arg('motor1_id', '3', 'Motor 1 CAN ID'),
        arg('motor1_type', 'AK70-10', 'Motor 1 type'),
        arg('motor1_name', 'ak70', 'Motor 1 name/namespace'),
        arg('motor2_id', '4', 'Motor 2 CAN ID'),
        arg('motor2_type', 'AK80-64', 'Motor 2 type'),
        arg('motor2_name', 'ak80', 'Motor 2 name/namespace'),

        # Motors
        motor(m1_name, m1_id, m1_type, iface, hz, False),
        motor(m2_name, m2_id, m2_type, iface, hz, False),

        # Joystick driver
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),

        # Controller (publishes to /<ns>/mit_cmd and /<ns>/special)
        Node(
            package='cubemars_v2_ros',
            executable='dual_joystick_position',
            name='dual_joystick_position',
            output='screen',
            parameters=[{
                'motor1_ns': m1_name,
                'motor2_ns': m2_name,
                'auto_start': True,

                # Controller gains
                'kp1': 30.0,
                'kd1': 0.7,
                'kp2': 500.0,
                'kd2': 1.5,
                'pos_lpf_hz': 15.0,

                # Full stick circle = full motor revolution
                'gain_per_rev1': 6.283,
                'gain_per_rev2': 6.283,

                'deadzone_enter': 0.0,
                'deadzone_exit': 0.0,

            }],
        ),
    ])
