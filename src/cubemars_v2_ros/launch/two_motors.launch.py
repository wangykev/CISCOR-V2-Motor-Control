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
        namespace=name,  # topics become /<name>/mit_cmd, /<name>/state_line, ...
        output='screen',
        parameters=[{
            'can_interface': iface,
            'can_id': can_id,
            'motor_type': motor_type,   # 'AK70-10' or 'AK80-64'
            'control_hz': control_hz,   # e.g., 200.0
            'joint_name': name,
            'auto_start': auto_start,   # false by default in our node
        }],
    )

def generate_launch_description():
    iface       = LaunchConfiguration('can_interface')
    control_hz  = LaunchConfiguration('control_hz')
    auto_start  = LaunchConfiguration('auto_start')

    m1_id   = LaunchConfiguration('motor1_id')
    m1_type = LaunchConfiguration('motor1_type')
    m1_name = LaunchConfiguration('motor1_name')

    m2_id   = LaunchConfiguration('motor2_id')
    m2_type = LaunchConfiguration('motor2_type')
    m2_name = LaunchConfiguration('motor2_name')

    return LaunchDescription([
        # Shared args
        arg('can_interface', 'can0', 'SocketCAN interface'),
        arg('control_hz', '200.0', 'Control loop rate (Hz)'),
        arg('auto_start', 'false', 'Send START once on launch'),

        # Motor 1
        arg('motor1_id',   '3',        'Motor 1 CAN ID'),
        arg('motor1_type', 'AK70-10',  'Motor 1 type'),
        arg('motor1_name', 'ak70',     'Motor 1 name/namespace'),

        # Motor 2
        arg('motor2_id',   '4',        'Motor 2 CAN ID'),
        arg('motor2_type', 'AK80-64',  'Motor 2 type'),
        arg('motor2_name', 'ak80',     'Motor 2 name/namespace'),

        motor(m1_name, m1_id, m1_type, iface, control_hz, auto_start),
        motor(m2_name, m2_id, m2_type, iface, control_hz, auto_start),
    ])
