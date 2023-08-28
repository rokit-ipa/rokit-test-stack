from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'test_name',
        default_value='MAXIMUM_VELOCITY',
        description='max velocity '
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'trial_number',
        default_value='1',
        description='trial number ')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'robot_name',
        default_value='MiR',
        description='robot name')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'temperature',
        default_value='20',
        description='temperature')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'tracking_object',
        default_value='rokit_1',
        description='tracking object')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'inclination',
        default_value='0',
        description='inclination')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'humidity',
        default_value='10',
        description='humidity')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'floor_type',
        default_value='wood',
        description='floor type')
    )
    declared_arguments.append(DeclareLaunchArgument(
        'notes',
        default_value='Note',
        description='notes')
    )
    
    print(declared_arguments)
    test_name = LaunchConfiguration('test_name')
    trial_number = LaunchConfiguration('trial_number')
    humidity = LaunchConfiguration('humidity')
    temperature = LaunchConfiguration('temperature')
    robot_name = LaunchConfiguration('robot_name')
    inclination = LaunchConfiguration('inclination')
    floor_type = LaunchConfiguration('floor_type')
    notes = LaunchConfiguration('notes')
    tracking_object = LaunchConfiguration('tracking_object')

    main_calculator_node = Node(
        package="rokit_test_stack",
        executable="rokit_calculator",
        name="rokit_calculator_node",
        output="screen",
                parameters=[{'test_name': test_name, 'trial_number': trial_number, 'humidity': humidity, 'temperature': temperature, 'robot_name': robot_name, 'inclination': inclination, 'floor_type': floor_type, 'notes': notes, 'tracking_object':tracking_object}],
    )
    
    
    hostname = '192.168.10.1'
    buffer_size = 200
    topic_namespace = 'vicon'
    
    vicon_receiver_node = Node(
            package='vicon_receiver', node_executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )
   

    return LaunchDescription(declared_arguments+[main_calculator_node, vicon_receiver_node])
