from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    main_calculator_node = Node(
        package="rokit_test_stack",
        executable="rokit_calculator",
        name="rokit_calculator_node",
        output="screen",
        parameters=[
            {"test_name": "MAX_VELOCITY"},
            {"robot_name": "MiR"},
            {"tracking_object": "tracker_1"},
            {"trial_number": "1"},
            {"temperature": "23.0"},
            {"humidity": "9.0"},
            {"notes": "none"},
            {"inclination": "0.0"},
            {"floor_type": "wood"}
        ],
    )
    hostname = '192.168.10.1'
    buffer_size = 200
    topic_namespace = 'vicon'
    
    vicon_receiver_node = Node(
            package='vicon_receiver', node_executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )
   

    return LaunchDescription([main_calculator_node, vicon_receiver_node])
