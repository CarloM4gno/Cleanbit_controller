import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    package_name = "cleanbit_control"


    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro'
    )

    sllidar_launch_file = os.path.join(
	get_package_share_directory('sllidar_ros2'),
	'launch',
	'sllidar_c1_launch.py'
    )
	
    joy_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'joystick.yaml'
    )
    

    slam_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    )

    twist_mux_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    # nav2_params_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'nav2_params.yaml'
    # )


    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()



    # Create a node to publish the robot's state based on its URDF description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False}
        ],
        output='screen'
    )


    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'spazz.ino/body_link/laser_frame'],
        output='screen'
    )

# 3. Lidar Node
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        launch_arguments={
            'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_1e295655477fef118ebc241cedd322a4-if00-port0',    
            'frame_id': 'laser_frame'
        }.items()
    )

    arduino_bridge_node = Node(
            package=package_name,
            executable="arduino_bridge.py",
            name="arduino_bridge",
            output="screen"
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params_path],
        )
    
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params_path],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        )


    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'false'
        }.items()
    )


    twist_mux_process = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params_path],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'params_file': nav2_params_path,
    #         'use_sim_time': 'false'
    #     }.items()
    # )





    return LaunchDescription([
        robot_state_publisher_node,
        lidar_launch,
        arduino_bridge_node,
        static_tf_pub_node,
        joy_node,
        teleop_node,
        slam_toolbox_launch,
        twist_mux_process
        # nav2_launch
    ])
