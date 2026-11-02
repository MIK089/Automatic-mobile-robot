import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file]) #, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}  #,'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    lidar_node = Node(
        package='cspc_lidar',
        executable='cspc_lidar',
        name='cspc_lidar',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[{
            'port': '/dev/serial/by-path/pci-0000:03:00.0-usb-0:2:1.0-port0',
            'frame_id': 'laser_frame',
            'ignore_array': "",
            'baudrate': 230400,
            'samp_rate': 9,
            'resolution_fixed': True,
            'singleChannel': False,
            'auto_reconnect': True,
            'reversion': True,
            'isToFLidar': False,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'max_range': 16.0,
            'min_range': 0.01,
            'frequency': 10.0,
            'version': 4}],
        
    )

    encoder_state_node = Node(
        package='encoder_state',
        executable='encoder_state',
        name='encoder_state',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : True,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0}],
    )

    # data_logger_node = Node(
    #     package='data_logger',
    #     executable='data_logger',
    #     name='data_logger',
    #     output='screen',
    # )


    # Launch!
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use sim time if true'),
        # DeclareLaunchArgument(
        #     'use_ros2_control',
        #     default_value='true',
        #     description='Use ros2_control if true'),

        node_robot_state_publisher,
        lidar_node,
        encoder_state_node,
        rf2o_laser_odometry_node,
    ])
