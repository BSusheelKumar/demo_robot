import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    urdf_xacro_path = '/home/susheel/ros2_tk/src/task_bot/description/robot.urdf.xacro'

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/susheel/ros2_tk/src/task_bot'
    )

    doc = xacro.process_file(urdf_xacro_path)
    robot_desc = doc.toprettyxml(indent='  ')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[
            ('gz_args', ['/home/susheel/ros2_tk/src/task_bot/worlds/customworld.sdf', ' -v 4', ' -r'])  
        ]
    )
    params = {'robot_description': robot_desc}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,'use_sim_time':True}]
    )
 
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0', '-y', '0.0', '-z', '0.1',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-name', 'task_bot', '-allow_renaming', 'false'],
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )



    return LaunchDescription([

        
        gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
