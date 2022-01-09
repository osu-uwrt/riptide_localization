import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # Read in the vehicle's namespace through the command line or use the default value one is not provided
    launch.actions.DeclareLaunchArgument(
        "namespace", 
        default_value="puddles",
        description="Namespace of the vehicle",
    )

    # declare the launch args to read for this file
    config = os.path.join(
        get_package_share_directory('riptide_localization2'),
        'params',
        'ekf_config.yaml'
    )
    # <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization_node">
    #     <rosparam command="load" file="$(find riptide_localization)/params/ekf_config.yaml" />
    #     <param name="base_link_frame" value="$(arg namespace)/base_link" />
    #     <param name="reset_on_time_jump" value="true" />
    # </node>
    return launch.LaunchDescription([
        
        # start robot_localization Extended Kalman filter (EKF)
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            # type='ekf_localization_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[config,
            {
                'base_link_frame':  'puddles' + '/base_link/',
                # 'base_link_frame':  'puddles' + '/base_link/',
                'reset_on_time_jump': True,
            }
            ]),
        launch_ros.actions.Node(
            package='riptide_localization2',
            executable='dvl_converter',
            name='dvl_converter',
        ),
        launch_ros.actions.Node(
            package='riptide_localization2',
            executable='depth_converter',
            name='depth_converter',
        ),

        # A joint state publisher
        launch_ros.actions.Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen"
        ),

        # Publish robot model for Sensor locations
        launch_ros.actions.Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[{'use_tf_static': True}],
        ),

        # Publish world and odom as same thing until we get SLAM
        # This is here so we can compare ground truth from sim to odom
        launch_ros.actions.Node(
            name="odom_to_world_broadcaster",
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "1", "world", "odom", "100"]
        )

    ])