from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter, PushRosNamespace

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import yaml


def launch_setup(context, *args, **kwargs):

#   robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
   imu_data_topic = LaunchConfiguration("imu_data_topic").perform(context)
   imu_configuration_file = LaunchConfiguration("imu_configuration_file").perform(context)
   gps_nmea_sentence_topic = LaunchConfiguration("gps_nmea_sentence_topic").perform(context)
   gps_configuration_file = LaunchConfiguration("gps_configuration_file").perform(context)
   vehicle_controller_odom_topic = LaunchConfiguration("vehicle_controller_odom_topic").perform(context)

   imu_plugin = Node(
       package="romea_imu_localisation_plugin",
       executable="imu_localisation_plugin",
       output="screen",
       parameters=[
          {"debug": True},
          {"enable_accelerations": True}
       ],
       remappings=[
          ("imu/data", imu_data_topic),
          ("vehicle_controller/odom",vehicle_controller_odom_topic)
       ],

   )


    return [
        gazebo,
        GroupAction(
            actions=[
#                SetParameter(name="use_sim_time", value=use_sim_time),
                PushRosNamespace("localisation"),
                imu_plugin,
            ]
        ),
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("imu_data_topic"))

     imu_data_topic = LaunchConfiguration("imu_data_topic").perform(context)
     imu_configuration_file = LaunchConfiguration("imu_configuration_file").perform(context)
     gps_nmea_sentence_topic = LaunchConfiguration("gps_nmea_sentence_topic").perform(context)
     gps_configuration_file = LaunchConfiguration("gps_configuration_file").perform(context)
     vehicle_controller_odom_topic = LaunchConfiguration("vehicle_controller_odom_topic").perform(context)

#declared_arguments.append(
#        DeclareLaunchArgument("robot_namespace", default_value="adap2e")
#    )

#    declared_arguments.append(
#        DeclareLaunchArgument("joystick_type", default_value="xbox")
#    )

#    declared_arguments.append(
#        DeclareLaunchArgument("launch_gazebo", default_value="True")
#    )

#    return LaunchDescription(
#        declared_arguments + [OpaqueFunction(function=launch_setup)]
#    )


