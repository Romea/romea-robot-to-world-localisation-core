<?xml version="1.0"?>
<launch>

  <arg name="robot_name"/>
  <arg name="wgs84_anchor"/>
  <arg name="config_filename" default="$(find wgs84_path_following)/config/localisation/gps_rtls.yaml"/>

  <arg name="kinematic_topic" default="/$(arg robot_name)/vehicle_controller/kinematic"/>
  <arg name="odom_topic" default="/$(arg robot_name)/vehicle_controller/odom"/>
  <arg name="gps_topic" default="/$(arg robot_name)/gps/nmea_sentence"/>
  <arg name="imu_topic" default="/$(arg robot_name)/imu/data" />
  <arg name="rtls_poll_topic" default="/$(arg robot_name)/rtls/poll" />
  <arg name="rtls_range_topic" default="/$(arg robot_name)/rtls/range" />

  <node pkg="romea_localisation_rtls_plugin"
        type="robot_to_world_rtls_localisation_plugin_node"
        name="robot_to_world_rtls_localisation_plugin"
        namespace="localisation"
        output="screen">
        <remap from="~robot/rtls" to="/$(arg robot_name)/rtls"/>
        <remap from="~infrastructure/rtls" to="/infrastructure/rtls"/>
        <remap from="rtls/poll" to="/$(arg rtls_poll_topic)"/>
        <remap from="rtls/range" to="/$(arg rtls_range_topic)"/>
        <param name="base_link_frame" value="/$(arg robot_name)/base_link" type="str"/>
        <param name="autostart" value="true" type="bool"/>
        <param name="poll_rate" value="20." type="double"/>
        <param name="debug" value="true" type="bool"/>
  </node>

  <include file="$(find romea_robot_to_world_localisation)/launch/wgs84_gps_localisation.launch">
     <arg name="robot_name" value="$(arg robot_name)"/>
     <arg name="wgs84_anchor" value="$(arg wgs84_anchor)"/>
     <arg name="config_filename" default="$(arg config_filename)"/>
     <arg name="kinematic_topic" default="$(arg kinematic_topic)"/>
     <arg name="odom_topic" default="$(arg odom_topic)"/>
     <arg name="gps_topic" default="$(arg gps_topic)"/>
     <arg name="imu_topic" default="$(arg imu_topic)" />

  </include>


</launch>
