// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <fstream>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// romea
#include "romea_robot_to_world_localisation/visibility_control.h"


namespace romea
{

class PathRecorder
{
public:
  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  explicit PathRecorder(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  virtual ~PathRecorder() = default;

private:
  void declare_parameters_();

  void initialize_();

  void odom_callback_(nav_msgs::msg::Odometry::ConstSharedPtr msg);

private:
  std::ofstream path_file_;
  double minimal_distance_between_two_points_;
  double minimal_vehicle_speed_to_insert_point_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
};

}  // namespace romea

// int main(int argc, char ** argv)
// {
//   ros::init(argc, argv, "path_recorder");

//   PathRecorder recorder;
//   try {
//     ros::NodeHandle nh;
//     ros::NodeHandle private_nh("~");
//     recorder.init(nh, private_nh);
//     ros::spin();
//   } catch (std::exception & e) {
//     ROS_ERROR_STREAM("PATH_RECORDER : " << e.what());
//   }
// }
