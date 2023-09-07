// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_AND_ROBOT_TO_ROBOT_LOCALISATION_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_AND_ROBOT_TO_ROBOT_LOCALISATION_HPP_

// std
#include <memory>

// romea
// #include "romea_common_utils/publishers/data_publisher.hpp"
// #include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
// #include "romea_common_utils/publishers/diagnostic_publisher.hpp"
// #include "romea_common_utils/publishers/transform_publisher.hpp"
// #include "romea_common_utils/publishers/odom_publisher.hpp"

// local
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include "romea_robot_to_world_localisation_core/robot_to_world_localisation.hpp"

namespace romea
{

template<FilterType FilterType_>
class R2WR2RLocalisation : public R2WLocalisation<FilterType_>
{
public:
  using OdometryMsg = nav_msgs::msg::Odometry;

public:
  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  explicit R2WR2RLocalisation(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  virtual ~R2WR2RLocalisation() = default;

private:
  void make_leader_odom_sub_();

  void make_leader_pose_and_twist_pub_();

  void leader_odom_callback_(OdometryMsg::ConstSharedPtr msg);

private:
  std::shared_ptr<rclcpp::Subscription<OdometryMsg>> leader_odom_sub_;
  std::shared_ptr<StampedPublisherBase<PoseAndTwist2D>> leader_pose_and_twist_publisher_;
};

using R2WR2RKalmanLocalisation = R2WR2RLocalisation<FilterType::KALMAN>;
using R2WR2RParticleLocalisation = R2WR2RLocalisation<FilterType::PARTICLE>;

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_AND_ROBOT_TO_ROBOT_LOCALISATION_HPP_
