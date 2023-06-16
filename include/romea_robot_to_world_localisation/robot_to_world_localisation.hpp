// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_HPP_

// std
#include <memory>

// romea
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_common_utils/publishers/transform_publisher.hpp"
#include "romea_common_utils/publishers/odom_publisher.hpp"

// local
#include "romea_robot_to_world_localisation/robot_to_world_localisation_filter.hpp"
#include "romea_robot_to_world_localisation/visibility_control.h"

namespace romea
{

template<FilterType FilterType_>
class R2WLocalisation
{
public:
  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  explicit R2WLocalisation(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  virtual ~R2WLocalisation() = default;

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void make_filter_();

  void make_timer_();

  void make_status_publisher_();

  void make_odom_publisher_();

  void make_tf_publisher_();

  void make_diagnostic_publisher_();

  void publish_diagnostics_(const rclcpp::Time & stamp);

  void timer_callback_();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::unique_ptr<R2WLocalisationFilter<FilterType_>> filter_;

  std::shared_ptr<StampedPublisherBase<Pose3D>> tf_publisher_;
  std::shared_ptr<StampedPublisherBase<PoseAndTwist3D>> odom_publisher_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_publisher_;
  std::shared_ptr<PublisherBase<LocalisationFSMState>> status_publisher_;
};

using R2WKalmanLocalisation = R2WLocalisation<FilterType::KALMAN>;
using R2WParticleLocalisation = R2WLocalisation<FilterType::PARTICLE>;

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_HPP_
