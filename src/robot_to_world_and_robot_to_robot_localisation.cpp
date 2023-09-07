// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// local
#include "romea_robot_to_world_localisation_core/robot_to_world_and_robot_to_robot_localisation.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
R2WR2RLocalisation<FilterType_>::R2WR2RLocalisation(const rclcpp::NodeOptions & options)
: R2WLocalisation<FilterType_>(options)
{
  make_leader_pose_and_twist_pub_();
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::make_leader_pose_and_twist_pub_()
{
  leader_pose_and_twist_publisher_ = make_stamped_data_publisher<PoseAndTwist2D,
      romea_common_msgs::msg::PoseAndTwist2DStamped>(
    this->node_,
    "filtered_leader_pose",
    get_base_footprint_frame_id(this->node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::make_leader_odom_sub_()
{
  auto callback =
    std::bind(&R2WR2RLocalisation::leader_odom_callback_, this, std::placeholders::_1);

  leader_odom_sub_ = this->node_->template create_subscription<OdometryMsg>(
    "leader_filtered_odom", best_effort(10), callback);
}


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::leader_odom_callback_(OdometryMsg::ConstSharedPtr msg)
{
  // publish odom
  auto stamp = this->node_->get_clock()->now();

  LocalisationFSMState fsm_state = this->filter_->get_fsm_state();
  if (fsm_state == LocalisationFSMState::RUNNING) {
    const auto & results = this->filter_->get_results(to_romea_duration(stamp));

    Pose2D follower_pose = results.toPose2D();
    Pose2D leader_pose = toPose2D(to_romea(msg->pose));

    PoseAndTwist2D follower_to_leader_pose;
    follower_to_leader_pose.pose.yaw =
      betweenMinusPiAndPi(leader_pose.yaw - follower_pose.yaw);

    follower_to_leader_pose.pose.position =
      eulerAngleToRotation2D(-follower_pose.yaw) *
      (leader_pose.position - follower_pose.position);

    // TOTO(jean) add covariance

    follower_to_leader_pose.twist = toTwist2D(to_romea(msg->twist));

    leader_pose_and_twist_publisher_->publish(msg->header.stamp, follower_to_leader_pose);
  }
}

template class R2WR2RLocalisation<FilterType::KALMAN>;
template class R2WR2RLocalisation<FilterType::PARTICLE>;

}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2WR2RKalmanLocalisation)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2WR2RParticleLocalisation)
