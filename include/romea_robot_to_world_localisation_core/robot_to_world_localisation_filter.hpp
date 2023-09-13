// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_

// std
#include <list>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_core_localisation/robot_to_world/R2WlocalisationTraits.hpp"
#include "romea_localisation_utils/filter/localisation_factory.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface.hpp"

namespace romea
{

template<FilterType FilterType_>
class R2WLocalisationFilter
{
public:
  using Filter = typename  R2WLocalisationTraits<FilterType_>::Filter;
  using Predictor = typename R2WLocalisationTraits<FilterType_>::Predictor;
  using UpdaterLinearSpeed = typename R2WLocalisationTraits<FilterType_>::UpdaterLinearSpeed;
  using UpdaterLinearSpeeds = typename R2WLocalisationTraits<FilterType_>::UpdaterLinearSpeeds;
  using UpdaterAngularSpeed = typename  R2WLocalisationTraits<FilterType_>::UpdaterAngularSpeed;
  using UpdaterAttitude = typename  R2WLocalisationTraits<FilterType_>::UpdaterAttitude;
  using UpdaterCourse = typename  R2WLocalisationTraits<FilterType_>::UpdaterCourse;
  using UpdaterRange = typename  R2WLocalisationTraits<FilterType_>::UpdaterRange;
  using UpdaterPose = typename R2WLocalisationTraits<FilterType_>::UpdaterPose;
  using UpdaterPosition = typename R2WLocalisationTraits<FilterType_>::UpdaterPosition;
  using Results = typename R2WLocalisationTraits<FilterType_>::Results;

  using UpdaterInterface = LocalisationUpdaterInterfaceBase;
  using UpdaterInterfaceLinearSpeed = LocalisationUpdaterInterface<Filter, UpdaterLinearSpeed,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeeds = LocalisationUpdaterInterface<Filter, UpdaterLinearSpeeds,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceAngularSpeed = LocalisationUpdaterInterface<Filter, UpdaterAngularSpeed,
      romea_localisation_msgs::msg::ObservationAngularSpeedStamped>;
  using UpdaterInterfaceAttitude = LocalisationUpdaterInterface<Filter, UpdaterAttitude,
      romea_localisation_msgs::msg::ObservationAttitudeStamped>;
  using UpdaterInterfaceCourse = LocalisationUpdaterInterface<Filter, UpdaterCourse,
      romea_localisation_msgs::msg::ObservationCourseStamped>;
  using UpdaterInterfacePosition = LocalisationUpdaterInterface<Filter, UpdaterPosition,
      romea_localisation_msgs::msg::ObservationPosition2DStamped>;
  using UpdaterInterfaceRange = LocalisationUpdaterInterface<Filter, UpdaterRange,
      romea_localisation_msgs::msg::ObservationRangeStamped>;
  using UpdaterInterfacePose = LocalisationUpdaterInterface<Filter, UpdaterPose,
      romea_localisation_msgs::msg::ObservationPose2DStamped>;

public:
  explicit R2WLocalisationFilter(std::shared_ptr<rclcpp::Node> node);

  void reset();

public:
  LocalisationFSMState get_fsm_state();

  const Results & get_results(const Duration & duration);

  DiagnosticReport make_diagnostic_report(const Duration & duration);

private:
  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void make_results_(std::shared_ptr<rclcpp::Node> node);

  template<typename Interface>
  void add_proprioceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

  template<typename interface>
  void add_exteroceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Results> results_;
  std::list<std::unique_ptr<UpdaterInterface>> updater_interfaces_;
};


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
R2WLocalisationFilter<FilterType_>::R2WLocalisationFilter(std::shared_ptr<rclcpp::Node> node)
: filter_(nullptr),
  results_(nullptr),
  updater_interfaces_()
{
  make_filter_(node);
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeed>(node, "linear_speed_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeeds>(
    node,
    "linear_speeds_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceAngularSpeed>(
    node,
    "angular_speed_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceAttitude>(node, "attitude_updater");
  add_exteroceptive_updater_interface_<UpdaterInterfacePosition>(node, "position_updater");
  add_exteroceptive_updater_interface_<UpdaterInterfaceCourse>(node, "course_updater");
  add_exteroceptive_updater_interface_<UpdaterInterfaceRange>(node, "range_updater");
  add_exteroceptive_updater_interface_<UpdaterInterfacePose>(node, "pose_updater");
  make_results_(node);
}


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
LocalisationFSMState R2WLocalisationFilter<FilterType_>::get_fsm_state()
{
  return filter_->getFSMState();
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::reset()
{
  filter_->reset();
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_filter_(std::shared_ptr<rclcpp::Node> node)
{
  declare_predictor_parameters(node);
  declare_filter_parameters<FilterType_>(node);
  filter_ = make_filter<Filter, Predictor, FilterType_>(node);
  RCLCPP_INFO_STREAM(node->get_logger(), "filter: started ");
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_results_(std::shared_ptr<rclcpp::Node> node)
{
  results_ = make_results<Results, FilterType_>(node);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_proprioceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_proprioceptive_updater_parameters(node, updater_name);

  if (!get_updater_topic_name(node, updater_name).empty()) {
    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(
      node,
      updater_name);

    auto plugin = make_updater_interface<Interface>(
      node,
      updater_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_exteroceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_exteroceptive_updater_parameters(node, updater_name);

  if (!get_updater_topic_name(node, updater_name).empty()) {
    using Updater = typename Interface::Updater;
    auto updater = make_exteroceptive_updater<Updater, FilterType_>(
      node,
      updater_name);
    auto plugin = make_updater_interface<Interface>(
      node,
      updater_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
const typename R2WLocalisationFilter<FilterType_>::Results &
R2WLocalisationFilter<FilterType_>::get_results(const Duration & duration)
{
  results_->setDuration(duration);
  filter_->getCurrentState(duration, results_.get());
  return *results_;
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
DiagnosticReport R2WLocalisationFilter<FilterType_>::make_diagnostic_report(const Duration & stamp)
{
  DiagnosticReport report;
  for (auto & interface_ptr : updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    report += interface_ptr->get_report();
  }

  return report;
}

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
