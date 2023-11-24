// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_

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
namespace ros2
{

template<core::FilterType FilterType_>
class R2WLocalisationFilter
{
public:
  using Filter =
    typename core::R2WLocalisationTraits<FilterType_>::Filter;
  using Predictor =
    typename core::R2WLocalisationTraits<FilterType_>::Predictor;
  using UpdaterTwist =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterTwist;
  using UpdaterLinearSpeed =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterLinearSpeed;
  using UpdaterLinearSpeeds =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterLinearSpeeds;
  using UpdaterAngularSpeed =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterAngularSpeed;
  using UpdaterAttitude =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterAttitude;
  using UpdaterCourse =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterCourse;
  using UpdaterRange =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterRange;
  using UpdaterPose =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterPose;
  using UpdaterPosition =
    typename core::R2WLocalisationTraits<FilterType_>::UpdaterPosition;
  using Results =
    typename core::R2WLocalisationTraits<FilterType_>::Results;

  using UpdaterInterface = LocalisationUpdaterInterfaceBase;
  using UpdaterInterfaceTwist = LocalisationUpdaterInterface<Filter, UpdaterTwist,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
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
  core::LocalisationFSMState get_fsm_state();

  const Results & get_results(const core::Duration & duration);

  core::DiagnosticReport make_diagnostic_report(const core::Duration & duration);

private:
  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void make_results_(std::shared_ptr<rclcpp::Node> node);

  template<typename Interface>
  void add_proprioceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name,
    const unsigned int & default_minimal_rate);

  template<typename interface>
  void add_exteroceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name,
    const unsigned int & default_minimal_rate,
    const std::string & default_trigger_mode);

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Results> results_;
  std::list<std::unique_ptr<UpdaterInterface>> updater_interfaces_;
};


//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2WLocalisationFilter<FilterType_>::R2WLocalisationFilter(std::shared_ptr<rclcpp::Node> node)
: filter_(nullptr),
  results_(nullptr),
  updater_interfaces_()
{
  make_filter_(node);
  add_proprioceptive_updater_interface_<UpdaterInterfaceTwist>(
    node, "twist_updater", "twist", 10);
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeed>(
    node, "linear_speed_updater", "twist", 0);
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeeds>(
    node, "linear_speeds_updater", "twist", 0);
  add_proprioceptive_updater_interface_<UpdaterInterfaceAngularSpeed>(
    node, "angular_speed_updater", "angular_speed", 0);
  add_proprioceptive_updater_interface_<UpdaterInterfaceAttitude>(
    node, "attitude_updater", "attitude", 10);
  add_exteroceptive_updater_interface_<UpdaterInterfacePosition>(
    node, "position_updater", "position", 1, "always");
  add_exteroceptive_updater_interface_<UpdaterInterfaceCourse>(
    node, "course_updater", "course", 1, "once");
  add_exteroceptive_updater_interface_<UpdaterInterfaceRange>(
    node, "range_updater", "range", 0, "always");
  add_exteroceptive_updater_interface_<UpdaterInterfacePose>(
    node, "pose_updater", "pose", 0, "always");
  make_results_(node);
}


//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
core::LocalisationFSMState R2WLocalisationFilter<FilterType_>::get_fsm_state()
{
  return filter_->getFSMState();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::reset()
{
  filter_->reset();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_filter_(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_parameters<FilterType_>(node);
  declare_predictor_parameters(node, 2.0, 10.0, std::numeric_limits<double>::max());
  filter_ = make_filter<Filter, Predictor, FilterType_>(node);
  RCLCPP_INFO_STREAM(node->get_logger(), "filter: started ");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_results_(std::shared_ptr<rclcpp::Node> node)
{
  results_ = make_results<Results, FilterType_>(node);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_proprioceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name,
  const unsigned int & default_minimal_rate)
{
  declare_proprioceptive_updater_parameters(node, updater_name, default_minimal_rate);

  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(
      node,
      updater_name);

    auto plugin = make_updater_interface<Interface>(
      node,
      topic_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}


//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_exteroceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name,
  const unsigned int & default_minimal_rate,
  const std::string & default_trigger_mode)
{
  declare_exteroceptive_updater_parameters(
    node, updater_name, default_minimal_rate, default_trigger_mode);

  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto updater = make_exteroceptive_updater<Updater, FilterType_>(
      node,
      updater_name);
    auto plugin = make_updater_interface<Interface>(
      node,
      topic_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
const typename R2WLocalisationFilter<FilterType_>::Results &
R2WLocalisationFilter<FilterType_>::get_results(const core::Duration & duration)
{
  results_->setDuration(duration);
  filter_->getCurrentState(duration, results_.get());
  return *results_;
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
core::DiagnosticReport R2WLocalisationFilter<FilterType_>::make_diagnostic_report(
  const core::Duration & stamp)
{
  core::DiagnosticReport report;
  for (auto & interface_ptr : updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    report += interface_ptr->get_report();
  }

  return report;
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
