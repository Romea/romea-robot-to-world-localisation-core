# 1 Overview #

This package provides robot to robot localisation node able to estimate the robot position in using data comming from IMU, GPS, radar, lidar and RTLS sensors.

# 2 Node #

### 2.1 Subscribed Topics ###

- localisation/twist (romea_localisation_msgs::msg::ObservationTwist2DStamped)

    This topic is provided by odo localisation plugin node and contains robot twist displacement data

- localisation/angular_speed(romea_localisation_msgs::msg::ObservationAngularSpeedStamped)

    This topic is provided by imu localisation plugin node and contains robot angular speed data

- localisation/position (romea_localisation_msgs::msg::ObservationPosition2DStamped)

    This topic is provided by gps localisation plugin node and contains a cartesian position with respect of an ENU reference frame defined by user

- localisation/course (romea_localisation_msgs::msg::ObservationCourse2DStamped)

    This topic is provided by gps localisation plugin node and contains a robot course angle with of an ENU reference frame defined by user

- localisation/pose (romea_localisation_msgs::msg::ObservationPose2DStamped)

    This topic is provided can be provided lidar, radar or rtls localisation plugin nodes. It contains a cartesian pose with respect of an ENU reference frame defined by user 

- localisation/range (romea_localisation_msgs::msg::ObservationRangeStamped)

    This topic is provided by rtls localisation plugin node and contains range data between follower and leader rtls transceivers

### 2.2 Published Topics ###

- leader_pose_filtered (romea_common_msgs::msg::Pose2DStamped)

  Pose of robot leader in the ENU reference frame defined by user

### 2.3 Parameters ###

  ~filter.state_pool_size (int , default: 1000)

    Size of the pool of filter states

  ~predictor.maximal_dead_recknoning_elapsed_time (double, default: 1.0)

    Maximal elapsed time in dead reckoning mode before to stop localisation filter
  
  ~predictor.maximal_dead_recknoning_travelled_distance (double, default: 1.0)

    Maximal travelled distance in dead reckoning mode before to stop localisation filter
  
  ~predictor.maximal_position_circular_error_probability (double, default)

    Maximal circular error in dead reckoning mode before to stop localisation filter

  ~twist_updater.minimal_rate (int, default: 10)

    Minimal rate for twist_updater input data (provided by odo plugin), if this rate is equal to 0 twist_updater is not started 

  ~linear_speed_updater.minimal_rate (int, default: 0)

    Minimal rate for linear_speed_updater input data (provided by odo plugin), if this rate is equal to 0 linear_speed_updater is not started 

  ~linear_speeds_updater.minimal_rate (int, default: 0)
  
    Minimal rate for linear_speeds_updater input data (provided by odo plugin), if this rate is equal to 0 linear_speeds_updater is not started 

  ~angular_updater.minimal_rate (int, default: 0)
  
    Minimal rate for angular_updater input data (provided by imu plugin), if this rate is equal to 0 angular_updater is not started 

  ~position_updater.mahalanobis_distance_rejection_threshold (double, default: 5.0)

    Mahalanobis distance taking into account by position updater to reject outliers 

  ~position_updater.minimal_rate (int, default: 1)

    Minimal rate for position_updater input data (provided by gps plugin), if this rate is equal to 0 position_updater is not started 

  ~position_updater.trigger (string, default: "always")

    Update trigger mode when position data is received. If "once" mode is selected the position updater will be triggered only one time otherwise the position updater will be triggerred each time data is received.

  ~course_updater.mahalanobis_distance_rejection_threshold (double, default: 5.0)

    Mahalanobis distance taking into account by course updater to reject outliers 

  ~course_updater.minimal_rate (int, default: 1)

    Minimal rate for course_updater input data (provided by gps plugin), if this rate is equal to 0 course_updater is not started 

  ~course_updater.trigger (string, default: "once")

    Update trigger mode when course data is received. If "once" mode is selected the course updater will be triggered only one time otherwise the course updater will be triggerred each time data is received.

  ~pose_updater.mahalanobis_distance_rejection_threshold (double, default: 5.0)

    Mahalanobis distance taking into account by pose updater to reject outliers 

  ~pose_updater.minimal_rate (int, default: 1)

    Minimal rate for pose_updater input data (provided by rtls plugin), if this rate is equal to 0 pose_updater is not started 

  ~pose_updater.trigger (string, default: "once")

    Update trigger mode when pose data is received. If "once" mode is selected the pose updater will be triggered only one time otherwise the pose updater will be triggerred each time data is received.

  ~range_updater.mahalanobis_distance_rejection_threshold (double, default: 5.0)

    Mahalanobis distance taking into account by range updater to reject outliers 

  ~range_updater.minimal_rate (int, default: 10)

    Minimal rate for position_updater input data (provided by rtls plugin); , if this rate is equal to 0 range_updater is not started  

  ~range_updater.trigger (string, default: "always")

    Update trigger mode when position data is received. If "once" mode is selected the range updater will be triggered only one time otherwise the range updater will be triggerred each time data is received.
     
  ~base_footprint_frame_id (string, default: base_footprint):

    Name of robot base footprint

  ~publish_rate (int, default: 10)

    Rate at which localisation results are published

  ~log_directory(string, default: result of rclcpp::get_logging_directory()):

    Directory where localisation logs are stored
  
  ~debug (bool, default: false)

    Enable debug logs
  