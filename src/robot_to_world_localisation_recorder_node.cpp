//ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

//romea
#include <romea_core_common/geodesy/ECEFConverter.hpp>
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_common_utils/conversions/transform_conversions.hpp>
#include <tf2_ros/transform_listener.h>

//std
#include <fstream>
#include <cmath>

const double DEFAULT_MINIMAL_DISTANCE_BETWEEN_TWO_POINTS = 0.2;
const double DEFAULT_MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT = 0.1;

inline Eigen::Vector2d position2d(const geometry_msgs::Point & p)
{
  return Eigen::Vector2d(p.x,p.y);
}

inline double speed2d(const geometry_msgs::Vector3 & v)
{
  return std::sqrt(v.x*v.x + v.y*v.y);
}

class PathRecorder
{

public:

  PathRecorder():
    path_file_(),
    minimalDistanceBetweenTwoPoints_(),
    minimalVehicleSpeedToInsertPoint_(),
    odom_sub_()
  {
  }

  void init(ros::NodeHandle & nh,
            ros::NodeHandle & private_nh)
  {
    std::string filename = romea::load_param<std::string>(private_nh,"output");

    path_file_.open(filename);
    if(!path_file_.is_open())
    {
      throw std::runtime_error("Unable to open file : " + filename);
    }

    std::string coordinate_system = romea::load_param<std::string>(private_nh,"coordinate_system");

    if(coordinate_system.compare("WGS84")==0)
    {

      auto world_to_map_ = romea::lookupTransformOnce("world",
                                                      "map",
                                                      ros::Time::now(),
                                                      ros::Duration(10.0));

      auto wgs84_origin= romea::ECEFConverter().toWGS84(world_to_map_.translation());
      path_file_ <<"WGS84"<< std::endl;
      path_file_ <<std::setprecision(10)<< wgs84_origin.latitude*180/M_PI<<" ";
      path_file_ <<std::setprecision(10)<< wgs84_origin.longitude*180/M_PI<<" ";
      path_file_ <<std::setprecision(10)<< wgs84_origin.altitude<<std::endl;
    }


    minimalDistanceBetweenTwoPoints_=private_nh.param("minimal_distance_between_two_points",
                                                      DEFAULT_MINIMAL_DISTANCE_BETWEEN_TWO_POINTS);


    minimalDistanceBetweenTwoPoints_=private_nh.param("minimal_vehicle_speed_to_insert_point_",
                                                      DEFAULT_MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT);

    odom_sub_ = nh.subscribe("odom",1,&PathRecorder::localisationCallback,this);
  }

  void localisationCallback(const nav_msgs::OdometryConstPtr & msg)
  {

    Eigen::Vector2d position = position2d(msg->pose.pose.position);
    static Eigen::Vector2d previous_position = position;

    double distance = (position-previous_position).norm();
    double vehicle_speed = speed2d(msg->twist.twist.linear);

    if(distance > minimalDistanceBetweenTwoPoints_ &&
       vehicle_speed > minimalVehicleSpeedToInsertPoint_)
    {
      path_file_ << position.x() << " "<<position.y()<<std::endl;
      previous_position=position;
    }
  }

private :

  std::ofstream path_file_;

  double minimalDistanceBetweenTwoPoints_;
  double minimalVehicleSpeedToInsertPoint_;

  ros::Subscriber odom_sub_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_recorder");

  PathRecorder recorder;
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    recorder.init(nh,private_nh);
    ros::spin();
  }
  catch(std::exception & e)
  {
    ROS_ERROR_STREAM("PATH_RECORDER : "<<e.what());
  }
}
