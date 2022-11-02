#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class PerfectLocalisation:

  def __init__(self, robot_name):

    print(robot_name)
    self.rname = robot_name
    self.bfname = robot_name+"::"+robot_name+"/base_footprint"
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkStatesCallback)
    self.odom_pub = rospy.Publisher("perfect_odom", Odometry, queue_size=10)

  def linkStatesCallback(self, data):

    try:
      current_time = rospy.Time.now()
      ind = data.name.index(self.bfname)
      odom = Odometry()
      odom.header.stamp = current_time
      odom.header.frame_id = "map"
      odom.child_frame_id = self.rname+"/base_footprint"
      odom.pose.pose = data.pose[ind];
      odom.twist.twist = data.twist[ind];
      self.odom_pub.publish(odom)

    except ValueError:
      pass


if __name__ == '__main__':
  try:

    rospy.init_node('gazebo_localisation', anonymous=True)
    lc = PerfectLocalisation(rospy.get_param('~robot_name','robot'))
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
