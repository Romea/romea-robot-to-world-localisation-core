#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from threading import Lock

class LocalisationChecker:

  def __init__(self, robot_name):

    self.bfname = robot_name+"::"+robot_name+"/base_footprint"
    self.mutex = Lock()
    self.gpose = Pose()
    self.gfposition = Point()
    self.flag = False

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkStatesCallback)
    self.odom_sub = rospy.Subscriber("/"+robot_name+"/localisation/filtered_odom", Odometry, self.odometryCallback)


  def linkStatesCallback(self, data):

    try:
      ind = data.name.index(self.bfname)
      self.mutex.acquire()
      self.gpose = data.pose[ind]
      if self.flag is not True :
         self.gfposition=self.gpose.position
         self.flag=True
      self.mutex.release()
    except ValueError:
      pass

  def odometryCallback(self, data):

      self.mutex.acquire()
      pg = self.gpose;
      self.mutex.release()

      pg.position.x-=self.gfposition.x
      pg.position.y-=self.gfposition.y

      po = data.pose.pose

      qg = pg.orientation
      (erg, epg, eyg) = euler_from_quaternion([qg.x,qg.y,qg.z,qg.w])

      qo = po.orientation
      (ero, epo, eyo) = euler_from_quaternion([qo.x,qo.y,qo.z,qo.w])


      print("%.3f" % pg.position.x,
            "%.3f" % po.position.x,
            "%.3f" % pg.position.y,
            "%.3f" % po.position.y,
            "%.3f" % pg.position.z,
            "%.3f" % po.position.z,
            "%.3f" % erg,
            "%.3f" % ero,
            "%.3f" % epg,
            "%.3f" % epo,
            "%.3f" % eyg,
            "%.3f" % eyo)


if __name__ == '__main__':
  try:

    rospy.init_node('checker', anonymous=True)
    lc = LocalisationChecker(rospy.get_param('~robot_name','robot'))
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
