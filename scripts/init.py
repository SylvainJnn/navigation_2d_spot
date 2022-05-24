#!/usr/bin/env python
import rosplan_pytools
import rospy

if __name__=="__main__":
  rospy.init_node("name")
  rosplan_pytools.init()
  rospy.spin()

