#!/usr/bin/env python3

from __future__ import print_function

import sys 
import rospy 
from navigation_2d_spot.srv import *
from geometry_msgs.msg import Twist
from spot_msgs.srv import SetVelocity, SetVelocityResponse

def test_move():
    rospy.wait_for_service('/spot/set_goal')
    try:
        set_goal = rospy.ServiceProxy('/spot/set_goal', GoalSpotTrajectory)
        resp = set_goal('map',-2.0, 0.0, 0, 1.0, True)
        #resp = set_goal('map',-1.0, -2.0, 0, 30, True)
        return None
    except rospy.ServiceException as e:
        print("Sercice call failed: %s"%e)

def set_max_vel():
    rospy.wait_for_service('/spot/max_velocity')
    try:
        set_goal = rospy.ServiceProxy('/spot/max_velocity', SetVelocity)
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        move_cmd.linear.y = 0.3
        move_cmd.angular.z = 0.5
        resp = set_goal(move_cmd)
        return None
    except rospy.ServiceException as e:
        print("Sercice call failed: %s"%e)


def usage():
    return "%s path"%sys.argv[0]


if __name__=="__main__":
    #path = sys.argv
    #print("Requesting %s"%path)
    set_max_vel()
    
    #test_move()


