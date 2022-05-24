#! /usr/bin/env python3

import rospy

import actionlib

from navigation_2d_spot.msg import NavigateToGoalAction, NavigateToGoalResult, NavigateToGoalFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration

from geometry_msgs.msg import Point 
from geometry_msgs.msg import Quaternion 
import spot_msgs.msg 

import numpy

class NavigateToGoalSpot(object):
    # create messages that are used to publish feedback/result
    _feedback = NavigateToGoalFeedback()
    _result = NavigateToGoalResult()

    def __init__(self):
        self._action_name = "navigate_to_goal_spot"
        self.converted_transform = None
        self._goal_server = actionlib.SimpleActionServer(self._action_name, NavigateToGoalAction, execute_cb=self.handle_navigate_to, auto_start = False)
        self._goal_server.start()



    ### Transformation from map to body frame ###      
    def handle_navigate_to(self, req):
        success = True 
        try:
            print(req.target_pose.pose.position)
            print(req.target_pose.pose.orientation)
            self._goal_server.set_succeeded(NavigateToGoalResult(True, "Goal reached"))
        except ():
            self._goal_server.set_aborted(NavigateToGoalResult(False, "Failed"))
        

        
if __name__ == '__main__':
    rospy.init_node('navigate_to_goal_spot')
    server = NavigateToGoalSpot()
    rospy.spin()
