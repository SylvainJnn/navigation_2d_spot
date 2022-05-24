#! /usr/bin/env python3
import rospy

from navigation_2d_spot.srv import *
from rosplan_dispatch_msgs.msg import ActionDispatch
from navigation_2d_spot.srv import DynamicParamsDouble


# Update yaw and xy goal tolerance given rosplan_waypoints_goal_tolerance in waypoints.yaml
class UpdateGoalToleranceUsingParam():

    def __init__(self):
        self.name = None      

    def update_tolerance_pram(self, req):
        rospy.wait_for_service('/dynamic_reconfig')       # Wait for servic which would dynamicly change the parametes (from dynamic_reconfiguration.py)
        try:
            waypoint_name = req.parameters[2].value           # Get the goal waypoint name 
            param = rospy.get_param('/rosplan_waypoints_goal_tolerance/'+str(waypoint_name))    # Get the yaw and xy tolerance for the specified waypoint
            set_param = rospy.ServiceProxy('/dynamic_reconfig', DynamicParamsDouble)      
            resp = set_param("xy_goal_tolerance", param[0])                 # Set xy_goal_tolerance parameter
            resp = set_param("yaw_goal_tolerance", param[1])                # Set yaw_goal_tolerance parameter
            return resp.sucksess
        except rospy.ServiceException as e:
            print("Sercice call failed: %s"%e)
  
            
    def main(self):
        rospy.init_node('update_goal_tolerance_using_param')
        # Listen to new goals being updated 
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch , self.update_tolerance_pram)
        rospy.spin()



if __name__ == "__main__":
    SR = UpdateGoalToleranceUsingParam()
    SR.main()
