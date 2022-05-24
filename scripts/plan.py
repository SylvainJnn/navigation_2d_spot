#!/usr/bin/env python3

from __future__ import print_function

import sys 
import rospy 
import tf
#import tf2_ros
from navigation_2d_spot.srv import *

from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *


def create_plan(new_path):
    rospy.wait_for_service('replan_using_a_new_plan')
    try:
        set_plan = rospy.ServiceProxy('replan_using_a_new_plan', CreatePath)
        resp = set_plan(new_path)
        return new_path
    except rospy.ServiceException as e:
        print("Sercice call failed: %s"%e)

def setPose(pose):
    # Old waypoint 
    old_waypoint = RemoveWaypointRequest()
    old_waypoint.id = 'home'
    # New home waypoint
    waypoint = AddWaypointRequest()
    waypoint.id = 'wwww1'
    waypoint.waypoint.pose.position.x = pose[0][0]
    waypoint.waypoint.pose.position.y = pose[0][1]
    waypoint.waypoint.pose.position.z = pose[0][2]
    waypoint.waypoint.pose.orientation.x = pose[1][0]
    waypoint.waypoint.pose.orientation.y = pose[1][1]
    waypoint.waypoint.pose.orientation.z = pose[1][2]
    waypoint.waypoint.pose.orientation.w = pose[1][3]
  
    #waypoint.connecting_distance = 10
    try:
        # Add waypoint using service 
        add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
        print("Adding new home waypoint [ %s %s %s ] "%(round(pose[0][0], 2), round(pose[0][1], 2), pose[0][2]))
        resp1 = add_waypoint(waypoint)
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


def usage():
    return "%s path"%sys.argv[0]


if __name__=="__main__":
    #path = sys.argv
    #print("Requesting %s"%path)
    ## rospy.init_node('plan') ## needed for when adding waypoints
    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
    print("got ")
    try:
        pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Fail')
        exit()

    #### setPose(pose) #adding waypoint

    create_plan("wp2 wp1 wp3")

