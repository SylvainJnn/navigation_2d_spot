#!/usr/bin/env python3

import rospy
import rosservice

import tf
import time
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *

from navigation_2d_spot.srv import CreatePath, CreatePathResponse

query = []


# ROSPlan documentation 
"""
When working with Knowledge base, create this
d2 = KnowledgeUpdateServiceArrayRequest()
d2.update_type = [0,]

uint8 ADD_KNOWLEDGE=0
uint8 ADD_GOAL=1
uint8 REMOVE_KNOWLEDGE=2
uint8 REMOVE_GOAL=3
uint8 ADD_METRIC=4
uint8 REMOVE_METRIC=5

http://kcl-planning.github.io/ROSPlan//tutorials/tutorial_08

k.knowledge_type = 1

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

https://kcl-planning.github.io/ROSPlan//documentation/knowledge/03_KnowledgeItem.html


"""


# Set the current location as a home location 
def setPose(pose):
    # Old waypoint 
    old_waypoint = RemoveWaypointRequest()
    old_waypoint.id = 'home'
    # New home waypoint
    waypoint = AddWaypointRequest()
    waypoint.id = 'home'
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


def addInstance():
    an = KnowledgeUpdateServiceArrayRequest()
    an.update_type = [0,]
    a = KnowledgeItem()
    a.knowledge_type = 0
    a.instance_type = 'robot'
    a.instance_name = 'spot'
    a.attribute_name = ''
    a.function_value = 0.0
    an.knowledge.append(a)
    query.append(an)

def addFact():
    kn = KnowledgeUpdateServiceArrayRequest()
    kn.update_type = [0,]
    k = KnowledgeItem()
    k.knowledge_type = 1
    k.instance_type = ''
    k.instance_name = ''
    k.attribute_name = 'robot_at'
    v1 = KeyValue()
    v1.key = 'r'
    v1.value = 'husarion'
    k.values.append(v1)
    v2 = KeyValue()
    v2.key = 'wp'
    v2.value = 'home'
    k.values.append(v2)
    k.function_value = 0.0
    kn.knowledge.append(k)
    query.append(kn)


def setWaypoint():
    d = {}
    a = {}
    queryRemove = []
    d= KnowledgeUpdateServiceArrayRequest()
    #a["{0}".format(word)] = KnowledgeUpdateServiceArrayRequest() # remove visited places
    d.update_type = [1,] # Used to add to the knowlage base
    #a["{0}".format(word)].update_type = [2,] # Used to remove from the knowlage base 
    k = KnowledgeItem()
    k.knowledge_type = 1
    k.instance_type = ''
    k.instance_name = ''
    k.attribute_name = 'visited'
    v2 = KeyValue()
    v2.key = 'wp'
    v2.value = 'wp1'    # to visit place
    k.values.append(v2)
    k.function_value = 0.0
    d.knowledge.append(k)
    #a["{0}".format(word)].knowledge.append(k)
    #query.append(d)
    #queryRemove.append(a["{0}".format(word)])
    try:
        srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        #for i in query:
        resp = srv(d)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [update_array]: %s"%e)
"""
    for word in new_path.split():
        tolerance =  rospy.get_param("rosplan_waypoints_goal_tolerance/" + word)
        print(tolerance)
        d["{0}".format(word)] = KnowledgeUpdateServiceArrayRequest()
        a["{0}".format(word)] = KnowledgeUpdateServiceArrayRequest() # remove visited places
        d["{0}".format(word)].update_type = [1,] # Used to add to the knowlage base
        a["{0}".format(word)].update_type = [2,] # Used to remove from the knowlage base 
        k = KnowledgeItem()
        k.knowledge_type = 1
        k.instance_type = ''
        k.instance_name = ''
        k.attribute_name = 'visited'
        v2 = KeyValue()
        v2.key = 'wp'
        v2.value = word    # to visit place
        k.values.append(v2)
        k.function_value = 0.0
        d["{0}".format(word)].knowledge.append(k)
        a["{0}".format(word)].knowledge.append(k)
        query.append(d["{0}".format(word)])
        queryRemove.append(a["{0}".format(word)])


    try:
        srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        for i in queryRemove:
            resp = srv(i)
        for i in query:
            resp = srv(i)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [update_array]: %s"%e)
"""

def genProb():
    try:
        srv = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [problem_generation_server]: %s"%e)

def plan():
    try:
        srv = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [planning_server]: %s"%e)

def parsePlan():
    try:
        srv = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [parse_plan]: %s"%e)

def dispatch():
    try:
        srv = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed [dispatch_plan]: %s"%e)


def newPlan():
    print("Service was called to visit following waypoints: %s " %(1))
    addInstance()
    print("Instance added")
    addFact()
    print("Fact added")
    setWaypoint()
    print("Path was added")
    genProb()
    print('Problem generated')
    plan()
    print('Plan found')
    parsePlan()
    print('Plan parsed')
    dispatch()
    print('Plan dispatched')



def node_init():
    rospy.init_node('replan') # , anonymous=True
    listener = tf.TransformListener()
    print("Replan node created")
    listener.waitForTransform('/map', '/base_link',rospy.Time(), rospy.Duration(4.0))
    print("1")
    #rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  
    print("2")
    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    print("3")
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    print("4")
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    print("5")
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    print("6")
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')

    print("All rosplan services accounted for")

    try:
        pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Fail')
        exit()

    #setPose(pose)
    #setWaypointConnection()
    print("New pose is set")

    #s = rospy.Service('replan_using_a_new_plan', CreatePath, newPlan)
    newPlan()
    rospy.spin()

if __name__=="__main__":
    node_init()
