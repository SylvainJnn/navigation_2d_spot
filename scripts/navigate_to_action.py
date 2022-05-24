#! /usr/bin/env python3

import rospy

import actionlib

from navigation_2d_spot.msg import NavigateToGoalAction, NavigateToGoalResult, NavigateToGoalFeedback
from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration
from spot_msgs.srv import SetVelocity, SetVelocityResponse
from navigation_2d_spot.srv import *
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Point 
from geometry_msgs.msg import Quaternion 
import geometry_msgs
import spot_msgs.msg 

from tf.transformations import quaternion_from_euler
import numpy
import tf

class NavigateToGoalSpot(object):
    # create messages that are used to publish feedback/result


    def __init__(self):
        self._action_name = "navigate_to_goal_spot"
        # tf listener 
        self.tf_listener = tf.TransformListener()  
        # Action server
        self._goal_server = actionlib.SimpleActionServer(self._action_name, NavigateToGoalAction, execute_cb=self.handle_navigate_to, auto_start = False)
        self._goal_server.start()
        # Pose gotten from ROSplan
        self.pose_map = None
        self.pose_body = None
        # Action client
        self.client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)
        self.client.wait_for_server()  
        # Pose to be send to spot_ros
        self.trajectory_goal = spot_msgs.msg.TrajectoryGoal()
        self.at_goal = False
        self.feedback = None
        self.timed_out_attempts = 0
        self.attempts = 4
        self.callback_result = None  


    ############### Callbacks from spot_ros ##############

    def callback_active(self):
        rospy.loginfo("NavigateToGoalSpot : Action server is processing the goal")


    def callback_done(self, state, result):
        rospy.loginfo("NavigateToGoalSpot : Action server is done. State: %s, result: %s" % (str(state), str(result)))
        self.callback_result = result
        if self.callback_result.success:
            self.at_goal = True



    def callback_feedback(self, feedback):
        self.feedback = str(feedback)
        if str(feedback) == "Preempted":
            print("here")
            #self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
            #self.trajectory_server.set_preempted()
    #############################
   
    def pose_conversion_from_map_to_body(self):
        try:
            self.pose_map.header.stamp = rospy.Time.now()
            t = rospy.Time(0)
            rospy.sleep(0.3)
            self.tf_listener.waitForTransform('/body','/map',t,rospy.Duration(5))
            if self.tf_listener.canTransform('/body','/map',t):
                mpose_transf = self.tf_listener.transformPose('body',self.pose_map)
                self.pose_body = self.tf_listener.transformPose('body',self.pose_map)
                return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("NavigateToGoalSpot : Failed to convert pose")
            return False

    ### Action server called by ROSplan ###      
    ### The error is here -> action methods have to be in this method 
    def handle_navigate_to(self, req):
        self.set_max_vel()

        self._goal_server.publish_feedback(NavigateToGoalFeedback("Started the action"))
        rospy.loginfo("NavigateToGoalSpot : Started the action server")
        try:
            # Pose from ROSPlan   
            self.pose_map =  req.target_pose
            self.pose_map.header.stamp = rospy.Time.now()

            rospy.sleep(0.1) 

            # Give duration and acuracy 
            duration =  rospy.Duration(req.duration)
            self.trajectory_goal.duration.data.secs = duration.secs
            self.trajectory_goal.duration.data.nsecs = duration.nsecs
            self.trajectory_goal.precise_positioning = False #req.precise_positioning

            self._goal_server.publish_feedback(NavigateToGoalFeedback("Goal recieved"))
            output_map_to_body = self.pose_conversion_from_map_to_body()
            rospy.loginfo("NavigateToGoalSpot : Goal recieved and converted to body frame")
            if output_map_to_body and not self.timed_out:
                self._goal_server.publish_feedback(NavigateToGoalFeedback("Converted from map frame to body"))
                # Send goal to spot
                self.trajectory_goal.target_pose = self.pose_body
                self.client.send_goal(self.trajectory_goal, active_cb=self.callback_active,
                    feedback_cb=self.callback_feedback,
                    done_cb=self.callback_done)

                self.client.wait_for_result()   #  Wait to get back results from wrapper

                # Loop to chek for feedback and update states 
                while not rospy.is_shutdown() and self._goal_server.is_active() and not self.at_goal and self.timed_out_attempts < self.attempts:
                    
                    if str(self.callback_result.message) == "Failed to reach goal, timed out":
                        rospy.loginfo("NavigateToGoalSpot : Timed out, resending the goal")
                        self.timed_out_attempts = self.timed_out_attempts + 1 # If a goal can not be reached in given number of attemst
                        # Cancell the curent goals and give new ones
                        self.client.cancel_all_goals() 
                        # Send new stamped pose to the robot 
                        self.pose_conversion_from_map_to_body()
                        self.trajectory_goal.target_pose = self.pose_body
                        self.client.send_goal(self.trajectory_goal, active_cb=self.callback_active,
                            feedback_cb=self.callback_feedback,
                            done_cb=self.callback_done)
                        self.client.wait_for_result() #  Wait to get back results from wrapper
                    elif str(self.callback_result.message) == "Duration must be larger than 0":
                        self._goal_server.set_aborted(NavigateToGoalResult(False, "Duration must be larger than 0"))
                    elif str(self.callback_result.message) == "Reached goal":
                        self._goal_server.set_succeeded(NavigateToGoalResult(True, "Goal reached"))
                    elif str(self.callback_result.message) == "Failed to reach goal":
                        self._goal_server.set_aborted(NavigateToGoalResult(False, "Failed to reach goal"))

                # Chek if the time out excceded its run, it succesfully reached the goal, or it failed to reach the goal
                if self.timed_out_attempts == self.attempts:
                    self._goal_server.set_aborted(NavigateToGoalResult(False, "Was not able to complete in time"))
                    rospy.loginfo("NavigateToGoalSpot : Was not able to complete in time. Goal ABORTED")
                elif str(self.callback_result.message) == "Success" or str(self.callback_result.success) == "True":
                    self._goal_server.set_succeeded(NavigateToGoalResult(True, "Goal reached")) 
                    rospy.loginfo("NavigateToGoalSpot : Goal reached. Goal SUCCEEDED")
                elif str(self.callback_result.success) == "False":
                    self._goal_server.set_aborted(NavigateToGoalResult(False, "Failed to reach the goal"))
                    rospy.loginfo("NavigateToGoalSpot : Failed to reach the goal. Goal ABORTED")
                
                # Reset the timout and at goal for next implementation
                self.at_goal = False 
                self.timed_out_attempt = 0
                
            elif not output_map_to_body:
                rospy.loginfo("NavigateToGoalSpot : Failed to convert pose. Goal ABORTED")
                self._goal_server.set_succeeded(NavigateToGoalResult(False, "Failed to convert pose"))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self._goal_server.set_aborted(NavigateToGoalResult(False, "Failed due to tf exception"))

    def set_max_vel(self):
        rospy.wait_for_service('/spot/max_velocity')
        try:
            set_goal = rospy.ServiceProxy('/spot/max_velocity', SetVelocity)
            move_cmd = Twist()
            move_cmd.linear.x = 0.25 # 0.3
            move_cmd.linear.y = 0.25 # 0.3
            move_cmd.angular.z = 0.4 #0.5
            resp = set_goal(move_cmd)
            rospy.loginfo("NavigateToGoalSpot : Velocity limit is set")
            return None
        except rospy.ServiceException as e:
            print("Sercice call failed: %s"%e)

        
if __name__ == '__main__':
    rospy.init_node('navigate_to_goal_spot')
    server = NavigateToGoalSpot()
    rospy.spin()
