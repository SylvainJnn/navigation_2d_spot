#! /usr/bin/env python3
import rospy

from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration

from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import Point 
import geometry_msgs.msg

import actionlib
import spot_msgs.msg 

from tf.transformations import quaternion_from_euler
import numpy
import tf
import tf2_py

from navigation_2d_spot.srv import GoalSpotTrajectory, GoalSpotTrajectoryResponse

        # 1. Find out where the robot is on the map using transformation /body -> /map
        # 2. Where is point you want to go on the map 
        # 3. How far away is the robot from the point (everything is on map frame) 
        # 4. Give the robot point to go 

class MoveTrajectoryCMD():

    def __init__(self):
        self.tf_listener = None      # Tf listener
        self.frame_to = None     
        self.client = None
        self.x = 0.0
        self.y = 0.0   
        self.yaw = 0.0 # Degress
        self.converted_transform = None
        self.duration = 0.0
        self.precise_positioning = None 
        self.pose_map = None
        


    def set_goal(self, converted_goal_msg):
        self.trajectory_goal = spot_msgs.msg.TrajectoryGoal()

        self.trajectory_goal.target_pose = converted_goal_msg
        duration =  rospy.Duration(self.duration)
        self.trajectory_goal.duration.data.secs = duration.secs
        self.trajectory_goal.duration.data.nsecs = duration.nsecs
        self.trajectory_goal.precise_positioning = self.precise_positioning
  
        self.client.send_goal(self.trajectory_goal, active_cb=self.callback_active,
                    feedback_cb=self.callback_feedback,
                    done_cb=self.callback_done)

    def callback_active(self):
        print("Action server is processing the goal")
        #rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        #rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        if str(result.message) == "Failed to reach goal, timed out":
            print("!!!!!!!")
            self.pose_conversion_from_map_to_body()
        # frame_id of target_pose must be \'body\'
        # duration must be larger than 0
        # self.trajectory_server.publish_feedback(TrajectoryFeedback("Reached goal"))
        # self.trajectory_server.set_succeeded(TrajectoryResult(resp[0], resp[1]))
        # Failed to reach goal
        

    def callback_feedback(self, feedback):
        #rospy.loginfo("Feedback:%s" % str(feedback))
        print("Feedback:%s" % str(feedback))
        if str(feedback) == "Failed to reach goal, timed out":
            print("here")


    def update_goal(self):
        converted_goal_msg = PoseStamped()
        converted_goal_msg.header.frame_id = 'body'
        converted_goal_msg.header.stamp = rospy.Time.now()

        converted_goal_msg.pose.position.x = self.converted_transform.translation.x
        converted_goal_msg.pose.position.y = self.converted_transform.translation.y
        converted_goal_msg.pose.orientation.x = self.converted_transform.rotation.x
        converted_goal_msg.pose.orientation.y = self.converted_transform.rotation.y
        converted_goal_msg.pose.orientation.z = self.converted_transform.rotation.z
        converted_goal_msg.pose.orientation.w = self.converted_transform.rotation.w    
        return converted_goal_msg


    ### Transformation from map to body frame ###
    def transformation_of_goal(self, req):
        print(req)
        self.x = req.x
        self.y = req.y
        self.yaw = req.yaw
        self.frame_to = req.frame_id
        self.duration = req.duration
        self.precise_positioning = req.precise_positioning
        # transformation of goal 
        try:
            # Transformation from map to robots own frame
            #(trans_frame, rot_frame) = self.tf_listener.lookupTransform('/map', '/body', rospy.Time(0)) # THIS IS CORRECT
            # The cordinates given by the user 
            #trans_given = [self.x, self.y, 0.0]

            rot_given = tf.transformations.quaternion_from_euler(0.0, 0.0, numpy.deg2rad(self.yaw))
            ###################
            self.pose_map = PoseStamped()
    
            self.pose_map.pose.position.x = self.x
            self.pose_map.pose.position.y = self.y
            self.pose_map.pose.position.z = 0
    
            self.pose_map.pose.orientation.x = rot_given[0]
            self.pose_map.pose.orientation.y = rot_given[1]
            self.pose_map.pose.orientation.z = rot_given[2]
            self.pose_map.pose.orientation.w = rot_given[3]
    
            self.pose_map.header.frame_id = '/map'
            self.pose_map.header.stamp = rospy.Time.now()

            self.pose_conversion_from_map_to_body()
            #t = rospy.Time(0)
            rospy.sleep(0.5)
            #self.tf_listener.waitForTransform('/body','/map',t,rospy.Duration(5))
            #if self.tf_listener.canTransform('/body','/map',t):
            #    mpose_transf = self.tf_listener.transformPose('body',self.pose_map)
            #    print(mpose_transf)


            # Diferance between given coordinates and map -> body 
            #self.converted_transform  = self.transform_diff(trans_frame, rot_frame, trans_given, rot_given)    #why this fucker decided not to work 
            #print(self.converted_transform)         
            #self.set_goal(mpose_transf) # UPDATE GOAL
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Tf failed to make pose")

    def pose_conversion_from_map_to_body(self):
        try:
            self.pose_map.header.stamp = rospy.Time.now()
            print(self.pose_map)
            t = rospy.Time(0)
            rospy.sleep(0.5)
            self.tf_listener.waitForTransform('/body','/map',t,rospy.Duration(5))
            if self.tf_listener.canTransform('/body','/map',t):
                mpose_transf = self.tf_listener.transformPose('body',self.pose_map)
                self.set_goal(mpose_transf)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Failed to convert pose")
                    
    
    # Transform to homogeneous matrix
    def transform_to_homogeneous_matrix(self, trans, rot):
        #It says Quat to euler sxyz, but the order of XYZW is fine. Isn't it a little confusing?
        tfeul= tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]],axes='sxyz')
        tfobjM = tf.transformations.compose_matrix(angles=tfeul,translate=trans)
        # return
        return  tfobjM
    
    def homogeneous_transform(self, Mat):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(Mat)
        quat = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
        tfobj = geometry_msgs.msg.Transform()
        tfobj.rotation.x = quat[0]
        tfobj.rotation.y = quat[1]
        tfobj.rotation.z = quat[2]
        tfobj.rotation.w = quat[3]
        tfobj.translation.x = trans[0]
        tfobj.translation.y = trans[1]
        tfobj.translation.z = trans[2]
        return tfobj
        
    # Transform diff tf1 to 2
    def transform_diff(self, trans1, rot1,trans2, rot2):
        tf1M = self.transform_to_homogeneous_matrix(trans1, rot1)
        tf2M = self.transform_to_homogeneous_matrix(trans2, rot2)
        return  self.homogeneous_transform(tf2M.dot(tf.transformations.inverse_matrix(tf1M)))

            
    def main(self):
        rospy.init_node('move_trajectory_cmd')
        self.tf_listener = tf.TransformListener()  
        # Action library for spot trajectory from SDK 
        self.client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)
        self.client.wait_for_server()  
        print("server received")
        # Service to call set the goal 
        rospy.Service('/spot/set_goal', GoalSpotTrajectory, self.transformation_of_goal) 
        

        rospy.spin()



if __name__ == "__main__":
    SR = MoveTrajectoryCMD()
    SR.main()
