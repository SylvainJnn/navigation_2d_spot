#! /usr/bin/env python3
import rospy

from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration
import geometry_msgs.msg

import actionlib
import spot_msgs.msg 

from tf.transformations import quaternion_from_euler
import numpy
import tf

from navigation_2d_spot.srv import GoalSpotTrajectory, GoalSpotTrajectoryResponse



class MoveTrajectoryCMD():

    def __init__(self):
        self.tf_listener = None      # Tf listener
        self.frame_to = None     
        self.client = None
        self.x = 0.0
        self.y = 0.0   
        self.yaw = 0.0 # Degress
        self.converted_transform = None
        self.frame_to = None
        self.duration = 0.0
        self.precise_positioning = None 
        


    def set_goal(self, converted_goal_msg):
        self.trajectory_goal = spot_msgs.msg.TrajectoryGoal()

        self.trajectory_goal.target_pose = converted_goal_msg
        duration =  rospy.Duration(self.duration)
        self.trajectory_goal.duration.data.secs = duration.secs
        self.trajectory_goal.duration.data.nsecs = duration.nsecs
        self.trajectory_goal.precise_positioning = self.precise_positioning
  
        self.client.send_goal(self.trajectory_goal)
        print("Goal send: ")
        print(self.trajectory_goal)


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
    def transformation_of_goal(self):
        print("Received goal: ")
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
            (trans_frame,rot_frame) = self.tf_listener.lookupTransform('body', 'map', rospy.Time(0))
            print("###########")
            print(trans_frame)
            print(rot_frame)
            print("###########")
            # The cordinates given by the user 
            trans_given = [self.x, self.y, 0.0]
            rot_given = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(self.yaw))
            # Diferance between given coordinates and map -> body 
            self.converted_transform  = self.transform_diff(trans_frame,rot_frame, trans_given,rot_given) 
            print("diferance")
            print(self.converted_transform)
            
            self.set_goal(self.update_goal()) # UPDATE GOAL
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed")

    
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
        rospy.init_node('testing')
        self.tf_listener = tf.TransformListener() 
        while not rospy.is_shutdown():

            try:
            # Transformation from map to robots own frame
                (trans_frame,rot_frame) = self.tf_listener.lookupTransform( '/body', '/map', rospy.Time(0))
                print(trans_frame)
                print(rot_frame)
                rospy.sleep(2.)
            # The cordinates given by the user 
            #trans_given = [self.x, self.y, 0.0]
            #rot_given = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(self.yaw))
            # Diferance between given coordinates and map -> body 
            #self.converted_transform  = self.transform_diff(trans_frame,rot_frame, trans_given,rot_given) 
            #print("diferance")
            #print(self.converted_transform)
            
            #self.set_goal(self.update_goal()) # UPDATE GOAL
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("failed")
        

        rospy.spin()



if __name__ == "__main__":
    SR = MoveTrajectoryCMD()
    SR.main()
