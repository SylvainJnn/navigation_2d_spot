#! /usr/bin/env python3
import rospy
import roslib
import tf

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import tf2_ros

import geometry_msgs.msg
from spot_msgs.msg import FiducialLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped   

import math
from rosparam import upload_params
from yaml import load
import yaml 
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped


from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
                               quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np



class Localize():

    def __init__(self):
        self.success = None
        self.tf_listener = None  
        self.ymal_location = '/home/administrator/catkin_ws/src/navigation_2d_spot/config/fiducial_param.yaml' 
        self.frame_to = '/body'
        
        self.location_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10) # /amcl_pose     /initialpose
        self.diference = None
        self.load_fiducials()
        self.test = 0
        self.pose_map = PoseStamped()
        self.pose_body = PoseStamped()
        self.timed_out = 3
        self.localized = False

    # Load fiducial as parameters 
    def load_fiducials(self):
        with open(self.ymal_location) as file:
            yamlfile = yaml.load(file, Loader=yaml.FullLoader)
            upload_params('', yamlfile)
            print("loaded the file")
    
    def update_localization(self):
        chekpoint = PoseWithCovarianceStamped()

        chekpoint.pose.pose.position.x = self.diference.translation.x
        chekpoint.pose.pose.position.y = self.diference.translation.y
        chekpoint.pose.pose.position.z = self.diference.translation.z

        chekpoint.pose.pose.orientation.x = self.diference.rotation.x
        chekpoint.pose.pose.orientation.y = self.diference.rotation.y
        chekpoint.pose.pose.orientation.z = self.diference.rotation.z
        chekpoint.pose.pose.orientation.w = self.diference.rotation.w
        return chekpoint

    def set_fiducial_localization(self, req):
        rospy.sleep(1)
        
        for x in req.fiducial_names:
            try:
                (trans1,rot1) = self.tf_listener.lookupTransform(self.frame_to, x, rospy.Time(0))

                # Sees if the fiducial it is seing is added to the map 
                try:
                    fidu_frame = rospy.get_param(x)  # Gets the parameter fiducial name 
                except:
                    break

                    # Saved diference between fiducial - > map, new diference between fiducial -> body / base_link 
                    # Finding the current location relative to the map 
                diference  = self.transform_diff(trans1,rot1, fidu_frame[0],fidu_frame[1]) 

                    # Only update initial pose once or when the fiducial changes
                if self.diference is None:
                    self.diference = diference

                    chekpoint = self.update_localization()

                    self.location_pub.publish(chekpoint)
                    # Test the TF
                    #self.add_fid_tf_test(chekpoint) 
                else: 
                    x = self.diference.translation.x/diference.translation.x
                    y = self.diference.translation.y/diference.translation.y
                    w = self.diference.rotation.z/diference.rotation.z
                    # If the diference between fiducials is great enough update the difereance and publish to initial pose
                    if x < 0.7 or y <0.7 or w <0.5: 
                        self.diference = diference
                        #rospy.sleep(1.)
                        location = self.update_localization()
                        print(location)
                        self.location_pub.publish(location)
                

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("failed")

    #### Finding the correct location of robot relative to the map using the faducial 
    
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

    

    # Save fiduails using a service
    def localize(self, req):
        old_time = rospy.Time.now().to_sec()
        diff_time = 0

        while diff_time < self.timed_out and not rospy.is_shutdown():
            rospy.sleep(0.2)
            diff_time = rospy.Time.now().to_sec() - old_time
            if self.diference is not None:
                return SetBoolResponse(True, "Was able to localize")
        return SetBoolResponse(False, "Could not licalize in the given time")
            
    def main(self):
        rospy.init_node('localize')
        self.tf_listener = tf.TransformListener()   
        rospy.Service('/localize', SetBool, self.localize)
        rospy.Subscriber("/spot/fiducial", FiducialLocalization, self.set_fiducial_localization)

        rospy.spin()



if __name__ == "__main__":
    SR = Localize()
    SR.main()
