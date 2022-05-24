#! /usr/bin/env python3
import rospy
import roslib
import rospy
import math
import tf

#from geometry_msgs.msg import Pose

import geometry_msgs.msg
from spot_msgs.msg import FiducialLocalization
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped   


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


# Loads the fiducial information, then uppon seeing it will update amcl pose topic

class UseFiducials():

    def __init__(self):
        print("")
        #self.tf_listener = None  
        #self.ymal_location = '/home/administrator/catkin_ws/src/navigation_2d_spot/config/fiducial_param.yaml' 
        #self.frame_to = '/body'
        
        #self.location_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        # /amcl_pose     /initialpose

        #self.diference = None
        #self.load_fiducials()
        #self.test = 0
        #self.pose_map = PoseStamped()
        #self.pose_body = PoseStamped()

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

    
    def add_fid_tf_test(self, pose_body):
        print("adding tf")
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "test1" #+  str()
        t.transform.translation.x = pose_body.pose.pose.position.x
        t.transform.translation.y = pose_body.pose.pose.position.y
        t.transform.translation.z = pose_body.pose.pose.position.z
        t.transform.rotation.x = pose_body.pose.pose.orientation.x
        t.transform.rotation.y = pose_body.pose.pose.orientation.y
        t.transform.rotation.z = pose_body.pose.pose.orientation.z
        t.transform.rotation.w = pose_body.pose.pose.orientation.w


        self.camera_static_transform_broadcaster.sendTransform(t)


############

    def set_fiducial_localization(self, req):

        rospy.sleep(1.)
        print("here")
        

        fidu_frame = rospy.get_param("/rosplan_demo_waypoints/wp/")  # Gets the parameter fiducial name 
        print(fidu_frame)

           #continue

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


    def testing(self):
        self.tf_listener.waitForTransform('/body', '/fiducial 209', rospy.Time(), rospy.Duration(4.0)) 
        try:
            (trans1,rot1) = self.tf_listener.lookupTransform('/body', '/fiducial 209', rospy.Time(0)) 
            (trans2,rot2) = self.tf_listener.lookupTransform('/body', '/fiducial 209', rospy.Time(0)) 

            diference  = self.transform_diff(trans1,rot1, trans2,rot2)
            print(diference)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("not working")
            
    def main(self):
        rospy.init_node('tf_listener')
        #self.tf_listener = tf.TransformListener()   
        #self.camera_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        rospy.Subscriber("/velodyne_points", PointCloud2, self.set_fiducial_localization) 



         
        rospy.spin()



if __name__ == "__main__":
    SR = UseFiducials()
    SR.main()
