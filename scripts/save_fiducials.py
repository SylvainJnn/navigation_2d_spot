#! /usr/bin/env python3
import rospy
import roslib
import math
import tf
import yaml


from spot_msgs.msg import FiducialLocalization
from std_srvs.srv import Empty

import tf2_ros
from geometry_msgs.msg import TransformStamped

# Works by calling a service to save the fiducials from tf frame 
# rosservice call /save_fiducial "fiducial_name: ["fiducial_206", "fiducial_208", "fiducial_209"]"

class FiducialToFile():

    def __init__(self):
        self.tf_listener = None      # Tf listener
        self.yaml_location = None    # Location to store ymal files 
        self.frame_to = None         # Frame the fiducial will be transformed to ("map")
        self.all_fiducials = []

    # Save fiduails using a service
    def save_fiducials(self, req):
        param_file = {}
        for x in self.all_fiducials:
            self.tf_listener.waitForTransform(self.frame_to, x, rospy.Time(), rospy.Duration(4.0)) # Wait for the transform
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.frame_to, x, rospy.Time(0)) # Get the transform
                param_file[x] = [trans, rot] # Store it in the dictionary 
                self.add_fid_tf_test(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("tf exception")
        
        with open(self.yaml_location, 'w') as file:
            yaml.dump(param_file, file) #  Load the parameters in to the file
        
    def add_fid_tf_test(self, trans,rot):
        print("adding tf")
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "test" #+  str()
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        #self.broadcaster.sendTransform((trans[0], trans[1], trans[2]),
         #            (rot[0], rot[1], rot[2], rot[3]),
          #           rospy.Time.now(),
           #          "/test",
            #         "/vision")
        self.camera_static_transform_broadcaster.sendTransform(t)



    def save_fiducials_to_list(self, req):
        for x in req.fiducial_names:
            if not x in self.all_fiducials:
                self.all_fiducials.append(x)
  
            
    def main(self):
        rospy.init_node('save_fiducial')
        self.yaml_location = '/home/administrator/catkin_ws/src/navigation_2d_spot/config/fiducial_param.yaml' 
        self.tf_listener = tf.TransformListener()  
        self.frame_to = '/map' 
        #self.broadcaster = tf.TransformBroadcaster()
        self.camera_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.Service('/save_fiducials', Empty, self.save_fiducials)
        rospy.Subscriber('/spot/fiducial', FiducialLocalization, self.save_fiducials_to_list)

        rospy.spin()



if __name__ == "__main__":
    SR = FiducialToFile()
    SR.main()
