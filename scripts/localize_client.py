#! /usr/bin/env python3
import rospy
import roslib
import tf

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import tf2_ros
from geometry_msgs.msg import TransformStamped


class Localize():

    def __init__(self):
        self.success =True

    # Save fiduails using a service
    def localize(self):
        rospy.wait_for_service('/localize')  
        try:  
            set_param = rospy.ServiceProxy('/localize', SetBool)      
            resp = set_param(True)              
            return resp.success
        except rospy.ServiceException as e:
            print("Sercice call failed: %s"%e)
        
            
    def main(self):
        rospy.init_node('localize_client')
        print(self.localize())

        rospy.spin()



if __name__ == "__main__":
    SR = Localize()
    SR.main()
