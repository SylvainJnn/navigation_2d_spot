#!/usr/bin/env python3
import rospy

from dynamic_reconfigure.msg import Config
from navigation_2d_spot.srv import DynamicParamsDouble , DynamicParamsDoubleResponse

# Service to update the double Config parameters 
class SetDynamicParam():

    def __init__(self):
        self.all_param = None


    def listenToParam(self, req):
        self.all_param = req

    def updateParam(self, req): 
        # Find the name of the parameter you want to update 
        for i in range(len(self.all_param.doubles)):
            if (self.all_param.doubles[i].name == req.name): 
                # Publish the value for the given parameter
                self.all_param.doubles[i].value = req.value
                self.pub.publish(self.all_param)
                return DynamicParamsDoubleResponse(True)
        return DynamicParamsDoubleResponse(False)

            
    def main(self):
        rospy.init_node('dynamic_reconfiguration')
        # Subscribe to dynamic reconfigure topic
        rospy.Subscriber('/move_base/DWAPlannerROS/parameter_updates', Config, self.listenToParam)
        # Service to reconfigure dft.doubles
        rospy.Service('/dynamic_reconfig', DynamicParamsDouble, self.updateParam)
        # Publish the updated parameters 
        self.pub = rospy.Publisher('/move_base/DWAPlannerROS/parameter_updates', Config, queue_size = 10)

        rospy.spin()



if __name__ == "__main__":
    SR = SetDynamicParam()
    SR.main()
