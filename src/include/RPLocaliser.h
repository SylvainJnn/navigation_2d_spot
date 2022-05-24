#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>

#ifndef KCL_localize
#define KCL_localize

/**
 * This file defines the RPMoveBase class.
 * RPMoveBase is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by id from the parameter server.
 */

namespace KCL_rosplan {

    class RPLocalize: public RPActionInterface
    {

      public:

        /**
         * @brief constructor
         */
        RPNavigate(std::string &server);

        /**
         * @brief listen to and process action_dispatch topic
         * @param msg this parameter comes from the topic subscription, it contains the request (id, name, params, etc)
         * @return true if execution was successful
         */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

      private:


        // to clear costmaps if move base gets stuck
        ros::ServiceClient localize_server_;

        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle nh_;

    };
}
#endif
