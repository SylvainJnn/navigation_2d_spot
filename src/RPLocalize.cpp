#include "include/RPNavigate.h"

namespace KCL_rosplan {

    // constructor
    RPNavigate::RPNavigate(std::string &actionserver) : action_client_(actionserver, true) {

        // create a node handle to manage communication with ROS network
        ros::NodeHandle nh("~");

        // setup a move base clear costmap client (to be able to send clear costmap requests later on)
        // clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        ROS_INFO("Constructor");
    }

    bool RPNavigate::wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped &result) {
        ROS_INFO("Method");
        ros::NodeHandle nh;
        std::vector<double> wp;
        if(nh.hasParam(wp_namespace_ + "/" + wpID)) {
            if(nh.getParam(wp_namespace_ + "/" + wpID, wp)) {
                if(wp.size() == 3) {
                    result.header.frame_id = waypoint_frameid_;
                    result.pose.position.x = wp[0];
                    result.pose.position.y = wp[1];

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, wp[2]);
                    result.pose.orientation.x = q[0];
                    result.pose.orientation.y = q[1];
                    result.pose.orientation.z = q[2];
                    result.pose.orientation.w = q[3];

                    return true;
                }
                else {
                    ROS_ERROR("wp size must be equal to 3 : (x, y, and theta)");
                    return false;
                }
            }
        }
        else
            return false;
    }

    // action dispatch callback
    bool RPNavigate::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("Callback");

        // get waypoint ID from action dispatch msg
        std::string wpID;
        bool found = false;
        // iterating over parameters (e.g. kenny, wp0, wp1)
        for(size_t i = 0; i < msg->parameters.size(); i++) {
            // check their keys
            if(0 == msg->parameters[i].key.compare("to") or 0 == msg->parameters[i].key.compare("w1")) {
                // wp id found in msg params
                wpID = msg->parameters[i].value;
                found = true;
            }
        }
        if(!found) {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
            return false;
        }

        // get waypoint coordinates from its ID via query to parameter server
        geometry_msgs::PoseStamped pose;
        if(!wpIDtoPoseStamped(wpID, pose)) {
            ROS_ERROR("Waypoint not found in parameter server");
            return false;
        }

        ROS_INFO("KCL: (%s) waiting for navigate action server to start", params.name.c_str());
        action_client_.waitForServer();

        navigation_2d_spot::NavigateToGoalGoal final_goal;
        // std_msgs::float32 duration;
        // duration = 1.4;
        // std_msgs::Bool precise_positioning;
        // precise_positioning = true;
        geometry_msgs::PoseStamped final_pose;  // HERE

        final_goal.target_pose = pose;
        final_goal.duration = 6.0;
        final_goal.precise_positioning = true;
        action_client_.sendGoal(final_goal);

        bool finished_before_timeout = action_client_.waitForResult();
        if (finished_before_timeout) {

            actionlib::SimpleClientGoalState state = action_client_.getState();
            ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

                // publish feedback (achieved)
                return true;

            } else {

                // clear costmaps
                // std_srvs::Empty emptySrv;
                // clear_costmaps_client_.call(emptySrv);

                // publish feedback (failed)
                // Maybe do something if not succeded
                return false;
            }
        } else {
            // timed out (failed)
            action_client_.cancelAllGoals();
            ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
            return false;
        }
    }
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_navigate");

    ros::NodeHandle nh("~");

    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/navigate_to_goal_spot")); //HERE

    // create PDDL action subscriber
    KCL_rosplan::RPNavigate rpmb(actionserver);

    rpmb.runActionInterface();
    ros::spin(); // added

    return 0;
}
