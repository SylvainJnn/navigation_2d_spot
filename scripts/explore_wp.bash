#!/bin/bash

# load waypoints into parameter server
rosparam load `rospack find navigation_2d_spot`/config/waypoints.yaml;

# let RoadmapServer know that wps are available in param server
rosservice call /rosplan_roadmap_server/load_waypoints;

# call husarion_explore_common.bash script
bash `rospack find navigation_2d_spot`/scripts/spot_go_to_point.bash
