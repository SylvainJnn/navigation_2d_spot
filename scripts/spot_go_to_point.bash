#!/bin/bash

# get parrameter from the fille
planned_path=$(rosparam get /upload_plan/path)

# add robot spot instance + goals: visit all waypoint instances
echo "Adding initial state and goals to knowledge base.";
param_type="update_type:
- 0";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'spot'
  attribute_name: ''
  function_value: 0.0";
param_type="$param_type
- 0";
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'r', value: 'spot'}
  - {key: 'wp', value: 'home'}
  function_value: 0.0";

# generate waypoints 
for i in $planned_path
do
echo "values  = ${i}"
param_type="$param_type
- 1"
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: '${i}'} 
  function_value: 0.0"
done;

rosservice call /rosplan_knowledge_base/update_array "
$param_type
$param"

# NOTE: robot_at(husarion wp0) gets added by the mapping interface

# automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

# make plan (e.g. call popf to create solution)
echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

# parse plan (parse console output and extract actions and params, e.g. create esterel graph)
echo "Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;

# dispatch (execute) plan. (send actions one by one to their respective interface and wait for them to finish)
echo "Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;

echo "Finished!";
