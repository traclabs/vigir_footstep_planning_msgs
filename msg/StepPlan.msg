std_msgs/Header header

std_msgs/String parameter_set_name  # name of parameter set to be used, keep empty to use default parameter set

vigir_footstep_planning_msgs/Feet start # start configuration used by planner
vigir_footstep_planning_msgs/Feet goal  # goal configuration used by planner (must not be equal to reached goal)
vigir_footstep_planning_msgs/Step[] steps

uint8 mode # walking mode

uint8[] data # contains robot specific data
