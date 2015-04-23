std_msgs/Header header

vigir_footstep_planning_msgs/Feet start # start configuration used by planner
vigir_footstep_planning_msgs/Feet goal  # goal configuration used by planner (must not be equal to reached goal)
vigir_footstep_planning_msgs/Step[] steps

uint8 mode # walking mode

uint8[] data # contains robot specific data
