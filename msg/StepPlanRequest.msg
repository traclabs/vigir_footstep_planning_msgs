std_msgs/Header header

vigir_footstep_planning_msgs/Feet start # if header doesn't contain a valid frame_id, the planner will use the robot pose

# used by 2D/3D planning mode
vigir_footstep_planning_msgs/Feet goal

uint32 start_step_index # index (offset) of first step the plan

# indicates which foot should be moved first
uint8 AUTO  = 0
uint8 LEFT  = 1
uint8 RIGHT = 2
uint8 start_foot_selection          # start foot selection mode

# used by pattern generator
vigir_footstep_planning_msgs/PatternParameters pattern_parameters

uint8 PLANNING_MODE_2D      = 0
uint8 PLANNING_MODE_3D      = 1
uint8 PLANNING_MODE_PATTERN = 2
uint8 planning_mode

float32 max_planning_time           # maximum planning time given in second, set 0 for default value
float32 max_number_steps            # maximum number of steps, set 0 for unlimited
float32 max_path_length_ratio       # maximum path length ratio computed as (current path length)/(sight line: start<->goal), must be larger 1 otherwise it will be ignored

std_msgs/String parameter_set_name  # name of parameter set to be used, keep empty to use default parameter set

uint8[] data # contains robot specific data

