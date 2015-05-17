std_msgs/Header header

vigir_footstep_planning_msgs/Foot foot
int32 step_index

float32 cost
float32 risk

bool valid      # indicates if this step can be reached from predecessor
bool colliding  # indicates if this step is colliding with the environment
bool locked
bool modified

# generic params
float32 sway_duration
float32 step_duration
float32 swing_height

uint8[] data # contains robot specific data
