# Define status codes for footstep execution
uint32 NO_ERROR                         =    0
uint32 REACHED_GOAL                     =    1
uint32 WAITING_FOR_ATLAS                =    2
uint32 EXECUTING_STEP_PLAN              =    4
uint32 PREMATURE_HALT                   =    8
uint32 UPDATED_STEP_PLAN                =   16
uint32 EMERGENCY_HALT                   =   32

# Error codes
uint32 ERR_UNKNOWN                      =  256
uint32 ERR_EMPTY_PLAN                   =  512
uint32 ERR_INVALID_BEHAVIOR_MODE        = 1024
uint32 ERR_INVALID_PLAN                 = 2048
uint32 ERR_INVALID_INCONSISTENT_INDICES = 4096
uint32 ERR_INCONSISTENT_STEP_PLAN       = 8192
uint32 ERR_CONTROLLER_NOT_READY         = 16384
uint32 ERR_FALLEN                       = 32768
uint32 ERR_NON_ZERO_INITIAL_STEP        = 65536
uint32 ERR_NEXT_ERROR                   = 131072

Header header
uint32 status

