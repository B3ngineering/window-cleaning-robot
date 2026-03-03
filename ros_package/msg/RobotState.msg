std_msgs/Header header

float32 x
float32 y

float32 cable_length_tl
float32 cable_length_tr
float32 cable_length_bl
float32 cable_length_br

int32 steps_motor_tl
int32 steps_motor_tr
int32 steps_motor_bl
int32 steps_motor_br

uint8 status
uint8 STATUS_IDLE=0
uint8 STATUS_MOVING=1
uint8 STATUS_AT_TARGET=2
uint8 STATUS_ESTOP=3
uint8 STATUS_ERROR=4