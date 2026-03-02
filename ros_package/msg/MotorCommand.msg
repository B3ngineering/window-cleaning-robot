std_msgs/Header Header

uint8 motor_id          # 0=top_left, 1=top_right, 2=bottom_left, 3=bottom_right
int32 steps             # Number of steps to move
float32 speed_rpm       # Desired speed, max (?)
int8 direction          # 1 = shorten cable, -1 = lengthen cable
bool stop               # False=running, True=hold in place
