std_msgs/Header header

uint8 target_id # station for 254, 255 for all

uint8 SETPOINT_GPS = 101
uint8 SETPOINT_LOCAL = 102
uint8 TAKEOFF = 103
uint8 LAND = 104
uint8 ARM = 105
uint8 DISARM = 106
uint8 HOLD = 107

uint8 command
# valid when command is 101
float64 latitude
float64 longitude
float64 altitude
# valid when command is 102
float64 x
float64 y
float64 z

float64 yaw