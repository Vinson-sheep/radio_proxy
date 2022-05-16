std_msgs/Header header

uint8 id # station for 254, 255 for all

uint8 SETPOINT_GPS = 0
uint8 SETPOINT_LOCAL = 1
uint8 TAKEOFF = 2
uint8 LAND = 3
uint8 ARM = 4
uint8 DISARM = 5
uint8 HOLD = 6
uint8 TRACKING = 7
uint8 GO_HOME = 8

uint8 mission
float64 latitude
float64 longitude
float64 altitude
float64 x
float64 y
float64 z

float64 yaw