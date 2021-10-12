std_msgs/Header header

uint8 id # 255 for all

# GPS
float64 latitude
float64 longitude
float64 altitude
# local
float64 x
float64 y
float64 z

float64 vx
float64 vy
float64 vz

float64 ax
float64 ay
float64 az

float64 pitch
float64 roll
float64 yaw
float64 yaw_rate

# valid when gps health >= 3
float64 height_above_takeoff