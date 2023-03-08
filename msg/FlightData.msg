std_msgs/Header header

uint8 id # 255 for all

# GPS
float64 latitude
float64 longitude
float64 altitude

float64 latitude_rtk
float64 longitude_rtk
float64 altitude_rtk

# geometry_msgs/Vector3 position_local
# geometry_msgs/Vector3 position_local_rtk
# geometry_msgs/Vector3 velocity_local
# geometry_msgs/Vector3 velocity_local_rtk
# geometry_msgs/Vector3 accel_local
# geometry_msgs/Vector3 rpy
# geometry_msgs/Vector3 gimbal_angular

# float64 yaw_rate

# valid when gps health >= 3
float64 height_above_takeoff