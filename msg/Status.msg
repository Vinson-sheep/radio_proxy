std_msgs/Header header

uint8 id # 254 for station

float64 battery_v
uint8 display_mode  # see dji_sdk.h
uint8 flight_status # see dji_sdk.h
uint8 gps_health
uint8 arm_state # 0: disarm, 1: arm
uint8 land_state # 0: land, 1: onground, 2: air
