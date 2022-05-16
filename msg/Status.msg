std_msgs/Header header

# flight state
uint8 STOPED = 0
uint8 ON_GROUND = 1
uint8 IN_AIR = 2

uint8 id # 254 for station

float64 battery_v
uint8 gps_health
bool armed
uint8 flight_state
bool rc_connected
bool rtk_connected
uint8 mission
bool connected
string mode
