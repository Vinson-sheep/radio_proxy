## 数传电台传输协议

*Vinson Sheep*

*20220516*

### 1. 数据结构

| HEAD      | MSG_ID     | TARGET_ID  | LOCAL_ID | PAYLOAD | END       |
| --------- | ---------- | ---------- | -------- | ------- | --------- |
| 0X5A 0X0B | 命令辨识码 | 目标飞机ID | 本机ID   | 数据段  | 0X0D 0X0A |
| 2字节     | 1个字节    | 1字节      | 1字节    | LEN-6   | 2字节     |

> TARGET_ID/ LOCAL_ID ： 地面站默认为254， 全体飞机默认为255；
>
> MSG_ID ：0号位保留；0-100为下行；101-254为上行；255为专用消息通道；

### 2. 消息内容

----

#### 飞行数据

MSG_ID: 1

发送频率: 5HZ

PAYLOAD结构: 

| 偏移量 | 数据类型 | 名称                 | 注释                                      |
| ------ | -------- | -------------------- | ----------------------------------------- |
| 0      | float32  | latitude             | GPS纬度 (度)                              |
| 4      | float32  | longitude            | GPS经度 (度)                              |
| 8      | float32  | altitude             | GPS海拔 (度)                              |
| 12     | float32  | latitude_rtk         | GPS纬度 (度) RTK                          |
| 16     | float32  | longitude_rtk        | GPS经度 (度) RTK                          |
| 20     | float32  | altitude_rtk         | GPS海拔 (度) RTK                          |
| 24     | float32  | position_local_x     | 局部位置--x  (NED坐标，m)                 |
| 28     | float32  | position_local_y     | 局部位置--y  (NED坐标，m)                 |
| 32     | float32  | position_local_z     | 局部位置--z  (NED坐标，m)                 |
| 36     | float32  | position_local_x_rtk | 局部位置--x  (NED坐标，m) RTK             |
| 40     | float32  | position_local_y_rtk | 局部位置--y  (NED坐标，m) RTK             |
| 44     | float32  | position_local_z_rtk | 局部位置--z  (NED坐标，m) RTK             |
| 48     | float32  | velocity_local_x     | 局部速度--vx (NED坐标，m/s)               |
| 52     | float32  | velocity_local_y     | 局部速度--vy (NED坐标，m/s)               |
| 56     | float32  | velocity_local_z     | 局部速度--vz (NED坐标，m/s)               |
| 60     | float32  | velocity_local_x_rtk | 局部速度--vx (NED坐标，m/s) RTK           |
| 64     | float32  | velocity_local_y_rtk | 局部速度--vy (NED坐标，m/s) RTK           |
| 68     | float32  | velocity_local_z_rtk | 局部速度--vz (NED坐标，m/s) RTK           |
| 72     | float32  | accel_local_x        | 局部加速度--ax (NED坐标，m/s<sup>2</sup>) |
| 76     | float32  | accel_local_y        | 局部加速度--ay (NED坐标，m/s<sup>2</sup>) |
| 80     | float32  | accel_local_z        | 局部加速度--ax (NED坐标，m/s<sup>2</sup>) |
| 84     | float32  | roll                 | 横滚角                                    |
| 88     | float32  | pitch                | 俯仰角                                    |
| 92     | float32  | yaw                  | 偏航角                                    |
| 96     | float32  | gimbal_angular_x     | 云台角x                                   |
| 100    | float32  | gimbal_angular_y     | 云台角y                                   |
| 104    | float32  | gimbal_angular_z     | 云台角z                                   |
| 108    | float32  | yaw_rate             | 偏航速度 (rad/s)                          |
| 112    | float32  | height_above_takeoff | 起飞高度（解锁后可用，m）                 |

> 务必确保gps_health在4以上

---

### 状态数据

MSG_ID: 2

发送频率: 2HZ

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称          | 注释                                   |
| ------ | -------- | ------------- | -------------------------------------- |
| 0      | float32  | battery_v     | 电池电压 (V)                           |
| 4      | uint8    | gps_health    | 范围0~5，5表示GPS状态最好              |
| 5      | uint8    | armed         | 加锁状态 (0: 未加锁， 1: 加锁)         |
| 6      | uint8    | flight_state  | 飞行状态 (0: 停止，2: 在地上，3: 飞行) |
| 7      | uint8    | rc_connected  | 远程连接 (0: 未连接， 1: 已连接)       |
| 8      | uint8    | rtk_connected | RTK连接 (0: 未连接， 1: 已连接)        |
| 9      | uint8    | mission       | 任务类型 (指令反馈接口)                |
| 10     | uint8    | connected     | MAVROS连接 (0: 未连接， 1: 已连接)     |
| 11     | uint8    | mode_len      | 飞行模式长度                           |
| 12     | char []  | mode          | 飞行模式                               |



---

### 目标位置

MSG_ID: 2

发送频率: 发送方自定义

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称                         | 注释                    |
| ------ | -------- | ---------------------------- | ----------------------- |
| 0      | float32  | target_latitude              | 目标纬度 (度)           |
| 4      | float32  | target_longitude             | 目标维度 (度)           |
| 8      | float32  | target_altitude              | 目标海拔 (度)           |
| 12     | float32  | target_position_body_frame_x | 目标位置x (BODY坐标，m) |
| 16     | float32  | target_position_body_frame_y | 目标位置y (BODY坐标，m) |
| 20     | float32  | target_position_body_frame_z | 目标位置z (BODY坐标，m) |
| 24     | float32  | target_position_in_photo     | 目标在画面中的位置x     |
| 28     | float32  | target_position_in_photo     | 目标在画面中的位置y     |



---

### 指点飞行

MSG_ID: 101

发送频率: 发送方自定义

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称      | 注释                           |
| ------ | -------- | --------- | ------------------------------ |
| 0      | uint8    | mission   | 任务类型 (由两端定义）         |
| 1      | float32  | latitude  | 目标GPS纬度 (度)               |
| 5      | float32  | longitude | 目标GPS经度 (度)               |
| 9      | float32  | altitude  | 目标GPS海拔 (度)               |
| 13     | float32  | x         | 目标位置x (NED坐标，m)         |
| 17     | float32  | y         | 目标位置y (NED坐标，m)         |
| 21     | float32  | z         | 目标位置z (NED坐标，m)         |
| 25     | float32  | yaw       | 目标偏航角--yaw (NED坐标, rad) |



---

### 消息

MSG_ID: 255

| 偏移量 | 数据类型 | 名称 | 注释               |
| ------ | -------- | ---- | ------------------ |
| 0      | char[]   | data | 回传消息，不限长度 |









