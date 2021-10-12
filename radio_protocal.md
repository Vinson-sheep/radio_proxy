## 数传电台传输协议

*Vinson Sheep*

*20211009*

### 1. 数据结构

| HEAD  | MSG_ID     | TARGET_ID  | LOCAL_ID | PAYLOAD | END       |
| ----- | ---------- | ---------- | -------- | ------- | --------- |
| 0X5A  | 命令辨识码 | 目标飞机ID | 本机ID   | 数据段  | 0X0D 0X0A |
| 1字节 | 1个字节    | 1字节      | 1字节    | LEN-6   | 2字节     |

> TARGET_ID/ LOCAL_ID ： 地面站默认为254， 全体飞机默认为255；

### 2. 消息内容

----

#### 飞行数据

MSG_ID: 1

发送频率: 10HZ

PAYLOAD结构: 

| 偏移量 | 数据类型 | 名称                 | 注释                                      |
| ------ | -------- | -------------------- | ----------------------------------------- |
| 0      | float32  | latitude             | GPS纬度 (度)                              |
| 4      | float32  | longitude            | GPS经度 (度)                              |
| 8      | float32  | altitude             | GPS海拔 (度)                              |
| 12     | float32  | x                    | 局部位置--x  (NED坐标，m)                 |
| 16     | float32  | y                    | 局部位置--y  (NED坐标，m)                 |
| 20     | float32  | z                    | 局部位置--z  (NED坐标，m)                 |
| 24     | float32  | vx                   | 局部速度--vx (NED坐标，m/s)               |
| 28     | float32  | vy                   | 局部速度--vy (NED坐标，m/s)               |
| 32     | float32  | vz                   | 局部速度--vz (NED坐标，m/s)               |
| 36     | float32  | ax                   | 局部加速度--ax (NED坐标，m/s<sup>2</sup>) |
| 40     | float32  | ay                   | 局部加速度--ay (NED坐标，m/s<sup>2</sup>) |
| 44     | float32  | az                   | 局部加速度--ax (NED坐标，m/s<sup>2</sup>) |
| 48     | float32  | pitch                | 俯仰角                                    |
| 52     | float32  | roll                 | 横滚角                                    |
| 56     | float32  | yaw                  | 偏航角                                    |
| 60     | float32  | yaw_rate             | 偏航速度 (rad/s)                          |
| 64     | float32  | height_above_takeoff | 起飞高度（解锁后可用，m）                 |

> 务必确保gps_health在4以上

---

### 状态数据

MSG_ID: 2

发送频率: 1HZ

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称          | 注释                                 |
| ------ | -------- | ------------- | ------------------------------------ |
| 0      | float32  | battery_v     | 电池电压 (V)                         |
| 4      | uint8    | display_mode  | see dji_sdk.h                        |
| 5      | uint8    | flight_status | see dji_sdk.h                        |
| 6      | uint8    | gps_health    | 范围0~5，5表示GPS状态最好            |
| 7      | uint8    | arm_state     | 加锁状态 (0: 未加锁， 1: 加锁)       |
| 8      | uint8    | land_state    | 离地状态 (0: 着陆, 1: 触地, 2: 空中) |



---

### 指点飞行(GPS)

MSG_ID: 101

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称      | 注释                           |
| ------ | -------- | --------- | ------------------------------ |
| 0      | float32  | latitude  | 目标GPS纬度 (度)               |
| 4      | float32  | longitude | 目标GPS经度 (度)               |
| 8      | float32  | altitude  | 目标GPS海拔 (度)               |
| 12     | float32  | yaw       | 目标偏航角--yaw (NED坐标, rad) |



---

### 指点飞行(LOCAL)

MSG_ID: 102

PAYLOAD结构:

| 偏移量 | 数据类型 | 名称 | 注释                           |
| ------ | -------- | ---- | ------------------------------ |
| 0      | float32  | x    | 目标局部位置--x  (NED坐标，m)  |
| 4      | float32  | y    | 目标局部位置--y  (NED坐标，m)  |
| 8      | float32  | z    | 目标局部位置--z  (NED坐标，m)  |
| 12     | float32  | yaw  | 目标偏航角--yaw (NED坐标, rad) |



---

### 自动起飞

MSG_ID: 103



---

### 自动降落

MSG_ID: 104



---

### 解锁

MSG_ID: 105



---

### 加锁

MSG_ID: 106



---

### 悬停

MSG_ID: 107



---

### 消息

MSG_ID: 255

| 偏移量 | 数据类型 | 名称 | 注释                   |
| ------ | -------- | ---- | ---------------------- |
| 0      | char[31] | data | 回传消息，最长31个字节 |









