# fake_referee开发者日志

## 2025.8.14

### 1. 不同的RobotStatus

#### 1. rmoss_interfaces::msg::RobotStatus

```
uint8 id #机器人id
uint8 level #机器人等级
string name #机器人名字
int32 remain_hp #剩余血量
int32 max_hp #血上限
int32 total_projectiles #总弹药量
int32 used_projectiles #已经用的子弹数
# only for referee system
int32 hit_projectiles #命中数
geometry_msgs/Transform gt_tf #相对地图原点的位置
```

eg:
```
id: 0
level: 0
name: ''
remain_hp: 500
max_hp: 500
total_projectiles: 100
used_projectiles: 0
hit_projectiles: 0
gt_tf:
  translation:
    x: 3.9497506495212082
    y: 1.4049815200741473
    z: 0.2758016964963833
  rotation:
    x: -1.7018944394110594e-06
    y: -1.9689506987077595e-07
    z: 0.6784613578147001
    w: 0.7346360908295332
```

#### 2. pb_rm_interfaces::msg::RobotStatus
```
# 机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)
uint8 robot_id                          # 本机器人 ID
uint8 robot_level                       # 机器人等级
uint16 current_hp                       # 机器人当前血量
uint16 maximum_hp                       # 机器人血量上限
uint16 shooter_barrel_cooling_value     # 机器人枪口热量每秒冷却值
uint16 shooter_barrel_heat_limit        # 机器人枪口热量上限

# 实时底盘缓冲能量和射击热量数 (裁判系统串口协议 V1.7.0 0x0202)
uint16 shooter_17mm_1_barrel_heat       # 第 1 个 17mm 发射机构的枪口热量

# 本机机器人位置数据 (裁判系统串口协议 V1.7.0 0x0203)
geometry_msgs/Pose robot_pos              # 本机器人姿态

# 伤害状态数据 (裁判系统串口协议 V1.7.0 0x0206)
# const for hp_deduction_reason
uint8 ARMOR_HIT = 0                     # 装甲模块被弹丸攻击导致扣血
uint8 SYSTEM_OFFLINE = 1                # 裁判系统重要模块离线导致扣血
uint8 OVER_SHOOT_SPEED = 2              # 射击初速度超限导致扣血
uint8 OVER_HEAT = 3                     # 枪口热量超限导致扣血
uint8 OVER_POWER = 4                    # 底盘功率超限导致扣血
uint8 ARMOR_COLLISION = 5               # 装甲模块受到撞击导致扣血

uint8 armor_id                          # 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
                                        # 数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
uint8 hp_deduction_reason               # 血量变化类型

# 允许发弹量 (裁判系统串口协议 V1.7.0 0x0208)
uint16 projectile_allowance_17mm        # 17mm 弹丸剩余发射次数
uint16 remaining_gold_coin              # 剩余金币数量

bool is_hp_deduced                      # 血量是否下降（上位机二次处理）
```

```
ld@D:~/nav2_ws$ ros2 interface show geometry_msgs/msg/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
        float64 x
        float64 y
        float64 z
Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
```

### 2. shoot_info
data: blue_standard_robot1/small_shooter,20.000000

