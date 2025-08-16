#ifndef FAKE_REFEREE_HPP_
#define FAKE_REFEREE_HPP_

#include <string>
#include <vector>
#include <tuple>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

#include "rmoss_interfaces/msg/robot_status.hpp"

namespace fake_referee
{
//------------------------------裁判协议相关的enum部分--------------------------------
typedef enum{
    UNOCCUPIED = 0,                // Not occupied or not activated
    OCCUPIED_FRIEND = 1,           // Occupied or activated by friendly side
    OCCUPIED_ENEMY = 2,            // Occupied or activated by enemy side
    OCCUPIED_BOTH = 3,             // Occupied or activated by both sides
} EventData;

typedef enum{
    NOT_START = 0,                 // 未开始比赛
    PREPARATION = 1,               // 准备阶段
    SELF_CHECKING = 2,             // 十五秒裁判系统自检阶段
    COUNT_DOWN = 3,                // 五秒倒计时
    RUNNING = 4,                   // 比赛中
    GAME_OVER = 5,                 // 比赛结算中
} GameStatus;

typedef enum{
    NOT_DETECTED = 0,              // RFID card not detected
    DETECTED = 1,                  // RFID card detected
} RfidStatus;

typedef enum{
    ARMOR_HIT = 0                     ,//装甲模块被弹丸攻击导致扣血
    SYSTEM_OFFLINE = 1                ,//裁判系统重要模块离线导致扣血
    OVER_SHOOT_SPEED = 2              ,//射击初速度超限导致扣血
    OVER_HEAT = 3                     ,//枪口热量超限导致扣血
    OVER_POWER = 4                    ,//底盘功率超限导致扣血
    ARMOR_COLLISION = 5               ,//装甲模块受到撞击导致扣血
}hp_deduction_reason;
//-------------------------详见裁判系统串口协议 V1.7.0及以上版本-------------------------

//--------------------------------用于保存参数的结构体---------------------------------
struct BuffConfig {
    uint8_t recovery_buff;           //机器人回血增益(百分比，值为 10 表示每秒恢复血量上限的 10%)
    uint8_t cooling_buff;            //机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff;            //机器人防御增益（百分比，值为 50 表示 50% 防御增益）
    uint8_t vulnerability_buff;      //机器人负防御增益（百分比，值为 30 表示 -30% 防御增益）
    uint16_t attack_buff;            //机器人攻击增益（百分比，值为 50 表示 50% 攻击增益）
    uint8_t remaining_energy;        //机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50% 时反馈，其余默认反馈 0x32。
};//0x0204

struct EventDataConfig {
    uint8_t non_overlapping_supply_zone;   //己方与兑换区不重叠的补给区的占领状态，1 为已占领
    uint8_t overlapping_supply_zone;       //己方与兑换区重叠的补给区的占领状态，1 为已占领
    uint8_t supply_zone;                   //己方补给区的占领状态，1 为已占领（仅 RMUL 适用）

    uint8_t small_energy;                  //己方小能量机关的激活状态，1 为已激活
    uint8_t big_energy;                    //己方大能量机关的激活状态，1 为已激活

    uint8_t central_highland;              //己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
    uint8_t trapezoidal_highland;          //己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领

    uint8_t center_gain_zone;              //中心增益点的占领情况，
                                        //0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领（仅 RMUL 适用）
};//0x0101

struct GameRobotHpConfig {
    uint16_t red_1_robot_hp       ;//红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
    uint16_t red_2_robot_hp       ;//红 2 工程机器人血量
    uint16_t red_3_robot_hp       ;//红 3 步兵机器人血量
    uint16_t red_4_robot_hp       ;//红 4 步兵机器人血量
    uint16_t red_7_robot_hp       ;//红 7 哨兵机器人血量
    uint16_t red_outpost_hp       ;//红方前哨站血量
    uint16_t red_base_hp          ;//红方基地血量
    uint16_t blue_1_robot_hp      ;//蓝 1 英雄机器人血
    uint16_t blue_2_robot_hp      ;//蓝 2 工程机器人血量
    uint16_t blue_3_robot_hp      ;//蓝 3 步兵机器人血量
    uint16_t blue_4_robot_hp      ;//蓝 4 步兵机器人血量
    uint16_t blue_7_robot_hp      ;//蓝 7 哨兵机器人血量
    uint16_t blue_outpost_hp      ;//蓝方前哨站血量
    uint16_t blue_base_hp         ;//蓝方基地血量
};//0x0003

struct GameStatusConfig {
    uint8_t game_progress;
    int stage_remain_time;
};//0x0001

struct GroundRobotPositionConfig {
    double hero_x, hero_y, hero_z;
    double engineer_x, engineer_y, engineer_z;
    double standard3_x, standard3_y, standard3_z;
    double standard4_x, standard4_y, standard4_z;
    // TODO：后续将每个机器人的位置优化为Point2D
};//0x020B

struct RfidStatusConfig {
    bool base_gain_point                                ;//己方基地增益点
    bool central_highland_gain_point                    ;//己方中央高地增益点
    bool enemy_central_highland_gain_point              ;//对方中央高地增益点
    bool friendly_trapezoidal_highland_gain_point       ;//己方梯形高地增益点
    bool enemy_trapezoidal_highland_gain_point          ;//对方梯形高地增益点
    bool friendly_fly_ramp_front_gain_point             ;//己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    bool friendly_fly_ramp_back_gain_point              ;//己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    bool enemy_fly_ramp_front_gain_point                ;//对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    bool enemy_fly_ramp_back_gain_point                 ;//对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    bool friendly_central_highland_lower_gain_point     ;//己方地形跨越增益点（中央高地下方）
    bool friendly_central_highland_upper_gain_point     ;//己方地形跨越增益点（中央高地上方）
    bool enemy_central_highland_lower_gain_point        ;//对方地形跨越增益点（中央高地下方）
    bool enemy_central_highland_upper_gain_point        ;//对方地形跨越增益点（中央高地上方）
    bool friendly_highway_lower_gain_point              ;//己方地形跨越增益点（公路下方）
    bool friendly_highway_upper_gain_point              ;//己方地形跨越增益点（公路上方）
    bool enemy_highway_lower_gain_point                 ;//对方地形跨越增益点（公路下方）
    bool enemy_highway_upper_gain_point                 ;//对方地形跨越增益点（公路上方）
    bool friendly_fortress_gain_point                   ;//己方堡垒增益点
    bool friendly_outpost_gain_point                    ;//己方前哨站增益点
    bool friendly_supply_zone_non_exchange              ;//己方与兑换区不重叠的补给区/RMUL 补给区
    bool friendly_supply_zone_exchange                  ;//己方与兑换区重叠的补给区
    bool friendly_big_resource_island                   ;//己方大资源岛增益点
    bool enemy_big_resource_island                      ;//对方大资源岛增益点
    bool center_gain_point                              ;//中心增益点（仅 RMUL 适用）
};//0x0209

struct PB_robot_status{
    //机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)
    uint8_t robot_id                        ;//本机器人 ID
    uint8_t robot_level                     ;//机器人等级
    uint16_t current_hp                       ;//机器人当前血量
    uint16_t maximum_hp                       ;//机器人血量上限
    uint16_t shooter_barrel_cooling_value     ;//机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit        ;//机器人枪口热量上限
    
    //实时底盘缓冲能量和射击热量数 (裁判系统串口协议 V1.7.0 0x0202)
    uint16_t shooter_17mm_1_barrel_heat       ;//第 1 个 17mm 发射机构的枪口热量
    
    //伤害状态数据 (裁判系统串口协议 V1.7.0 0x0206)
    //const for hp_deduction_reason
    // uint8_t ARMOR_HIT = 0                     //装甲模块被弹丸攻击导致扣血
    // uint8_t SYSTEM_OFFLINE = 1                //裁判系统重要模块离线导致扣血
    // uint8_t OVER_SHOOT_SPEED = 2              //射击初速度超限导致扣血
    // uint8_t OVER_HEAT = 3                     //枪口热量超限导致扣血
    // uint8_t OVER_POWER = 4                    //底盘功率超限导致扣血
    // uint8_t ARMOR_COLLISION = 5               //装甲模块受到撞击导致扣血
    
    uint8_t armor_id;                          //当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
                                               //数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t hp_deduction_reason;               //血量变化类型
    
    //允许发弹量 (裁判系统串口协议 V1.7.0 0x0208)
    uint16_t projectile_allowance_17mm;        //17mm 弹丸剩余发射次数

    //本机机器人位置数据 (裁判系统串口协议 V1.7.0 0x0203) //挪到下面方便字对齐
    std::array<double,3> translation;
    std::array<double,4> rotation;

    uint16_t remaining_gold_coin;              //剩余金币数量
    
    bool is_hp_deduced;                      //血量是否下降（上位机二次处理）
};
//----------------------------------------------------------------------------------

class FakeRefereeNode : public rclcpp::Node
{
public:
    explicit FakeRefereeNode(const rclcpp::NodeOptions & options);

private:
    void PublishBuff();
    //发出“referee/buff”消息，针对个体

    void PublishEventData();
    //发出“referee/eventdata”,针对己方全体

    void PublishGameRobotHP();
    //发出“/referee/all_robot_hp”,针对全体

    void PublishGameStatus();
    //发出“/referee/game_status”,针对全体

    void PublishGroundRobotPosition();
    //发出“referee/ground_robot_position”,针对哨兵

    void PublishRfidStatus();
    //发出“referee/rfid_status”,针对个体

    void PublishRobotStatus();
    //发出“referee/robot_status”,针对个体

    void SubRobotStatus(const rmoss_interfaces::msg::RobotStatus::SharedPtr msg);
    //订阅器回调函数，接收并准备重新发出机器人状态话题

    void SubAttackInfo(const std_msgs::msg::String::SharedPtr msg);
    //订阅器回调函数，接收attack_info字符串

    std::vector<std::string> split(const std::string &str, char delim);

    // 参数声明 + 更新
    void declareAndInitParams();
    void updateParams();

    // 发布器
    rclcpp::Publisher<pb_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr eventdata_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr game_robot_hp_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr ground_robot_position_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;

    //订阅器
    rclcpp::Subscription<rmoss_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr attack_info_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数存储
    BuffConfig buff_cfg_;
    EventDataConfig eventdata_cfg_;
    GameRobotHpConfig game_robot_hp_cfg_;
    GameStatusConfig game_status_cfg_;
    GroundRobotPositionConfig ground_robot_position_cfg_;
    RfidStatusConfig rfid_status_cfg_;
    PB_robot_status rst_pb;

    // 参数回调句柄
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace fake_referee

#endif // FAKE_REFEREE_HPP_