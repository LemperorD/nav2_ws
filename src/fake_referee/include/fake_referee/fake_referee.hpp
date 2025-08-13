#ifndef FAKE_REFEREE_HPP_
#define FAKE_REFEREE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"

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

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数存储
    BuffConfig buff_cfg_;
    EventDataConfig eventdata_cfg_;
    GameRobotHpConfig game_robot_hp_cfg_;
    GameStatusConfig game_status_cfg_;
    GroundRobotPositionConfig ground_robot_position_cfg_;
    RfidStatusConfig rfid_status_cfg_;

    // 参数回调句柄
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace fake_referee

#endif // FAKE_REFEREE_HPP_