#ifndef FAKE_REFEREE_HPP_
#define FAKE_REFEREE_HPP_

#include <array>      // [MOD] 你用到了 std::array，必须显式 include
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
    UNOCCUPIED = 0,
    OCCUPIED_FRIEND = 1,
    OCCUPIED_ENEMY = 2,
    OCCUPIED_BOTH = 3,
} EventData;

typedef enum{
    NOT_START = 0,
    PREPARATION = 1,
    SELF_CHECKING = 2,
    COUNT_DOWN = 3,
    RUNNING = 4,
    GAME_OVER = 5,
} GameStatus;

typedef enum{
    NOT_DETECTED = 0,
    DETECTED = 1,
} RfidStatus;

typedef enum{
    ARMOR_HIT = 0,
    SYSTEM_OFFLINE = 1,
    OVER_SHOOT_SPEED = 2,
    OVER_HEAT = 3,
    OVER_POWER = 4,
    ARMOR_COLLISION = 5,
} hp_deduction_reason;

//--------------------------------用于保存参数的结构体---------------------------------
struct BuffConfig {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
};

struct EventDataConfig {
    uint8_t non_overlapping_supply_zone;
    uint8_t overlapping_supply_zone;
    uint8_t supply_zone;
    uint8_t small_energy;
    uint8_t big_energy;
    uint8_t central_highland;
    uint8_t trapezoidal_highland;
    uint8_t center_gain_zone;
};

struct GameRobotHpConfig {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
};

struct GameStatusConfig {
    uint8_t game_progress;
    int stage_remain_time;
};

struct GroundRobotPositionConfig {
    double hero_x, hero_y, hero_z;
    double engineer_x, engineer_y, engineer_z;
    double standard3_x, standard3_y, standard3_z;
    double standard4_x, standard4_y, standard4_z;
};

struct RfidStatusConfig {
    bool base_gain_point;
    bool central_highland_gain_point;
    bool enemy_central_highland_gain_point;
    bool friendly_trapezoidal_highland_gain_point;
    bool enemy_trapezoidal_highland_gain_point;
    bool friendly_fly_ramp_front_gain_point;
    bool friendly_fly_ramp_back_gain_point;
    bool enemy_fly_ramp_front_gain_point;
    bool enemy_fly_ramp_back_gain_point;
    bool friendly_central_highland_lower_gain_point;
    bool friendly_central_highland_upper_gain_point;
    bool enemy_central_highland_lower_gain_point;
    bool enemy_central_highland_upper_gain_point;
    bool friendly_highway_lower_gain_point;
    bool friendly_highway_upper_gain_point;
    bool enemy_highway_lower_gain_point;
    bool enemy_highway_upper_gain_point;
    bool friendly_fortress_gain_point;
    bool friendly_outpost_gain_point;
    bool friendly_supply_zone_non_exchange;
    bool friendly_supply_zone_exchange;
    bool friendly_big_resource_island;
    bool enemy_big_resource_island;
    bool center_gain_point;
};

struct PB_robot_status{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_17mm_1_barrel_heat;
    uint8_t armor_id;
    uint8_t hp_deduction_reason;
    uint16_t projectile_allowance_17mm;
    std::array<double,3> translation;
    std::array<double,4> rotation;
    uint16_t remaining_gold_coin;
    bool is_hp_deduced;
};

class FakeRefereeNode : public rclcpp::Node
{
public:
    explicit FakeRefereeNode(const rclcpp::NodeOptions & options);

private:
    void PublishBuff();
    void PublishEventData();
    void PublishGameRobotHP();
    void PublishGameStatus();
    void PublishGroundRobotPosition();
    void PublishRfidStatus();
    void PublishRobotStatus();

    void SubRobotStatus(const rmoss_interfaces::msg::RobotStatus::SharedPtr msg);
    void SubAttackInfo(const std_msgs::msg::String::SharedPtr msg);

    std::vector<std::string> split(const std::string &str, char delim);

    void declareAndInitParams();
    void updateParams();

    rclcpp::Publisher<pb_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr eventdata_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr game_robot_hp_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr ground_robot_position_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;

    rclcpp::Subscription<rmoss_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr attack_info_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    BuffConfig buff_cfg_;
    EventDataConfig eventdata_cfg_;
    GameRobotHpConfig game_robot_hp_cfg_;
    GameStatusConfig game_status_cfg_;
    GroundRobotPositionConfig ground_robot_position_cfg_;
    RfidStatusConfig rfid_status_cfg_;
    PB_robot_status rst_pb;

    // [MOD] 新增：控制“外部裁判系统数据”和“参数注入血量”的优先级
    bool use_external_robot_status_{true};         // 是否使用 /referee_system/.../robot_status 更新（位置等）
    bool lock_hp_to_param_{true};                  // 若 true：发布的 current_hp 强制来自参数 current_hp
    bool clear_damage_flags_after_publish_{true};  // 若 true：发布一次扣血标志后清零

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace fake_referee

#endif // FAKE_REFEREE_HPP_
