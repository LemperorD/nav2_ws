#include "fake_referee/fake_referee.hpp"

namespace fake_referee{
FakeRefereeNode::FakeRefereeNode(const rclcpp::NodeOptions & options)
: Node("fake_referee_node", options)
{
    RCLCPP_INFO(get_logger(), "Start Fake Referee!");

    // 创建发布器
    buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 20);
    eventdata_pub_ = this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/eventdata", 20);
    game_robot_hp_pub_ = this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("/referee/all_robot_hp", 20);
    game_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::GameStatus>("/referee/game_status", 20);
    ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>("referee/ground_robot_position", 20);
    rfid_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 20);
    robot_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 20);

    //创建订阅器
    robot_status_sub_ = this->create_subscription<rmoss_interfaces::msg::RobotStatus>
    ("/referee_system/red_standard_robot1/robot_status", 10, std::bind(&FakeRefereeNode::SubRobotStatus, this, std::placeholders::_1));

    attack_info_sub_ = this->create_subscription<std_msgs::msg::String>
    ("/referee_system/attack_info", 10, std::bind(&FakeRefereeNode::SubAttackInfo, this, std::placeholders::_1));

    // 声明参数并初始化
    declareAndInitParams();

    // 参数动态更新回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            (void)params;
            updateParams();
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }
    );

    // 定时器：周期发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
            PublishBuff();
            PublishEventData();
            PublishGameRobotHP();
            PublishGameStatus();
            PublishGroundRobotPosition();
            PublishRfidStatus();
            PublishRobotStatus();
        }
    );
}

std::vector<std::string> split(const std::string &str, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}

using AttackInfo = std::tuple<std::string, std::string, std::string, std::string>;

std::optional<AttackInfo> parse_attack_info(const std::string &attack_str) {
    auto info = split(attack_str, ',');
    if (info.size() < 2) {
        return std::nullopt;
    }

    auto shooter = split(info[0], '/');
    if (shooter.size() != 2) {
        return std::nullopt;
    }
    std::string shooter_model_name = shooter[0];
    std::string shooter_name = shooter[1];

    auto target = split(info[1], '/');
    if (target.size() != 4) {
        return std::nullopt;
    }
    std::string target_model_name = target[1];
    std::string target_link_name = target[2];
    std::string target_collision_name = target[3];

    if (target_collision_name != "target_collision") {
        return std::nullopt;
    }

    return std::make_tuple(shooter_model_name, shooter_name, target_model_name, target_link_name);
}

void FakeRefereeNode::PublishBuff()
{
    pb_rm_interfaces::msg::Buff msg;
    msg.recovery_buff = buff_cfg_.recovery_buff;
    msg.cooling_buff = buff_cfg_.cooling_buff;
    msg.defence_buff = buff_cfg_.defence_buff;
    msg.vulnerability_buff = buff_cfg_.vulnerability_buff;
    msg.attack_buff = buff_cfg_.attack_buff;
    msg.remaining_energy = buff_cfg_.remaining_energy;
    buff_pub_->publish(msg);
}

void FakeRefereeNode::PublishEventData()
{
    pb_rm_interfaces::msg::EventData msg;
    msg.non_overlapping_supply_zone = eventdata_cfg_.non_overlapping_supply_zone;
    msg.overlapping_supply_zone = eventdata_cfg_.overlapping_supply_zone;
    msg.supply_zone = eventdata_cfg_.supply_zone;
    msg.small_energy = eventdata_cfg_.small_energy;
    msg.big_energy = eventdata_cfg_.big_energy;
    msg.central_highland = eventdata_cfg_.central_highland;
    msg.trapezoidal_highland = eventdata_cfg_.trapezoidal_highland;
    msg.center_gain_zone = eventdata_cfg_.center_gain_zone;
    eventdata_pub_->publish(msg);
}

void FakeRefereeNode::PublishGameRobotHP()
{
    pb_rm_interfaces::msg::GameRobotHP msg;
    msg.red_1_robot_hp = game_robot_hp_cfg_.red_1_robot_hp;
    msg.red_2_robot_hp = game_robot_hp_cfg_.red_2_robot_hp;
    msg.red_3_robot_hp = game_robot_hp_cfg_.red_3_robot_hp;
    msg.red_4_robot_hp = game_robot_hp_cfg_.red_4_robot_hp;
    msg.red_7_robot_hp = game_robot_hp_cfg_.red_7_robot_hp;
    msg.red_outpost_hp = game_robot_hp_cfg_.red_outpost_hp;
    msg.red_base_hp = game_robot_hp_cfg_.red_base_hp;
    msg.blue_1_robot_hp = game_robot_hp_cfg_.blue_1_robot_hp;
    msg.blue_2_robot_hp = game_robot_hp_cfg_.blue_2_robot_hp;
    msg.blue_3_robot_hp = game_robot_hp_cfg_.blue_3_robot_hp;
    msg.blue_4_robot_hp = game_robot_hp_cfg_.blue_4_robot_hp;
    msg.blue_7_robot_hp = game_robot_hp_cfg_.blue_7_robot_hp;
    msg.blue_outpost_hp = game_robot_hp_cfg_.blue_outpost_hp;
    msg.blue_base_hp = game_robot_hp_cfg_.blue_base_hp;
    game_robot_hp_pub_->publish(msg);
}

void FakeRefereeNode::PublishGameStatus()
{
    pb_rm_interfaces::msg::GameStatus msg;
    msg.game_progress = game_status_cfg_.game_progress;
    msg.stage_remain_time = game_status_cfg_.stage_remain_time;
    game_status_pub_->publish(msg);
}

void FakeRefereeNode::PublishGroundRobotPosition()
{
    pb_rm_interfaces::msg::GroundRobotPosition msg;
    // 设置英雄机器人位置
    msg.hero_position.x = ground_robot_position_cfg_.hero_x;
    msg.hero_position.y = ground_robot_position_cfg_.hero_y;
    msg.hero_position.z = ground_robot_position_cfg_.hero_z;
    
    // 设置工程机器人位置
    msg.engineer_position.x = ground_robot_position_cfg_.engineer_x;
    msg.engineer_position.y = ground_robot_position_cfg_.engineer_y;
    msg.engineer_position.z = ground_robot_position_cfg_.engineer_z;
    
    // 设置3号步兵机器人位置
    msg.standard_3_position.x = ground_robot_position_cfg_.standard3_x;
    msg.standard_3_position.y = ground_robot_position_cfg_.standard3_y;
    msg.standard_3_position.z = ground_robot_position_cfg_.standard3_z;
    
    // 设置4号步兵机器人位置
    msg.standard_4_position.x = ground_robot_position_cfg_.standard4_x;
    msg.standard_4_position.y = ground_robot_position_cfg_.standard4_y;
    msg.standard_4_position.z = ground_robot_position_cfg_.standard4_z;
    ground_robot_position_pub_->publish(msg);
}

void FakeRefereeNode::PublishRfidStatus()
{
    pb_rm_interfaces::msg::RfidStatus msg;
    msg.base_gain_point = rfid_status_cfg_.base_gain_point;
    msg.central_highland_gain_point = rfid_status_cfg_.central_highland_gain_point;
    msg.enemy_central_highland_gain_point = rfid_status_cfg_.enemy_central_highland_gain_point;
    msg.friendly_trapezoidal_highland_gain_point = rfid_status_cfg_.friendly_trapezoidal_highland_gain_point;
    msg.enemy_trapezoidal_highland_gain_point = rfid_status_cfg_.enemy_trapezoidal_highland_gain_point;
    msg.friendly_fly_ramp_front_gain_point = rfid_status_cfg_.friendly_fly_ramp_front_gain_point;
    msg.friendly_fly_ramp_back_gain_point = rfid_status_cfg_.friendly_fly_ramp_back_gain_point;
    msg.enemy_fly_ramp_front_gain_point = rfid_status_cfg_.enemy_fly_ramp_front_gain_point;
    msg.enemy_fly_ramp_back_gain_point = rfid_status_cfg_.enemy_fly_ramp_back_gain_point;
    msg.friendly_central_highland_lower_gain_point = rfid_status_cfg_.friendly_central_highland_lower_gain_point;
    msg.friendly_central_highland_upper_gain_point = rfid_status_cfg_.friendly_central_highland_upper_gain_point;
    msg.enemy_central_highland_lower_gain_point = rfid_status_cfg_.enemy_central_highland_lower_gain_point;
    msg.enemy_central_highland_upper_gain_point = rfid_status_cfg_.enemy_central_highland_upper_gain_point;
    msg.friendly_highway_lower_gain_point = rfid_status_cfg_.friendly_highway_lower_gain_point;
    msg.friendly_highway_upper_gain_point = rfid_status_cfg_.friendly_highway_upper_gain_point;
    msg.enemy_highway_lower_gain_point = rfid_status_cfg_.enemy_highway_lower_gain_point;
    msg.enemy_highway_upper_gain_point = rfid_status_cfg_.enemy_highway_upper_gain_point;
    msg.friendly_fortress_gain_point = rfid_status_cfg_.friendly_fortress_gain_point;
    msg.friendly_outpost_gain_point = rfid_status_cfg_.friendly_outpost_gain_point;
    msg.friendly_supply_zone_non_exchange = rfid_status_cfg_.friendly_supply_zone_non_exchange;
    msg.friendly_supply_zone_exchange = rfid_status_cfg_.friendly_supply_zone_exchange;
    msg.friendly_big_resource_island = rfid_status_cfg_.friendly_big_resource_island;
    msg.enemy_big_resource_island = rfid_status_cfg_.enemy_big_resource_island;
    // msg.center_gain_point = rfid_status_cfg_.center_gain_point;
    if (rst_pb.translation[0]<1 && rst_pb.translation[0]>-1 &&
        rst_pb.translation[1]<1 && rst_pb.translation[1]>-1){
        msg.center_gain_point = DETECTED;
    }
    else{
        msg.center_gain_point = NOT_DETECTED;
    }
    rfid_status_pub_->publish(msg);
}

void FakeRefereeNode::declareAndInitParams()
{
    // BuffConfig
    this->declare_parameter("buff.recovery_buff", 0);
    this->declare_parameter("buff.cooling_buff", 0);
    this->declare_parameter("buff.defence_buff", 0);
    this->declare_parameter("buff.vulnerability_buff", 0);
    this->declare_parameter("buff.attack_buff", 0);
    this->declare_parameter("buff.remaining_energy", 0);

    // EventDataConfig
    this->declare_parameter("eventdata.non_overlapping_supply_zone", 0);
    this->declare_parameter("eventdata.overlapping_supply_zone", 0);
    this->declare_parameter("eventdata.supply_zone", 0);
    this->declare_parameter("eventdata.small_energy", 0);
    this->declare_parameter("eventdata.big_energy", 0);
    this->declare_parameter("eventdata.central_highland", 0);
    this->declare_parameter("eventdata.trapezoidal_highland", 0);
    this->declare_parameter("eventdata.center_gain_zone", 0);

    // GameRobotHpConfig
    this->declare_parameter("game_robot_hp.red_1_robot_hp", 0);
    this->declare_parameter("game_robot_hp.red_2_robot_hp", 0);
    this->declare_parameter("game_robot_hp.red_3_robot_hp", 0);
    this->declare_parameter("game_robot_hp.red_4_robot_hp", 0);
    this->declare_parameter("game_robot_hp.red_7_robot_hp", 0);
    this->declare_parameter("game_robot_hp.red_outpost_hp", 0);
    this->declare_parameter("game_robot_hp.red_base_hp", 0);
    this->declare_parameter("game_robot_hp.blue_1_robot_hp", 0);
    this->declare_parameter("game_robot_hp.blue_2_robot_hp", 0);
    this->declare_parameter("game_robot_hp.blue_3_robot_hp", 0);
    this->declare_parameter("game_robot_hp.blue_4_robot_hp", 0);
    this->declare_parameter("game_robot_hp.blue_7_robot_hp", 0);
    this->declare_parameter("game_robot_hp.blue_outpost_hp", 0);
    this->declare_parameter("game_robot_hp.blue_base_hp", 0);

    // GameStatusConfig
    this->declare_parameter("game_status.game_progress", 4);
    this->declare_parameter("game_status.stage_remain_time", 300);

    // GroundRobotPositionConfig
    this->declare_parameter("ground_robot_position.hero_x", 0.0);
    this->declare_parameter("ground_robot_position.hero_y", 0.0);
    this->declare_parameter("ground_robot_position.hero_z", 0.0);
    this->declare_parameter("ground_robot_position.engineer_x", 0.0);
    this->declare_parameter("ground_robot_position.engineer_y", 0.0);
    this->declare_parameter("ground_robot_position.engineer_z", 0.0);
    this->declare_parameter("ground_robot_position.standard3_x", 0.0);
    this->declare_parameter("ground_robot_position.standard3_y", 0.0);
    this->declare_parameter("ground_robot_position.standard3_z", 0.0);
    this->declare_parameter("ground_robot_position.standard4_x", 0.0);
    this->declare_parameter("ground_robot_position.standard4_y", 0.0);
    this->declare_parameter("ground_robot_position.standard4_z", 0.0);

    // RfidStatusConfig
    this->declare_parameter("rfid_status.base_gain_point", false);
    this->declare_parameter("rfid_status.central_highland_gain_point", false);
    this->declare_parameter("rfid_status.enemy_central_highland_gain_point", false);
    this->declare_parameter("rfid_status.friendly_trapezoidal_highland_gain_point", false);
    this->declare_parameter("rfid_status.enemy_trapezoidal_highland_gain_point", false);
    this->declare_parameter("rfid_status.friendly_fly_ramp_front_gain_point", false);
    this->declare_parameter("rfid_status.friendly_fly_ramp_back_gain_point", false);
    this->declare_parameter("rfid_status.enemy_fly_ramp_front_gain_point", false);
    this->declare_parameter("rfid_status.enemy_fly_ramp_back_gain_point", false);
    this->declare_parameter("rfid_status.friendly_central_highland_lower_gain_point", false);
    this->declare_parameter("rfid_status.friendly_central_highland_upper_gain_point", false);
    this->declare_parameter("rfid_status.enemy_central_highland_lower_gain_point", false);
    this->declare_parameter("rfid_status.enemy_central_highland_upper_gain_point", false);
    this->declare_parameter("rfid_status.friendly_highway_lower_gain_point", false);
    this->declare_parameter("rfid_status.friendly_highway_upper_gain_point", false);
    this->declare_parameter("rfid_status.enemy_highway_lower_gain_point", false);
    this->declare_parameter("rfid_status.enemy_highway_upper_gain_point", false);
    this->declare_parameter("rfid_status.friendly_fortress_gain_point", false);
    this->declare_parameter("rfid_status.friendly_outpost_gain_point", false);
    this->declare_parameter("rfid_status.friendly_supply_zone_non_exchange", false);
    this->declare_parameter("rfid_status.friendly_supply_zone_exchange", false);
    this->declare_parameter("rfid_status.friendly_big_resource_island", false);
    this->declare_parameter("rfid_status.enemy_big_resource_island", false);
    this->declare_parameter("rfid_status.center_gain_point", false);

    //rst_pb
    this->declare_parameter<int>("robot_id", 1);
    this->declare_parameter<int>("robot_level", 1);
    this->declare_parameter<int>("current_hp", 500);
    this->declare_parameter<int>("maximum_hp", 500);
    this->declare_parameter<int>("shooter_barrel_cooling_value", 50);
    this->declare_parameter<int>("shooter_barrel_heat_limit", 240);
    this->declare_parameter<int>("shooter_17mm_1_barrel_heat", 0);
    this->declare_parameter<int>("armor_id", 0);
    this->declare_parameter<int>("hp_deduction_reason", 0);
    this->declare_parameter<int>("projectile_allowance_17mm", 100);
    this->declare_parameter<double>("translation_x", 0.0);
    this->declare_parameter<double>("translation_y", 0.0);
    this->declare_parameter<double>("translation_z", 0.0);
    this->declare_parameter<double>("rotation_x", 0.0);
    this->declare_parameter<double>("rotation_y", 0.0);
    this->declare_parameter<double>("rotation_z", 0.0);
    this->declare_parameter<double>("rotation_w", 1.0);
    this->declare_parameter<int>("remaining_gold_coin", 0);
    this->declare_parameter<bool>("is_hp_deduced", false);

    updateParams();
}

void FakeRefereeNode::updateParams()
{
    // BuffConfig
    buff_cfg_.recovery_buff = this->get_parameter("buff.recovery_buff").as_int();
    buff_cfg_.cooling_buff = this->get_parameter("buff.cooling_buff").as_int();
    buff_cfg_.defence_buff = this->get_parameter("buff.defence_buff").as_int();
    buff_cfg_.vulnerability_buff = this->get_parameter("buff.vulnerability_buff").as_int();
    buff_cfg_.attack_buff = this->get_parameter("buff.attack_buff").as_int();
    buff_cfg_.remaining_energy = this->get_parameter("buff.remaining_energy").as_int();

    // EventDataConfig
    eventdata_cfg_.non_overlapping_supply_zone = this->get_parameter("eventdata.non_overlapping_supply_zone").as_int();
    eventdata_cfg_.overlapping_supply_zone = this->get_parameter("eventdata.overlapping_supply_zone").as_int();
    eventdata_cfg_.supply_zone = this->get_parameter("eventdata.supply_zone").as_int();
    eventdata_cfg_.small_energy = this->get_parameter("eventdata.small_energy").as_int();
    eventdata_cfg_.big_energy = this->get_parameter("eventdata.big_energy").as_int();
    eventdata_cfg_.central_highland = this->get_parameter("eventdata.central_highland").as_int();
    eventdata_cfg_.trapezoidal_highland = this->get_parameter("eventdata.trapezoidal_highland").as_int();
    eventdata_cfg_.center_gain_zone = this->get_parameter("eventdata.center_gain_zone").as_int();

    // GameRobotHpConfig
    game_robot_hp_cfg_.red_1_robot_hp = this->get_parameter("game_robot_hp.red_1_robot_hp").as_int();
    game_robot_hp_cfg_.red_2_robot_hp = this->get_parameter("game_robot_hp.red_2_robot_hp").as_int();
    game_robot_hp_cfg_.red_3_robot_hp = this->get_parameter("game_robot_hp.red_3_robot_hp").as_int();
    game_robot_hp_cfg_.red_4_robot_hp = this->get_parameter("game_robot_hp.red_4_robot_hp").as_int();
    game_robot_hp_cfg_.red_7_robot_hp = this->get_parameter("game_robot_hp.red_7_robot_hp").as_int();
    game_robot_hp_cfg_.red_outpost_hp = this->get_parameter("game_robot_hp.red_outpost_hp").as_int();
    game_robot_hp_cfg_.red_base_hp = this->get_parameter("game_robot_hp.red_base_hp").as_int();
    game_robot_hp_cfg_.blue_1_robot_hp = this->get_parameter("game_robot_hp.blue_1_robot_hp").as_int();
    game_robot_hp_cfg_.blue_2_robot_hp = this->get_parameter("game_robot_hp.blue_2_robot_hp").as_int();
    game_robot_hp_cfg_.blue_3_robot_hp = this->get_parameter("game_robot_hp.blue_3_robot_hp").as_int();
    game_robot_hp_cfg_.blue_4_robot_hp = this->get_parameter("game_robot_hp.blue_4_robot_hp").as_int();
    game_robot_hp_cfg_.blue_7_robot_hp = this->get_parameter("game_robot_hp.blue_7_robot_hp").as_int();
    game_robot_hp_cfg_.blue_outpost_hp = this->get_parameter("game_robot_hp.blue_outpost_hp").as_int();
    game_robot_hp_cfg_.blue_base_hp = this->get_parameter("game_robot_hp.blue_base_hp").as_int();

    // GameStatusConfig
    game_status_cfg_.game_progress = this->get_parameter("game_status.game_progress").as_int();
    game_status_cfg_.stage_remain_time = this->get_parameter("game_status.stage_remain_time").as_int();

    // GroundRobotPositionConfig
    ground_robot_position_cfg_.hero_x = this->get_parameter("ground_robot_position.hero_x").as_double();
    ground_robot_position_cfg_.hero_y = this->get_parameter("ground_robot_position.hero_y").as_double();
    ground_robot_position_cfg_.hero_z = this->get_parameter("ground_robot_position.hero_z").as_double();
    ground_robot_position_cfg_.engineer_x = this->get_parameter("ground_robot_position.engineer_x").as_double();
    ground_robot_position_cfg_.engineer_y = this->get_parameter("ground_robot_position.engineer_y").as_double();
    ground_robot_position_cfg_.engineer_z = this->get_parameter("ground_robot_position.engineer_z").as_double();
    ground_robot_position_cfg_.standard3_x = this->get_parameter("ground_robot_position.standard3_x").as_double();
    ground_robot_position_cfg_.standard3_y = this->get_parameter("ground_robot_position.standard3_y").as_double();
    ground_robot_position_cfg_.standard3_z = this->get_parameter("ground_robot_position.standard3_z").as_double();
    ground_robot_position_cfg_.standard4_x = this->get_parameter("ground_robot_position.standard4_x").as_double();
    ground_robot_position_cfg_.standard4_y = this->get_parameter("ground_robot_position.standard4_y").as_double();
    ground_robot_position_cfg_.standard4_z = this->get_parameter("ground_robot_position.standard4_z").as_double();

    // RfidStatusConfig
    rfid_status_cfg_.base_gain_point = this->get_parameter("rfid_status.base_gain_point").as_bool();
    rfid_status_cfg_.central_highland_gain_point = this->get_parameter("rfid_status.central_highland_gain_point").as_bool();
    rfid_status_cfg_.enemy_central_highland_gain_point = this->get_parameter("rfid_status.enemy_central_highland_gain_point").as_bool();
    rfid_status_cfg_.friendly_trapezoidal_highland_gain_point = this->get_parameter("rfid_status.friendly_trapezoidal_highland_gain_point").as_bool();
    rfid_status_cfg_.enemy_trapezoidal_highland_gain_point = this->get_parameter("rfid_status.enemy_trapezoidal_highland_gain_point").as_bool();
    rfid_status_cfg_.friendly_fly_ramp_front_gain_point = this->get_parameter("rfid_status.friendly_fly_ramp_front_gain_point").as_bool();
    rfid_status_cfg_.friendly_fly_ramp_back_gain_point = this->get_parameter("rfid_status.friendly_fly_ramp_back_gain_point").as_bool();
    rfid_status_cfg_.enemy_fly_ramp_front_gain_point = this->get_parameter("rfid_status.enemy_fly_ramp_front_gain_point").as_bool();
    rfid_status_cfg_.enemy_fly_ramp_back_gain_point = this->get_parameter("rfid_status.enemy_fly_ramp_back_gain_point").as_bool();
    rfid_status_cfg_.friendly_central_highland_lower_gain_point = this->get_parameter("rfid_status.friendly_central_highland_lower_gain_point").as_bool();
    rfid_status_cfg_.friendly_central_highland_upper_gain_point = this->get_parameter("rfid_status.friendly_central_highland_upper_gain_point").as_bool();
    rfid_status_cfg_.enemy_central_highland_lower_gain_point = this->get_parameter("rfid_status.enemy_central_highland_lower_gain_point").as_bool();
    rfid_status_cfg_.enemy_central_highland_upper_gain_point = this->get_parameter("rfid_status.enemy_central_highland_upper_gain_point").as_bool();
    rfid_status_cfg_.friendly_highway_lower_gain_point = this->get_parameter("rfid_status.friendly_highway_lower_gain_point").as_bool();
    rfid_status_cfg_.friendly_highway_upper_gain_point = this->get_parameter("rfid_status.friendly_highway_upper_gain_point").as_bool();
    rfid_status_cfg_.enemy_highway_lower_gain_point = this->get_parameter("rfid_status.enemy_highway_lower_gain_point").as_bool();
    rfid_status_cfg_.enemy_highway_upper_gain_point = this->get_parameter("rfid_status.enemy_highway_upper_gain_point").as_bool();
    rfid_status_cfg_.friendly_fortress_gain_point = this->get_parameter("rfid_status.friendly_fortress_gain_point").as_bool();
    rfid_status_cfg_.friendly_outpost_gain_point = this->get_parameter("rfid_status.friendly_outpost_gain_point").as_bool();
    rfid_status_cfg_.friendly_supply_zone_non_exchange = this->get_parameter("rfid_status.friendly_supply_zone_non_exchange").as_bool();
    rfid_status_cfg_.friendly_supply_zone_exchange = this->get_parameter("rfid_status.friendly_supply_zone_exchange").as_bool();
    rfid_status_cfg_.friendly_big_resource_island = this->get_parameter("rfid_status.friendly_big_resource_island").as_bool();
    rfid_status_cfg_.enemy_big_resource_island = this->get_parameter("rfid_status.enemy_big_resource_island").as_bool();
    rfid_status_cfg_.center_gain_point = this->get_parameter("rfid_status.center_gain_point").as_bool();

    //rst_pb
    rst_pb.robot_id = this->get_parameter("robot_id").as_int();
    rst_pb.robot_level = this->get_parameter("robot_level").as_int();
    rst_pb.current_hp = this->get_parameter("current_hp").as_int();
    rst_pb.maximum_hp = this->get_parameter("maximum_hp").as_int();
    rst_pb.shooter_barrel_cooling_value = this->get_parameter("shooter_barrel_cooling_value").as_int();
    rst_pb.shooter_barrel_heat_limit = this->get_parameter("shooter_barrel_heat_limit").as_int();
    rst_pb.shooter_17mm_1_barrel_heat = this->get_parameter("shooter_17mm_1_barrel_heat").as_int();
    rst_pb.armor_id = this->get_parameter("armor_id").as_int();
    rst_pb.hp_deduction_reason = this->get_parameter("hp_deduction_reason").as_int();
    rst_pb.projectile_allowance_17mm = this->get_parameter("projectile_allowance_17mm").as_int();

    rst_pb.translation[0] = this->get_parameter("translation_x").as_double();
    rst_pb.translation[1] = this->get_parameter("translation_y").as_double();
    rst_pb.translation[2] = this->get_parameter("translation_z").as_double();

    rst_pb.rotation[0] = this->get_parameter("rotation_x").as_double();
    rst_pb.rotation[1] = this->get_parameter("rotation_y").as_double();
    rst_pb.rotation[2] = this->get_parameter("rotation_z").as_double();
    rst_pb.rotation[3] = this->get_parameter("rotation_w").as_double();

    rst_pb.remaining_gold_coin = this->get_parameter("remaining_gold_coin").as_int();
    rst_pb.is_hp_deduced = this->get_parameter("is_hp_deduced").as_bool();
}

void FakeRefereeNode::PublishRobotStatus()
{
    pb_rm_interfaces::msg::RobotStatus msg;
    msg.robot_id = rst_pb.robot_id;
    msg.robot_level = rst_pb.robot_level;
    msg.current_hp = rst_pb.current_hp;
    msg.maximum_hp = rst_pb.maximum_hp;
    msg.robot_pos.position.x = rst_pb.translation[0];
    msg.robot_pos.position.y = rst_pb.translation[1];
    msg.robot_pos.position.z = rst_pb.translation[2];
    // RCLCPP_INFO(get_logger(),"pub translation:%f, %f, %f",msg.robot_pos.position.x, msg.robot_pos.position.y, msg.robot_pos.position.z);
    msg.robot_pos.orientation.x = rst_pb.rotation[0];
    msg.robot_pos.orientation.y = rst_pb.rotation[1];
    msg.robot_pos.orientation.z = rst_pb.rotation[2];
    msg.robot_pos.orientation.w = rst_pb.rotation[3];
    msg.armor_id = rst_pb.armor_id;
    msg.hp_deduction_reason = rst_pb.hp_deduction_reason;
    msg.is_hp_deduced = rst_pb.is_hp_deduced;
    msg.projectile_allowance_17mm = rst_pb.projectile_allowance_17mm;
    robot_status_pub_->publish(msg);
}

void FakeRefereeNode::SubRobotStatus(const rmoss_interfaces::msg::RobotStatus::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(),"get translation:%f, %f, %f", msg->gt_tf.translation.x, msg->gt_tf.translation.y, msg->gt_tf.translation.z);
    rst_pb.robot_id = msg->id;
    rst_pb.robot_level = msg->level;
    rst_pb.current_hp = msg->remain_hp;
    rst_pb.maximum_hp = msg->max_hp;
    rst_pb.translation = {msg->gt_tf.translation.x, msg->gt_tf.translation.y, msg->gt_tf.translation.z};
    // RCLCPP_INFO(get_logger(),"set translation:%f, %f, %f", rst_pb.translation[0], rst_pb.translation[1], rst_pb.translation[2]);
    rst_pb.rotation = {msg->gt_tf.rotation.x, msg->gt_tf.rotation.y, msg->gt_tf.rotation.z, msg->gt_tf.rotation.w};
}

void FakeRefereeNode::SubAttackInfo(const std_msgs::msg::String::SharedPtr msg)
{
    auto result = parse_attack_info(msg->data);  // msg.attack_info 是 std::string
    if (result) {
        auto [shooterModel, shooterName, targetModel, targetLink] = result.value();
        RCLCPP_INFO(rclcpp::get_logger("parser"), 
                    "Shooter: %s/%s, Target: %s/%s",
                    shooterModel.c_str(), shooterName.c_str(),
                    targetModel.c_str(), targetLink.c_str());
        if (shooterModel == "red_standard_robot1")
        {
            rst_pb.projectile_allowance_17mm -= 1;
        }
        if (targetModel == "red_standard_robot1")
        {
            rst_pb.current_hp -= 10;
            rst_pb.is_hp_deduced = 1;
            rst_pb.hp_deduction_reason = ARMOR_HIT;
            const std::string prefix = "armor_";
            if (targetLink.rfind(prefix, 0) == 0 && targetLink.size() > prefix.size()) {
                try {
                    rst_pb.armor_id = std::stoi(targetLink.substr(prefix.size()));
                    RCLCPP_INFO(rclcpp::get_logger("parser"), "Armor ID: %d", rst_pb.armor_id);
                } catch (const std::exception &e) {
                    RCLCPP_WARN(rclcpp::get_logger("parser"), "Invalid armor ID format: %s", targetLink.c_str());
                }
            }
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("parser"), "Failed to parse attack info.");
    }
}

}// namespace fake_referee

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fake_referee::FakeRefereeNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}