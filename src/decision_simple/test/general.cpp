#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "decision_simple/node/decision_simple.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace decision_simple {
  using GameProgress = uint8_t;
  using namespace std::chrono_literals;

  class DecisionSimpleTest : public testing::Test {
  protected:
    struct CreateSystemInput {
      bool require_game_running = false;
      double start_delay_sec = 0.0;
      double attacked_hold_sec = 1.5;
      double default_goal_hz = 2.0;
      double supply_goal_hz = 2.0;
      int hp_survival_enter = 120;
      int hp_survival_exit = 300;
      int ammo_min = 0;
    };

    DecisionSimpleTest() {
      CreateSystem(CreateSystemInput{});
    }

    ~DecisionSimpleTest() override = default;

    void TearDown() override {
      ASSERT_NO_THROW(DestroySystem())
          << "TearDown 失败：释放测试系统资源时抛出异常。";
    }

    void CreateSystem(const CreateSystemInput& config) {
      rclcpp::NodeOptions options;
      options.append_parameter_override("require_game_running",
                                        config.require_game_running);
      options.append_parameter_override("start_delay_sec",
                                        config.start_delay_sec);
      options.append_parameter_override("attacked_hold_sec",
                                        config.attacked_hold_sec);
      options.append_parameter_override("default_goal_hz",
                                        config.default_goal_hz);
      options.append_parameter_override("supply_goal_hz",
                                        config.supply_goal_hz);
      options.append_parameter_override("hp_survival_enter",
                                        config.hp_survival_enter);
      options.append_parameter_override("hp_survival_exit",
                                        config.hp_survival_exit);
      options.append_parameter_override("ammo_min", config.ammo_min);

      node_under_test_ = std::make_shared<decision_simple::DecisionSimple>(
          options);
      gameStatusController_ = std::make_shared<rclcpp::Node>(
          "game_status_publisher");

      game_status_pub_ =
          gameStatusController_
              ->create_publisher<pb_rm_interfaces::msg::GameStatus>(
                  "referee/game_status", rclcpp::QoS(10));
      robot_status_pub_ =
          gameStatusController_
              ->create_publisher<pb_rm_interfaces::msg::RobotStatus>(
                  "referee/robot_status", rclcpp::QoS(10));
      chassis_mode_sub_ =
          gameStatusController_->create_subscription<std_msgs::msg::UInt8>(
              "chassis_mode", rclcpp::QoS(10),
              [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                chassis_modes_.push_back(msg->data);
              });

      goal_pose_sub_ =
          gameStatusController_
              ->create_subscription<geometry_msgs::msg::PoseStamped>(
                  "goal_pose", rclcpp::SensorDataQoS(),
                  [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    goal_poses_.push_back(*msg);
                  });

      exec_.add_node(node_under_test_);
      exec_.add_node(gameStatusController_);

      WaitForConnections();
    }

    void WaitForConnections() {
      WaitUntil([this]() { return AllConnected(); }, 1000ms);
    }

    bool AllConnected() {
      const bool is_game_connected = game_status_pub_->get_subscription_count()
                                   > 0;
      const bool is_robot_connected =
          robot_status_pub_->get_subscription_count() > 0;
      const bool is_mode_connected = chassis_mode_sub_->get_publisher_count()
                                   > 0;
      const bool is_goal_connected = goal_pose_sub_->get_publisher_count() > 0;

      return is_game_connected && is_robot_connected && is_mode_connected
          && is_goal_connected;
    }

    bool WaitForChassisAtLeast(size_t chassisModeCount,
                               std::chrono::milliseconds timeout) {
      return WaitUntil(
          [this, chassisModeCount]() {
            return chassis_modes_.size() >= chassisModeCount;
          },
          timeout);
    }

    bool WaitForGoalAtLeast(size_t goalPoseCount,
                            std::chrono::milliseconds timeout) {
      return WaitUntil(
          [this, goalPoseCount]() {
            return goal_poses_.size() >= goalPoseCount;
          },
          timeout);
    }

    template <typename Predicate>
    bool WaitUntil(Predicate&& predicate, std::chrono::milliseconds timeout) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < timeout) {
        exec_.spin_some();
        if (predicate()) {
          return true;
        }
        std::this_thread::sleep_for(10ms);
      }
      return false;
    }

    void DestroySystem() {
      ClearReceived();
      RemoveNode();
      ResetPointers();
    }

    void ClearReceived() {
      chassis_modes_.clear();
      goal_poses_.clear();
    }

    void RemoveNode() {
      exec_.remove_node(gameStatusController_);
      exec_.remove_node(node_under_test_);
    }

    void ResetPointers() {
      game_status_pub_.reset();
      robot_status_pub_.reset();
      chassis_mode_sub_.reset();
      goal_pose_sub_.reset();
      gameStatusController_.reset();
      node_under_test_.reset();
    }

    template <typename PublisherT, typename MsgT>
    void PublishAndSpin(const std::shared_ptr<PublisherT>& publisher,
                        const MsgT& msg) {
      publisher->publish(msg);
      exec_.spin_some();
    }

    void SendGameStatus(GameProgress theGameProgress) {
      constexpr int32_t sendingDurationSeconds = 300;
      pb_rm_interfaces::msg::GameStatus gameStatus;
      gameStatus.game_progress = theGameProgress;
      gameStatus.stage_remain_time = sendingDurationSeconds;
      PublishAndSpin(game_status_pub_, gameStatus);
    }

    void SendRobotStatus(uint16_t hp, uint16_t ammo,
                         bool is_hp_deduced = false) {
      pb_rm_interfaces::msg::RobotStatus status;
      status.current_hp = hp;
      status.projectile_allowance_17mm = ammo;
      status.is_hp_deduced = is_hp_deduced;
      PublishAndSpin(robot_status_pub_, status);
    }
    //

    void SpinFor(std::chrono::milliseconds timeout) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < timeout) {
        exec_.spin_some();
        std::this_thread::sleep_for(10ms);
      }
    }

    size_t ChassisCount() {
      return chassis_modes_.size();
    }

    size_t GoalCount() {
      return goal_poses_.size();
    }

    uint8_t LastChassisMode() {
      return chassis_modes_.empty() ? 0 : chassis_modes_.back();
    }

    geometry_msgs::msg::PoseStamped LastGoalPose() {
      return goal_poses_.empty() ? geometry_msgs::msg::PoseStamped{}
                                 : goal_poses_.back();
    }

    rclcpp::executors::SingleThreadedExecutor exec_;
    std::shared_ptr<decision_simple::DecisionSimple> node_under_test_;
    rclcpp::Node::SharedPtr gameStatusController_;

    rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr
        game_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr
        robot_status_pub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_pose_sub_;

    std::vector<uint8_t> chassis_modes_;
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;
  };

  struct GateCase {
    bool require_game_running;
    double start_delay_sec;
    bool send_robot;
    bool send_game;
    uint8_t game_progress;
    std::chrono::milliseconds wait;
    bool expect_output;
    uint8_t expected_chassis = 0;
    bool check_default_goal = false;
  };

  GateCase MakeGateCase(bool require_game_running, double start_delay_sec,
                        bool send_robot, bool send_game, uint8_t game_progress,
                        std::chrono::milliseconds wait, bool expect_output,
                        uint8_t expected_chassis = 0,
                        bool check_default_goal = false) {
    GateCase gate_case{};
    gate_case.require_game_running = require_game_running;
    gate_case.start_delay_sec = start_delay_sec;
    gate_case.send_robot = send_robot;
    gate_case.send_game = send_game;
    gate_case.game_progress = game_progress;
    gate_case.wait = wait;
    gate_case.expect_output = expect_output;
    gate_case.expected_chassis = expected_chassis;
    gate_case.check_default_goal = check_default_goal;
    return gate_case;
  }

  class DecisionSimpleGateTest : public DecisionSimpleTest,
                                 public testing::WithParamInterface<GateCase> {
  protected:
    DecisionSimpleGateTest() {
      const GateCase& gate_case = GetParam();
      DestroySystem();
      CreateSystem(CreateSystemInput{gate_case.require_game_running,
                                     gate_case.start_delay_sec});
      ClearReceived();
    }
  };

  INSTANTIATE_TEST_SUITE_P(
      DecisionGateCases, DecisionSimpleGateTest,
      testing::Values(
          // 仅发送 game（不发送 robot）：应无输出。
          MakeGateCase(false, 0.0, false, true,
                       pb_rm_interfaces::msg::GameStatus::RUNNING, 500ms,
                       false),
          // 要求 RUNNING，但当前为 COUNT_DOWN：应无输出。
          MakeGateCase(true, 0.1, true, true,
                       pb_rm_interfaces::msg::GameStatus::COUNT_DOWN, 600ms,
                       false),
          // 要求 RUNNING，但未发送 game 状态：应无输出。
          MakeGateCase(true, 0.0, true, false,
                       pb_rm_interfaces::msg::GameStatus::NOT_START, 500ms,
                       false),
          // 要求 RUNNING，但未达到 start_delay：应无输出。
          MakeGateCase(true, 0.8, true, true,
                       pb_rm_interfaces::msg::GameStatus::RUNNING, 300ms,
                       false),
          // 要求 RUNNING 且达到 start_delay：应输出底盘模式与默认目标。
          MakeGateCase(true, 0.1, true, true,
                       pb_rm_interfaces::msg::GameStatus::RUNNING, 1400ms, true,
                       1u, true)));

  // 1.1) 门测试：根据参数决定是否允许输出，并在允许时校验模式与目标。
  TEST_P(DecisionSimpleGateTest, GateBehavior) {
    const GateCase& gate_case = GetParam();

    if (gate_case.send_robot) {
      SendRobotStatus(500, 120, false);
    }
    if (gate_case.send_game) {
      SendGameStatus(gate_case.game_progress);
    }

    SpinFor(gate_case.wait);

    if (!gate_case.expect_output) {
      EXPECT_EQ(ChassisCount(), 0u) << "门测试失败：期望无底盘模式输出。";
      EXPECT_EQ(GoalCount(), 0u) << "门测试失败：期望无目标点输出。";
      return;
    }

    ASSERT_TRUE(WaitForChassisAtLeast(1, 2000ms))
        << "门测试失败：超时未收到底盘模式。";
    EXPECT_EQ(LastChassisMode(), gate_case.expected_chassis)
        << "门测试失败：底盘模式与期望不一致。";
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms))
        << "门测试失败：超时未收到目标点。";

    if (gate_case.check_default_goal) {
      const auto goal = LastGoalPose();
      EXPECT_NEAR(goal.pose.position.x, 2.0, 1e-6)
          << "门测试失败：默认目标 x 坐标不正确。";
      EXPECT_NEAR(goal.pose.position.y, 0.5, 1e-6)
          << "门测试失败：默认目标 y 坐标不正确。";
    }
  }

  // 2.1) 逻辑测试：低血量应进入补给行为，发布补给点 goal（默认在 0,0）。
  TEST_F(DecisionSimpleTest, LowHp_EnterSupply_PublishesSupplyGoal) {
    SendRobotStatus(100, 120, false);

    ASSERT_TRUE(WaitForChassisAtLeast(1, 1200ms))
        << "低血量测试失败：超时未收到底盘模式。";
    EXPECT_EQ(LastChassisMode(), 1u)
        << "低血量测试失败：底盘模式应为补给模式(1)。";
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "低血量测试失败：超时未收到补给目标点。";

    const auto goal = LastGoalPose();
    EXPECT_NEAR(goal.pose.position.x, 0.0, 1e-6)
        << "低血量测试失败：补给目标 x 坐标不正确。";
    EXPECT_NEAR(goal.pose.position.y, 0.0, 1e-6)
        << "低血量测试失败：补给目标 y 坐标不正确。";
  }

  // 2.2) 逻辑测试：低弹量（<= ammo_min）也应进入补给。
  TEST_F(DecisionSimpleTest, LowAmmo_EnterSupply_PublishesSupplyGoal) {
    SendRobotStatus(500, 0, false);

    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "低弹量测试失败：超时未收到补给目标点。";
    const auto goal = LastGoalPose();
    EXPECT_NEAR(goal.pose.position.x, 0.0, 1e-6)
        << "低弹量测试失败：补给目标 x 坐标不正确。";
    EXPECT_NEAR(goal.pose.position.y, 0.0, 1e-6)
        << "低弹量测试失败：补给目标 y 坐标不正确。";
    EXPECT_EQ(LastChassisMode(), 1u)
        << "低弹量测试失败：底盘模式应为补给模式(1)。";
  }

  // 2.3) 逻辑测试：进入补给后未恢复时应保持补给目标。
  TEST_F(DecisionSimpleTest, SupplyHoldUntilRecovered) {
    SendRobotStatus(100, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "补给保持测试失败：首次超时未收到补给目标点。";
    const auto first_goal = LastGoalPose();
    EXPECT_NEAR(first_goal.pose.position.x, 0.0, 1e-6)
        << "补给保持测试失败：首次补给目标 x 坐标不正确。";
    EXPECT_NEAR(first_goal.pose.position.y, 0.0, 1e-6)
        << "补给保持测试失败：首次补给目标 y 坐标不正确。";

    ClearReceived();
    SendRobotStatus(200, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "补给保持测试失败：恢复前超时未收到目标点。";
    const auto hold_goal = LastGoalPose();
    EXPECT_NEAR(hold_goal.pose.position.x, 0.0, 1e-6)
        << "补给保持测试失败：恢复前目标 x 坐标应保持在补给点。";
    EXPECT_NEAR(hold_goal.pose.position.y, 0.0, 1e-6)
        << "补给保持测试失败：恢复前目标 y 坐标应保持在补给点。";
    EXPECT_EQ(LastChassisMode(), 1u)
        << "补给保持测试失败：恢复前底盘模式应保持补给模式(1)。";
  }

  // 2.4) 逻辑测试：达到恢复阈值后应退出补给并转默认目标。
  TEST_F(DecisionSimpleTest, SupplyRecovered_ExitToDefaultGoal) {
    SendRobotStatus(100, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "补给退出测试失败：进入补给阶段超时未收到目标点。";

    ClearReceived();
    SendRobotStatus(350, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 2000ms))
        << "补给退出测试失败：恢复后超时未收到默认目标点。";

    const auto goal = LastGoalPose();
    EXPECT_NEAR(goal.pose.position.x, 2.0, 1e-6)
        << "补给退出测试失败：恢复后默认目标 x 坐标不正确。";
    EXPECT_NEAR(goal.pose.position.y, 0.5, 1e-6)
        << "补给退出测试失败：恢复后默认目标 y 坐标不正确。";
    EXPECT_EQ(LastChassisMode(), 1u)
        << "补给退出测试失败：恢复后底盘模式应为跟随模式(1)。";
  }

  // 2.5) 逻辑测试：近期受击应触发 littleTES（2）。
  TEST_F(DecisionSimpleTest, AttackedRecent_UseLittleTESMode) {
    SendRobotStatus(500, 120, true);

    ASSERT_TRUE(WaitForChassisAtLeast(1, 1200ms))
        << "受击即时测试失败：超时未收到底盘模式。";
    EXPECT_EQ(LastChassisMode(), 2u)
        << "受击即时测试失败：受击后底盘模式应为 littleTES(2)。";
  }

  class DecisionSimpleAttackedHoldTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleAttackedHoldTest() {
      DestroySystem();
      constexpr double attacked_hold_sec = 0.2;
      CreateSystem(CreateSystemInput{false, 0.0, attacked_hold_sec});
      ClearReceived();
    }
  };

  // 3.1) 时间相关测试：attacked_hold_sec 到期后应回到非 littleTES（默认为
  // followed=1）。
  TEST_F(DecisionSimpleAttackedHoldTest, AttackedHoldExpired_BackToFollowed) {
    SendRobotStatus(500, 120, true);
    ASSERT_TRUE(WaitForChassisAtLeast(1, 1200ms))
        << "受击保持测试失败：受击后超时未收到底盘模式。";
    EXPECT_EQ(LastChassisMode(), 2u)
        << "受击保持测试失败：受击后底盘模式应先进入 littleTES(2)。";

    ClearReceived();
    SendRobotStatus(500, 120, false);
    SpinFor(400ms);

    ASSERT_TRUE(WaitForChassisAtLeast(1, 1200ms))
        << "受击保持测试失败：保持时间到期后超时未收到底盘模式。";
    EXPECT_EQ(LastChassisMode(), 1u)
        << "受击保持测试失败：保持时间到期后应回到跟随模式(1)。";
  }

  class DecisionSimpleRunningGateTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleRunningGateTest() {
      DestroySystem();
      CreateSystem(CreateSystemInput{true, 0.0});
      ClearReceived();
    }
  };

  // 1.2) 门测试：从 RUNNING 离开后，比赛门控应复位（不再输出）。
  TEST_F(DecisionSimpleRunningGateTest,
         LeaveRunning_ResetsGate_NoFurtherOutput) {
    SendRobotStatus(500, 120, false);
    SendGameStatus(pb_rm_interfaces::msg::GameStatus::RUNNING);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "门复位测试失败：RUNNING 阶段超时未收到目标点。";

    ClearReceived();
    SendGameStatus(pb_rm_interfaces::msg::GameStatus::GAME_OVER);
    SpinFor(500ms);

    EXPECT_EQ(ChassisCount(), 0u)
        << "门复位测试失败：离开 RUNNING 后不应继续输出底盘模式。";
    EXPECT_EQ(GoalCount(), 0u)
        << "门复位测试失败：离开 RUNNING 后不应继续输出目标点。";
  }

  class DecisionSimpleGoalHzTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleGoalHzTest() {
      DestroySystem();
      constexpr double default_goal_hz = 1.0;
      CreateSystem(CreateSystemInput{false, 0.0, 1.5, default_goal_hz});
      ClearReceived();
    }
  };

  // 3.2) 时间相关测试：默认目标发布应受 default_goal_hz 节流。
  TEST_F(DecisionSimpleGoalHzTest, DefaultGoalPublishing_ThrottledByHz) {
    SendRobotStatus(500, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "默认目标频率测试失败：超时未收到首次目标点。";

    const size_t first_count = GoalCount();
    SpinFor(300ms);
    EXPECT_EQ(GoalCount(), first_count)
        << "默认目标频率测试失败：节流窗口内不应新增目标点。";

    SpinFor(900ms);
    EXPECT_GE(GoalCount(), first_count + 1)
        << "默认目标频率测试失败：超过节流周期后应新增目标点。";
  }

  class DecisionSimpleSupplyHzTest : public DecisionSimpleTest {
  protected:
    DecisionSimpleSupplyHzTest() {
      DestroySystem();
      constexpr double supply_goal_hz = 1.0;
      CreateSystem(CreateSystemInput{false, 0.0, 1.5, 2.0, supply_goal_hz});
      ClearReceived();
    }
  };

  // 3.3) 时间相关测试：补给目标发布应受 supply_goal_hz 节流。
  TEST_F(DecisionSimpleSupplyHzTest, SupplyGoalPublishing_ThrottledByHz) {
    SendRobotStatus(50, 120, false);
    ASSERT_TRUE(WaitForGoalAtLeast(1, 1200ms))
        << "补给目标频率测试失败：超时未收到首次补给目标点。";

    const size_t first_count = GoalCount();
    SpinFor(300ms);
    EXPECT_EQ(GoalCount(), first_count)
        << "补给目标频率测试失败：节流窗口内不应新增补给目标点。";

    SpinFor(900ms);
    EXPECT_GE(GoalCount(), first_count + 1)
        << "补给目标频率测试失败：超过节流周期后应新增补给目标点。";
  }

}  // namespace decision_simple

class Ros2GlobalEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

::testing::Environment* const kRos2Env = ::testing::AddGlobalTestEnvironment(
    new Ros2GlobalEnvironment());