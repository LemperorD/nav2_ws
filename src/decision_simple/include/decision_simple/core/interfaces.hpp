#pragma once

#include <cstdint>

namespace decision_simple {

  /// Chassis movement mode
  enum class ChassisMode : uint8_t {
    CHASSIS_FOLLOWED = 1,  ///< 底盘跟随云台
    LITTLE_TES = 2,        ///< 小陀螺
    GO_HOME = 3,           ///< 回家
  };

  /// Robot behavior state
  enum class State : uint8_t {
    DEFAULT = 1,  ///< 默认状态
    ATTACK = 2,   ///< 攻击状态
    SUPPLY = 3,   ///< 补给状态
  };

}  // namespace decision_simple
