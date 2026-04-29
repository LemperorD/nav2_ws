#include "decision_simple/core/environment_context.hpp"
#include "decision_simple/core/types.hpp"

namespace decision_simple {
  class Decision {
  public:
    explicit Decision(const ContextConfig& context_config);
    DecisionAction compute(const Snapshot& s);

    // Complete tick decision logic - takes snapshot and returns full action
    DecisionAction computeAction(const Snapshot& snapshot) const;

    bool isStatusRecovered(const RobotStatus& rs) const;
    bool isStatusBad(const RobotStatus& rs) const;

    // Attack goal computation
    bool buildAttackGoal(Snapshot& snapshot, const Armors& armors,
                         const std::optional<Target>& target_opt) const;

  private:
    int hp_enter_supply_{120};
    int hp_exit_supply_{300};
    int ammo_min_{0};
    double combat_max_distance_{8.0};
    bool require_game_running_{false};
    double start_delay_sec_{5.0};
    double supply_x_{0.0}, supply_y_{0.0}, supply_yaw_{0.0};
    double default_x_{1.0}, default_y_{0.0}, default_yaw_{0.0};
    double x_{0.0}, y_{0.0}, yaw_{0.0};
  };
}  // namespace decision_simple