#include "decision_simple/core/environment_context.hpp"
#include "decision_simple/core/types.hpp"

namespace decision_simple {
  class Decision {
  public:
    explicit Decision(const ContextConfig& context_config);
    DecisionAction compute(const Snapshot& s);

    DecisionAction computeAction(const Snapshot& snapshot) const;

    bool isStatusRecovered(const RobotStatus& rs) const;
    bool isStatusBad(const RobotStatus& rs) const;

    bool buildAttackGoal(Snapshot& snapshot, const Armors& armors,
                         const std::optional<Target>& target_opt) const;

  private:
    const ContextConfig config;
    double x_{0.0}, y_{0.0}, yaw_{0.0};
  };
}  // namespace decision_simple