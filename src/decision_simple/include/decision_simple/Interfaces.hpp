#include <cstdint>

typedef enum {
  chassisFollowed = 1,
  littleTES,
  goHome,
} chassisMode;
enum class State : uint8_t { DEFAULT = 1, ATTACK = 2, SUPPLY = 3 };