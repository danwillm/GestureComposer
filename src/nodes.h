#pragma once
// Internal AST nodes produced by the gc::gestures builders and consumed by the compiler.

#include <memory>
#include <string>
#include <vector>

#include "gesture_composer/gestures.h"

namespace gc {
namespace detail {

enum class SignalOp : uint8_t {
  Constant,
  Pinch,
  Curl,
  Extended,
  Curled,
  Spread,
  PalmFacing,
  InView,
  Still,
  Grasping,
  Occupied,
  Tracked,
  Custom,
  And,
  Or,
  Not
};

struct SignalNode {
  SignalOp op = SignalOp::Constant;
  Finger finger = Finger::Index;
  Target target = Target::Head;
  float p[4] = {0.f, 0.f, 0.f, 0.f};
  bool flag = false;
  std::shared_ptr<const SignalNode> a, b;
  CustomSignalFn fn;
  FeatureMask mask = Features::None;
  std::string name;
};

enum class PatternOp : uint8_t { Press, Release, Click, Active, Hold, Seq, Repeat, Both, Chord };

struct PatternNode {
  PatternOp op = PatternOp::Press;
  Condition condition;  // leaf ops
  std::vector<std::shared_ptr<const PatternNode>> children;
  float duration = 0.f;  // click max duration / hold duration / active min duration
  float travel = 0.f;    // click max travel
  float within = 0.f;    // seq / repeat total limit (0 = none)
  float maxGap = 0.f;    // seq step gap limit (0 = none)
  float sync = 0.15f;    // both / chord
  int count = 2;         // repeat
  std::string misuse;    // set when a modifier was applied to an op that does not support it
};

inline bool isLeaf(PatternOp op) {
  return op == PatternOp::Press || op == PatternOp::Release || op == PatternOp::Click ||
         op == PatternOp::Active || op == PatternOp::Hold;
}

}  // namespace detail
}  // namespace gc
