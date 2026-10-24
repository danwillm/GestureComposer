#pragma once
// Declarative gesture-definition language (namespace gc::gestures).
//
//   Signal    - continuous per-hand score in [0,1] with a confidence (pinch, curl, palm facing, ...).
//               Combine with && (min), || (max), ! (1 - x).
//   Condition - a Signal turned into a debounced boolean: Schmitt trigger, velocity-adaptive dwell,
//               release dwell, refractory period, onset requirement, tracking-loss grace.
//   Pattern   - temporal expression over conditions: press, release, click, active, hold, seq, repeat, both, chord.
//   GestureDesc - binds a Pattern to context (arm/gate/cancelIf), arbitration (priority/exclusive/shared) and aim.

#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "features.h"
#include "types.h"

namespace gc {

namespace detail {
struct SignalNode;
struct PatternNode;
}  // namespace detail

struct Event;

enum class Target : uint8_t { Head, Gaze, Up, Down };
enum class Aim : uint8_t { None, IndexRay, ShoulderRay };
enum class Stabilize : uint8_t {
  None,           // report the current filtered ray
  RewindToOnset   // discrete gestures report the ray from just before the triggering motion began
};
enum class Resolve : uint8_t {
  Eager,  // fire immediately (default)
  Wait    // delay until no other gesture consumes the same physical action within the wait window
};

using CustomSignalFn = std::function<float(const HandFeatures&)>;

class Condition;

class Signal {
 public:
  Signal() = default;
  explicit Signal(std::shared_ptr<const detail::SignalNode> node) : node_(std::move(node)) {}

  bool valid() const { return node_ != nullptr; }
  const std::shared_ptr<const detail::SignalNode>& node() const { return node_; }

  // Shorthand for Condition(*this).thresholds(enter, exit).
  Condition above(float enter, float exit) const;

 private:
  std::shared_ptr<const detail::SignalNode> node_;
};

Signal operator&&(const Signal& a, const Signal& b);
Signal operator||(const Signal& a, const Signal& b);
Signal operator!(const Signal& a);

struct ConditionParams {
  float enter = 0.7f;          // signal must rise above this to activate
  float exit = 0.4f;           // ...and fall below this to release (hysteresis)
  float maxDwell = 0.06f;      // s; dwell for a slow approach. Shrinks to 0 as the rise speed reaches rateRef
  float rateRef = 6.f;         // 1/s; rise speed at which activation is immediate
  float releaseDwell = 0.04f;  // s; release must persist this long (survives one-frame glitches)
  float refractory = 0.05f;    // s; dead time after release (debounce)
  float rearmTime = 0.03f;     // s; signal must be below exit this long to (re)arm an onset condition
  float onsetWindow = 0.5f;    // s; for onset conditions, the "open" observation must be this recent
  float predict = 0.f;         // s; predictive onset: test enter against value + rate * predict
  float minConfidence = 0.3f;  // below this the condition freezes (suspended)
  float grace = 0.12f;         // s; how long a suspended active condition holds before cancelling
};

class Condition {
 public:
  Condition() = default;
  Condition(Signal s);  // NOLINT(google-explicit-constructor): implicit so Signals can be passed directly

  Condition& thresholds(float enter, float exit);
  Condition& dwell(Seconds maxDwell);
  Condition& rateRef(float perSecond);
  Condition& releaseDwell(Seconds s);
  Condition& refractory(Seconds s);
  Condition& rearm(Seconds s);
  Condition& onset(bool required);  // default: on for press/release/click/hold, off for active/arm/gate
  Condition& onsetWindow(Seconds s);
  Condition& predict(Seconds s);
  Condition& minConfidence(float c);
  Condition& grace(Seconds s);
  Condition& params(const ConditionParams& p);

  const Signal& signal() const { return signal_; }
  const ConditionParams& parameters() const { return params_; }
  bool hasParams() const { return hasParams_; }
  int onsetSetting() const { return onset_; }  // -1 auto, 0 off, 1 on

 private:
  Signal signal_;
  ConditionParams params_;
  bool hasParams_ = false;
  int onset_ = -1;
};

class Pattern {
 public:
  Pattern() = default;
  explicit Pattern(std::shared_ptr<const detail::PatternNode> node) : node_(std::move(node)) {}

  Pattern within(Seconds total) const;      // seq / repeat: whole pattern must complete within this
  Pattern maxGap(Seconds gap) const;        // seq: maximum time between consecutive steps
  Pattern sync(Seconds tolerance) const;    // both / chord: onsets of the two hands within this
  Pattern minDuration(Seconds d) const;     // active: must be held this long before Began

  bool valid() const { return node_ != nullptr; }
  const std::shared_ptr<const detail::PatternNode>& node() const { return node_; }

 private:
  std::shared_ptr<const detail::PatternNode> node_;
};

struct PinchOptions {
  float closeDistance = 0.005f;     // m (reference hand) surface distance mapping to strength 1
  float openDistance = 0.040f;      // m surface distance mapping to strength 0
  float exclusivityMargin = 0.012f; // m; how much closer than the next finger the target must be
  bool exclusive = true;            // suppress when another finger is about as close to the thumb

  PinchOptions& close(float m) { closeDistance = m; return *this; }
  PinchOptions& open(float m) { openDistance = m; return *this; }
  PinchOptions& margin(float m) { exclusivityMargin = m; return *this; }
  PinchOptions& exclusivity(bool on) { exclusive = on; return *this; }
};

namespace detail {
struct GestureData {
  std::string name;
  Pattern trigger;
  struct Arm {
    Condition condition;
    float minHold = 0.f;
  };
  std::vector<Arm> arms;
  std::vector<Condition> gates;
  std::vector<Condition> cancels;
  Hands hands = Hands::Either;
  int priority = 0;
  std::string exclusiveGroup;
  bool shared = false;
  Aim aim = Aim::None;
  Stabilize stabilize = Stabilize::None;
  Resolve resolve = Resolve::Eager;
  float waitWindow = 0.3f;
  bool emitUpdates = true;
  std::function<void(const Event&)> callback;
};
}  // namespace detail

class GestureDesc {
 public:
  explicit GestureDesc(std::string name);

  GestureDesc& trigger(const Pattern& p);
  // Context that must be active for >= minHold *before* the trigger's onset (and still active when it fires).
  GestureDesc& arm(const Condition& c, Seconds minHold = Seconds(0.f));
  // Context that must be active when the trigger fires / begins.
  GestureDesc& gate(const Condition& c);
  // Cancels an active continuous gesture (or a waiting discrete one) when this becomes active.
  GestureDesc& cancelIf(const Condition& c);
  GestureDesc& hands(Hands h);
  GestureDesc& priority(int p);
  GestureDesc& exclusive(std::string group);
  GestureDesc& shared(bool s = true);
  GestureDesc& aim(Aim a, Stabilize s = Stabilize::None);
  GestureDesc& ambiguity(Resolve r, Seconds window = Seconds(0.3f));
  GestureDesc& updates(bool emitUpdated);
  GestureDesc& onEvent(std::function<void(const Event&)> cb);

  const detail::GestureData& data() const { return data_; }

 private:
  detail::GestureData data_;
};

namespace gestures {

using gc::Aim;
using gc::Condition;
using gc::Finger;
using gc::GestureDesc;
using gc::Hand;
using gc::Hands;
using gc::Pattern;
using gc::PinchOptions;
using gc::Resolve;
using gc::Signal;
using gc::Stabilize;
using gc::Target;

// ---- Signals ------------------------------------------------------------------------------------
Signal pinch(Finger f, const PinchOptions& o = {});  // thumb to finger (f != Thumb)
Signal curl(Finger f);                               // raw curl 0..1
Signal extended(Finger f);                           // high when straight
Signal curled(Finger f);                             // high when curled
template <class... More>
Signal extended(Finger f, More... more) { return (extended(f) && ... && extended(more)); }
template <class... More>
Signal curled(Finger f, More... more) { return (curled(f) && ... && curled(more)); }
Signal spread(Finger f, float minDeg = 5.f, float fullDeg = 15.f);  // gap between f and the next finger
Signal palmFacing(Target t, float coneDeg = 35.f);
Signal inView(float coneDeg = 30.f);                 // hand near the centre of the head's view
Signal still(float maxSpeed = 0.25f, float maxAngularDegPerSec = 120.f);
Signal grasping();                                   // built-in occupied-hand detector
Signal occupied();                                   // app-provided (Composer::setHandOccupied)
Signal tracked();                                    // overall tracking confidence
Signal constant(float v);
Signal custom(std::string name, CustomSignalFn fn, FeatureMask needs = Features::All);

// ---- Patterns -----------------------------------------------------------------------------------
Pattern press(const Condition& c);
Pattern release(const Condition& c);
// Press then release within maxDuration, moving the palm less than maxTravel metres.
Pattern click(const Condition& c, Seconds maxDuration = Seconds(0.5f), float maxTravel = 0.06f);
Pattern active(const Condition& c);  // continuous: Began / Updated / Ended
Pattern hold(const Condition& c, Seconds duration);
Pattern seq(std::initializer_list<Pattern> steps);
template <class... P>
Pattern seq(const Pattern& a, const Pattern& b, const P&... rest) { return seq({a, b, rest...}); }
Pattern repeat(const Pattern& p, int count);
Pattern both(const Pattern& p);  // same pattern on both hands
Pattern chord(const Pattern& left, const Pattern& right);

GestureDesc gesture(std::string name);

}  // namespace gestures
}  // namespace gc
