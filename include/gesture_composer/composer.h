#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gestures.h"
#include "features.h"
#include "types.h"

namespace gc {

using GestureId = uint32_t;
constexpr GestureId kInvalidGesture = ~0u;

enum class Phase : uint8_t {
  Armed,      // arm conditions satisfied: the trigger can now fire (show an affordance)
  Disarmed,
  Began,      // continuous gestures
  Updated,
  Ended,
  Cancelled,  // tracking timed out, cancelIf, or pre-empted in an exclusive group
  Triggered   // discrete gestures
};

struct Event {
  GestureId id = kInvalidGesture;
  Hand hand = Hand::Left;    // for two-handed gestures: the hand that completed it
  bool bothHands = false;
  Phase phase = Phase::Triggered;
  double time = 0.0;         // sample time of the deciding frame
  float strength = 0.f;      // signal value at the decision
  float progress = 0.f;      // hold / sequence progress 0..1
  bool rayValid = false;
  Ray ray;
  uint32_t serial = 0;       // monotonically increasing
};

struct GestureState {
  bool armed = false;
  bool active = false;       // continuous gesture began and has not ended
  float progress = 0.f;
  float strength = 0.f;
  bool rayValid = false;
  Ray ray;
  double since = 0.0;        // time of last state change
};

struct SignalValue {
  float value = 0.f;
  float confidence = 0.f;
};

struct Config {
  ConditionParams conditionDefaults;
  float shoulderOffset = 0.18f;  // m, lateral offset from head centre for shoulder rays
  float shoulderDrop = 0.15f;    // m, below the head
  float graspHoldoff = 0.3f;     // s, grasping() stays high this long after a grasp ends
  float rewindMax = 0.25f;       // s, furthest back RewindToOnset may look
  float rewindStableSpeed = 0.45f;  // rad/s; ray angular speed regarded as "holding still" when rewinding
  float inputTimeout = 0.1f;     // s; a hand/head with no new data for this long counts as untracked
  // true: events() holds only the events of the latest update() call.
  // false: events accumulate across update() calls until clearEvents() (e.g. one update() per hand,
  // one read per frame). Callbacks always fire exactly once, inside the update() that produced them.
  bool autoClearEvents = true;
  size_t eventCapacity = 0;      // preallocated event buffer; 0 = automatic. Overflow counts in droppedEvents()
};

struct CompileResult {
  bool ok = true;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

template <class T>
struct Span {
  const T* first = nullptr;
  size_t count = 0;
  const T* begin() const { return first; }
  const T* end() const { return first + count; }
  size_t size() const { return count; }
  bool empty() const { return count == 0; }
  const T& operator[](size_t i) const { return first[i]; }
};

// Single-threaded: create, add gestures, then call update() once per frame on one thread.
// update() performs no heap allocation once compiled.
class Composer {
 public:
  explicit Composer(const Config& config = {});
  ~Composer();
  Composer(Composer&&) noexcept;
  Composer& operator=(Composer&&) noexcept;
  Composer(const Composer&) = delete;
  Composer& operator=(const Composer&) = delete;

  // ---- setup (may allocate; adding/removing after compile triggers a recompile that resets runtime state)
  GestureId add(const GestureDesc& desc);
  void remove(GestureId id);
  void track(const Signal& s);  // make a signal queryable via signal() even if no gesture uses it
  const CompileResult& compile();

  // ---- per frame
  // nullptr means "no new data for this hand/head in this call", not "untracked": the last state is
  // held, so hands may be fed in separate calls (update(t, &left, nullptr) ... update(t, nullptr, &right)).
  // A hand becomes untracked when its sample has tracked == false, or when no new sample has arrived
  // for Config::inputTimeout. Samples whose HandSample::time was already processed are ignored.
  // `time` must not go backwards; the 3-argument form uses the newest timestamp among the inputs.
  void update(const HandSample* left, const HandSample* right, const HeadSample* head = nullptr);
  void update(double time, const HandSample* left, const HandSample* right, const HeadSample* head = nullptr);
  Span<Event> events() const;  // see Config::autoClearEvents
  void clearEvents();          // empties events(); needed when autoClearEvents is false

  // ---- queries / context
  GestureState state(GestureId id, Hand hand) const;
  SignalValue signal(const Signal& s, Hand hand) const;
  const HandFeatures& features(Hand hand) const;
  const char* name(GestureId id) const;
  GestureId find(const std::string& name) const;

  void setEnabled(GestureId id, bool enabled);
  bool enabled(GestureId id) const;
  void setHandOccupied(Hand hand, bool occupied);
  void reset();  // clears all runtime state (filters, conditions, matchers)

  uint32_t droppedEvents() const;  // events dropped because the preallocated buffer was full
  FeatureMask featureMask() const; // features currently being computed

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gc
