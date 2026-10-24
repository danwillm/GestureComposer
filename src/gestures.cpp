#include "gesture_composer/gestures.h"

#include "nodes.h"

namespace gc {

using detail::PatternNode;
using detail::PatternOp;
using detail::SignalNode;
using detail::SignalOp;

namespace {

std::shared_ptr<SignalNode> makeSignal(SignalOp op, FeatureMask mask) {
  auto n = std::make_shared<SignalNode>();
  n->op = op;
  n->mask = mask;
  return n;
}

Signal binary(SignalOp op, const Signal& a, const Signal& b) {
  auto n = makeSignal(op, (a.valid() ? a.node()->mask : 0u) | (b.valid() ? b.node()->mask : 0u));
  n->a = a.node();
  n->b = b.node();
  return Signal(n);
}

std::shared_ptr<PatternNode> makeLeaf(PatternOp op, const Condition& c) {
  auto n = std::make_shared<PatternNode>();
  n->op = op;
  n->condition = c;
  return n;
}

Pattern modified(const Pattern& p, const char* what, bool allowed, void (*apply)(PatternNode&, float), float v) {
  if (!p.valid()) return p;
  auto n = std::make_shared<PatternNode>(*p.node());
  if (allowed) {
    apply(*n, v);
  } else if (n->misuse.empty()) {
    n->misuse = std::string(what) + " is not supported on this pattern";
  }
  return Pattern(n);
}

}  // namespace

// ---- Signal ------------------------------------------------------------------------------------

Signal operator&&(const Signal& a, const Signal& b) { return binary(SignalOp::And, a, b); }
Signal operator||(const Signal& a, const Signal& b) { return binary(SignalOp::Or, a, b); }
Signal operator!(const Signal& a) {
  auto n = makeSignal(SignalOp::Not, a.valid() ? a.node()->mask : 0u);
  n->a = a.node();
  return Signal(n);
}

Condition Signal::above(float enter, float exit) const { return Condition(*this).thresholds(enter, exit); }

// ---- Condition ---------------------------------------------------------------------------------

Condition::Condition(Signal s) : signal_(std::move(s)) {}
Condition& Condition::thresholds(float enter, float exit) {
  params_.enter = enter;
  params_.exit = exit;
  hasParams_ = true;
  return *this;
}
Condition& Condition::dwell(Seconds s) { params_.maxDwell = s.value; hasParams_ = true; return *this; }
Condition& Condition::rateRef(float r) { params_.rateRef = r; hasParams_ = true; return *this; }
Condition& Condition::releaseDwell(Seconds s) { params_.releaseDwell = s.value; hasParams_ = true; return *this; }
Condition& Condition::refractory(Seconds s) { params_.refractory = s.value; hasParams_ = true; return *this; }
Condition& Condition::rearm(Seconds s) { params_.rearmTime = s.value; hasParams_ = true; return *this; }
Condition& Condition::onset(bool required) { onset_ = required ? 1 : 0; return *this; }
Condition& Condition::onsetWindow(Seconds s) { params_.onsetWindow = s.value; hasParams_ = true; return *this; }
Condition& Condition::predict(Seconds s) { params_.predict = s.value; hasParams_ = true; return *this; }
Condition& Condition::minConfidence(float c) { params_.minConfidence = c; hasParams_ = true; return *this; }
Condition& Condition::grace(Seconds s) { params_.grace = s.value; hasParams_ = true; return *this; }
Condition& Condition::params(const ConditionParams& p) { params_ = p; hasParams_ = true; return *this; }

// ---- Pattern -----------------------------------------------------------------------------------

Pattern Pattern::within(Seconds t) const {
  const bool ok = valid() && (node()->op == PatternOp::Seq || node()->op == PatternOp::Repeat);
  return modified(*this, "within()", ok, [](PatternNode& n, float v) { n.within = v; }, t.value);
}
Pattern Pattern::maxGap(Seconds t) const {
  const bool ok = valid() && node()->op == PatternOp::Seq;
  return modified(*this, "maxGap()", ok, [](PatternNode& n, float v) { n.maxGap = v; }, t.value);
}
Pattern Pattern::sync(Seconds t) const {
  const bool ok = valid() && (node()->op == PatternOp::Both || node()->op == PatternOp::Chord);
  return modified(*this, "sync()", ok, [](PatternNode& n, float v) { n.sync = v; }, t.value);
}
Pattern Pattern::minDuration(Seconds t) const {
  const bool ok = valid() && node()->op == PatternOp::Active;
  return modified(*this, "minDuration()", ok, [](PatternNode& n, float v) { n.duration = v; }, t.value);
}

// ---- GestureDesc -------------------------------------------------------------------------------

GestureDesc::GestureDesc(std::string name) { data_.name = std::move(name); }
GestureDesc& GestureDesc::trigger(const Pattern& p) { data_.trigger = p; return *this; }
GestureDesc& GestureDesc::arm(const Condition& c, Seconds minHold) {
  data_.arms.push_back({c, minHold.value});
  return *this;
}
GestureDesc& GestureDesc::gate(const Condition& c) { data_.gates.push_back(c); return *this; }
GestureDesc& GestureDesc::cancelIf(const Condition& c) { data_.cancels.push_back(c); return *this; }
GestureDesc& GestureDesc::hands(Hands h) { data_.hands = h; return *this; }
GestureDesc& GestureDesc::priority(int p) { data_.priority = p; return *this; }
GestureDesc& GestureDesc::exclusive(std::string group) { data_.exclusiveGroup = std::move(group); return *this; }
GestureDesc& GestureDesc::shared(bool s) { data_.shared = s; return *this; }
GestureDesc& GestureDesc::aim(Aim a, Stabilize s) {
  data_.aim = a;
  data_.stabilize = s;
  return *this;
}
GestureDesc& GestureDesc::ambiguity(Resolve r, Seconds window) {
  data_.resolve = r;
  data_.waitWindow = window.value;
  return *this;
}
GestureDesc& GestureDesc::updates(bool emit) { data_.emitUpdates = emit; return *this; }
GestureDesc& GestureDesc::onEvent(std::function<void(const Event&)> cb) {
  data_.callback = std::move(cb);
  return *this;
}

// ---- gestures free functions -------------------------------------------------------------------

namespace gestures {

Signal pinch(Finger f, const PinchOptions& o) {
  auto n = makeSignal(SignalOp::Pinch, Features::Pinch);
  n->finger = f;
  n->p[0] = o.closeDistance;
  n->p[1] = o.openDistance;
  n->p[2] = o.exclusivityMargin;
  n->flag = o.exclusive;
  return Signal(n);
}

Signal curl(Finger f) {
  auto n = makeSignal(SignalOp::Curl, Features::Curl);
  n->finger = f;
  return Signal(n);
}

Signal extended(Finger f) {
  auto n = makeSignal(SignalOp::Extended, Features::Curl);
  n->finger = f;
  n->p[0] = 0.25f;  // curl at which "extended" starts to fall
  n->p[1] = 0.5f;   // curl at which it reaches 0
  return Signal(n);
}

Signal curled(Finger f) {
  auto n = makeSignal(SignalOp::Curled, Features::Curl);
  n->finger = f;
  n->p[0] = 0.4f;
  n->p[1] = 0.65f;
  return Signal(n);
}

Signal spread(Finger f, float minDeg, float fullDeg) {
  auto n = makeSignal(SignalOp::Spread, Features::Spread);
  n->finger = f;
  n->p[0] = degToRad(minDeg);
  n->p[1] = degToRad(fullDeg);
  return Signal(n);
}

Signal palmFacing(Target t, float coneDeg) {
  auto n = makeSignal(SignalOp::PalmFacing, Features::None);
  n->target = t;
  n->p[0] = degToRad(coneDeg);
  return Signal(n);
}

Signal inView(float coneDeg) {
  auto n = makeSignal(SignalOp::InView, Features::None);
  n->p[0] = degToRad(coneDeg);
  return Signal(n);
}

Signal still(float maxSpeed, float maxAngularDegPerSec) {
  auto n = makeSignal(SignalOp::Still, Features::Kinematics);
  n->p[0] = maxSpeed;
  n->p[1] = degToRad(maxAngularDegPerSec);
  return Signal(n);
}

Signal grasping() {
  return Signal(makeSignal(SignalOp::Grasping,
                           Features::Grasp | Features::Curl | Features::Pinch | Features::Kinematics));
}
Signal occupied() { return Signal(makeSignal(SignalOp::Occupied, Features::None)); }
Signal tracked() { return Signal(makeSignal(SignalOp::Tracked, Features::None)); }

Signal constant(float v) {
  auto n = makeSignal(SignalOp::Constant, Features::None);
  n->p[0] = v;
  return Signal(n);
}

Signal custom(std::string name, CustomSignalFn fn, FeatureMask needs) {
  auto n = makeSignal(SignalOp::Custom, needs);
  n->name = std::move(name);
  n->fn = std::move(fn);
  return Signal(n);
}

Pattern press(const Condition& c) { return Pattern(makeLeaf(PatternOp::Press, c)); }
Pattern release(const Condition& c) { return Pattern(makeLeaf(PatternOp::Release, c)); }

Pattern click(const Condition& c, Seconds maxDuration, float maxTravel) {
  auto n = makeLeaf(PatternOp::Click, c);
  n->duration = maxDuration.value;
  n->travel = maxTravel;
  return Pattern(n);
}

Pattern active(const Condition& c) { return Pattern(makeLeaf(PatternOp::Active, c)); }

Pattern hold(const Condition& c, Seconds duration) {
  auto n = makeLeaf(PatternOp::Hold, c);
  n->duration = duration.value;
  return Pattern(n);
}

Pattern seq(std::initializer_list<Pattern> steps) {
  auto n = std::make_shared<PatternNode>();
  n->op = PatternOp::Seq;
  for (const Pattern& p : steps) n->children.push_back(p.node());
  return Pattern(n);
}

Pattern repeat(const Pattern& p, int count) {
  auto n = std::make_shared<PatternNode>();
  n->op = PatternOp::Repeat;
  n->count = count;
  n->children.push_back(p.node());
  return Pattern(n);
}

Pattern both(const Pattern& p) {
  auto n = std::make_shared<PatternNode>();
  n->op = PatternOp::Both;
  n->children.push_back(p.node());
  return Pattern(n);
}

Pattern chord(const Pattern& left, const Pattern& right) {
  auto n = std::make_shared<PatternNode>();
  n->op = PatternOp::Chord;
  n->children.push_back(left.node());
  n->children.push_back(right.node());
  return Pattern(n);
}

GestureDesc gesture(std::string name) { return GestureDesc(std::move(name)); }

}  // namespace gestures
}  // namespace gc
