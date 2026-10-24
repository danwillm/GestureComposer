#include "gesture_composer/composer.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <sstream>
#include <unordered_map>

#include "condition.h"
#include "hand_tracker.h"
#include "nodes.h"

namespace gc {

using namespace detail;

namespace {

// ---- compiled program -----------------------------------------------------------------------------

struct CSignal {
  SignalOp op = SignalOp::Constant;
  Finger finger = Finger::Index;
  Target target = Target::Head;
  float p[4] = {};
  bool flag = false;
  int a = -1, b = -1;
  const CustomSignalFn* fn = nullptr;
  FeatureMask mask = 0;
};

struct CCond {
  int signal = -1;
  ConditionParams params;
  bool onset = false;
};

struct TokenSet {
  std::array<uint32_t, 8> v{};
  uint8_t n = 0;
  void clear() { n = 0; }
  void add(uint32_t t) {
    if (t == 0) return;
    for (uint8_t i = 0; i < n; ++i)
      if (v[i] == t) return;
    if (n < v.size()) {
      v[n++] = t;
    } else {
      std::memmove(&v[0], &v[1], sizeof(uint32_t) * (v.size() - 1));
      v[v.size() - 1] = t;
    }
  }
  void addAll(const TokenSet& o) {
    for (uint8_t i = 0; i < o.n; ++i) add(o.v[i]);
  }
  bool contains(uint32_t t) const {
    for (uint8_t i = 0; i < n; ++i)
      if (v[i] == t) return true;
    return false;
  }
};

struct PNode {
  PatternOp op = PatternOp::Press;
  int cond = -1;
  std::vector<int> children;
  float duration = 0.f, travel = 0.f, within = 0.f, maxGap = 0.f, sync = 0.15f;
  int count = 2;
  bool continuous = false;
};

struct POut {
  bool fired = false;
  bool active = false;
  bool cancelled = false;
  uint32_t token = 0;
  TokenSet tokens;
  double onset = 0.0;  // when the pattern's first physical motion crossed its threshold
  double quiet = 0.0;  // last still moment before the deciding motion (for ray rewind)
  float strength = 0.f;
  float progress = 0.f;
  int primary = -1;    // condition whose activation "owns" this firing
  int side = 0;        // two-handed: hand slot that completed it
};

struct PState {
  int step = 0;
  int count = 0;
  double t0 = 0.0, lastStep = 0.0;
  TokenSet acc;
  double accOnset = 0.0;
  bool held = false;
  double pressTime = 0.0, pressOnset = 0.0, pressQuiet = 0.0;
  Vec3 pressPos;
  float travel = 0.f;
  bool fired = false;
  bool begun = false;
  double sideFire[2] = {-1e9, -1e9};
  TokenSet sideTokens[2];
  uint32_t sideToken[2] = {0, 0};
  float sideStrength[2] = {0.f, 0.f};
  double sideOnset[2] = {0.0, 0.0};
  double sideQuiet[2] = {0.0, 0.0};
  double sideActiveSince[2] = {0.0, 0.0};
  bool sideWasActive[2] = {false, false};
};

struct PendingWait {
  bool valid = false;
  double deadline = 0.0;
  uint32_t token = 0;
  Event event;
};

struct Instance {
  std::vector<PState> states;
  std::vector<POut> outs;
  bool armed = false;
  bool active = false;
  double since = 0.0;
  uint32_t activeToken = 0;
  uint32_t blockedToken = 0;
  float progress = 0.f;
  float strength = 0.f;
  bool rayValid = false;
  Ray ray;
  PendingWait pending;
};

struct CGesture {
  GestureId id = kInvalidGesture;
  int order = 0;
  int priority = 0;
  int specificity = 0;
  bool shared = false;
  int group = -1;
  Hands hands = Hands::Either;
  bool chord = false;
  bool continuous = false;
  Aim aim = Aim::None;
  Stabilize stabilize = Stabilize::None;
  Resolve resolve = Resolve::Eager;
  float waitWindow = 0.3f;
  bool emitUpdates = true;
  std::vector<PNode> nodes;  // post-order, root last
  std::vector<std::pair<int, float>> arms;
  std::vector<int> gates, cancels;
  std::vector<int> conds;    // every condition used
  std::vector<int> signals;  // root signals of those conditions
  FeatureMask mask = 0;
  std::string key;
  Instance inst[2];
};

struct Candidate {
  int g = -1;
  int slot = 0;
  bool begin = false;
  uint32_t token = 0;
  TokenSet tokens;
  double time = 0.0;
  double quiet = 0.0;
  float strength = 0.f;
  float progress = 0.f;
  int primary = -1;
  bool won = false;
};

struct GroupOwner {
  int g = -1;
  int primary = -1;
  bool continuous = false;
};

std::string floatKey(float v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
  return buf;
}

}  // namespace

// =====================================================================================================

struct Composer::Impl {
  explicit Impl(const Config& c) : cfg(c) {}

  struct Def {
    GestureData data;
    bool removed = false;
    bool enabled = true;
  };

  Config cfg;
  std::vector<Def> defs;
  std::vector<Signal> trackedSignals;
  bool dirty = true;
  CompileResult result;

  // compiled
  std::vector<CSignal> signals;
  std::vector<std::shared_ptr<const SignalNode>> keepAlive;
  std::unordered_map<const SignalNode*, int> signalIndex;
  std::unordered_map<std::string, int> signalKeys;
  std::vector<CCond> conds;
  std::unordered_map<std::string, int> condKeys;
  std::vector<CGesture> gestures;
  std::vector<int> compiledOf;  // GestureId -> gestures index (or -1)
  std::vector<int> trackedSignalIdx;
  int groupCount = 0;

  // runtime
  std::vector<char> signalNeeded, condNeeded;
  std::vector<float> sigVal[2], sigConf[2];
  std::vector<ConditionRuntime> condRt[2];
  std::vector<std::array<GroupOwner, 2>> owners;
  std::vector<int> groupWinner;  // groupCount * 2, per frame
  std::vector<Candidate> candidates;
  std::vector<Event> events;
  HandTracker trackers[2] = {HandTracker(Hand::Left), HandTracker(Hand::Right)};
  HeadInfo head;
  bool occupied[2] = {false, false};
  double time = -1.0;
  double lastHeadTime = -1e9;
  double lastSampleTime[2] = {-1e9, -1e9};  // HandSample::time of the last processed sample
  double lastInputTime[2] = {-1e9, -1e9};   // frame time when fresh data last arrived
  uint32_t condSerial = 0;
  uint32_t eventSerial = 0;
  uint32_t dropped = 0;
  FeatureMask mask = 0;

  // ---- compile ----------------------------------------------------------------------------------

  void error(const std::string& g, const std::string& msg) {
    result.ok = false;
    result.errors.push_back("gesture '" + g + "': " + msg);
  }

  int internSignal(const std::shared_ptr<const SignalNode>& n) {
    auto it = signalIndex.find(n.get());
    if (it != signalIndex.end()) return it->second;
    CSignal cs;
    cs.op = n->op;
    cs.finger = n->finger;
    cs.target = n->target;
    std::memcpy(cs.p, n->p, sizeof(cs.p));
    cs.flag = n->flag;
    cs.mask = n->mask;
    if (n->a) cs.a = internSignal(n->a);
    if (n->b) cs.b = internSignal(n->b);
    std::ostringstream key;
    key << static_cast<int>(cs.op) << '|' << static_cast<int>(cs.finger) << '|' << static_cast<int>(cs.target) << '|'
        << floatKey(cs.p[0]) << ',' << floatKey(cs.p[1]) << ',' << floatKey(cs.p[2]) << ',' << floatKey(cs.p[3])
        << '|' << cs.flag << '|' << cs.a << '|' << cs.b;
    if (cs.op == SignalOp::Custom) key << "|custom@" << static_cast<const void*>(n.get());
    const std::string k = key.str();
    auto kit = signalKeys.find(k);
    int idx;
    if (kit != signalKeys.end()) {
      idx = kit->second;  // structurally identical: evaluate once (hash-consing)
    } else {
      if (cs.op == SignalOp::Custom) cs.fn = &n->fn;
      idx = static_cast<int>(signals.size());
      signals.push_back(cs);
      signalKeys.emplace(k, idx);
    }
    keepAlive.push_back(n);
    signalIndex.emplace(n.get(), idx);
    return idx;
  }

  int internCondition(const Condition& c, bool onsetDefault, const std::string& gname) {
    if (!c.signal().valid()) {
      error(gname, "condition has no signal");
      return -1;
    }
    CCond cc;
    cc.signal = internSignal(c.signal().node());
    cc.params = c.hasParams() ? c.parameters() : cfg.conditionDefaults;
    cc.onset = c.onsetSetting() < 0 ? onsetDefault : c.onsetSetting() == 1;
    const ConditionParams& p = cc.params;
    std::ostringstream key;
    key << cc.signal << '|' << cc.onset;
    for (float v : {p.enter, p.exit, p.maxDwell, p.rateRef, p.releaseDwell, p.refractory, p.rearmTime, p.onsetWindow,
                    p.predict, p.minConfidence, p.grace})
      key << ',' << floatKey(v);
    const std::string k = key.str();
    auto it = condKeys.find(k);
    if (it != condKeys.end()) return it->second;
    const int idx = static_cast<int>(conds.size());
    conds.push_back(cc);
    condKeys.emplace(k, idx);
    return idx;
  }

  // Returns node index; appends key text for duplicate detection.
  int compilePattern(const std::shared_ptr<const PatternNode>& n, CGesture& cg, bool isRoot, const std::string& gname,
                     std::string& key) {
    if (!n) {
      error(gname, "empty pattern");
      return -1;
    }
    if (!n->misuse.empty()) error(gname, n->misuse);
    PNode pn;
    pn.op = n->op;
    pn.duration = n->duration;
    pn.travel = n->travel;
    pn.within = n->within;
    pn.maxGap = n->maxGap;
    pn.sync = n->sync;
    pn.count = n->count;
    key += "(" + std::to_string(static_cast<int>(n->op));
    if (isLeaf(n->op)) {
      const bool onsetDefault = n->op != PatternOp::Active;
      pn.cond = internCondition(n->condition, onsetDefault, gname);
      pn.continuous = n->op == PatternOp::Active;
      key += ":c" + std::to_string(pn.cond) + ":" + floatKey(pn.duration) + ":" + floatKey(pn.travel);
    } else {
      if ((n->op == PatternOp::Both || n->op == PatternOp::Chord) && !isRoot)
        error(gname, "both()/chord() must be the outermost pattern of a trigger");
      if (n->op == PatternOp::Repeat && n->count < 1) error(gname, "repeat() count must be >= 1");
      if (n->children.empty()) error(gname, "pattern has no steps");
      for (const auto& c : n->children) pn.children.push_back(compilePattern(c, cg, false, gname, key));
      for (int c : pn.children) {
        if (c < 0) continue;
        const bool cc = cg.nodes[c].continuous;
        if ((n->op == PatternOp::Seq || n->op == PatternOp::Repeat) && cc)
          error(gname, "seq()/repeat() steps must be discrete (press, release, click, hold, ...), not active()");
      }
      if (n->op == PatternOp::Both || n->op == PatternOp::Chord) {
        pn.continuous = !pn.children.empty() && pn.children[0] >= 0 && cg.nodes[pn.children[0]].continuous;
        if (n->op == PatternOp::Chord && pn.children.size() == 2 && pn.children[1] >= 0 &&
            cg.nodes[pn.children[1]].continuous != pn.continuous)
          error(gname, "chord() sides must both be continuous or both be discrete");
      }
      key += ":" + floatKey(pn.within) + ":" + floatKey(pn.maxGap) + ":" + floatKey(pn.sync) + ":" +
             std::to_string(pn.count);
    }
    key += ")";
    cg.nodes.push_back(pn);
    return static_cast<int>(cg.nodes.size()) - 1;
  }

  // Number of physical actions a pattern consumes: seq sums, repeat multiplies, two-handed adds one.
  static int patternSteps(const CGesture& cg, int ni) {
    const PNode& pn = cg.nodes[ni];
    switch (pn.op) {
      case PatternOp::Seq: {
        int n = 0;
        for (int c : pn.children) n += patternSteps(cg, c);
        return n;
      }
      case PatternOp::Repeat: return std::max(pn.count, 1) * patternSteps(cg, pn.children[0]);
      case PatternOp::Both: return patternSteps(cg, pn.children[0]) + 1;
      case PatternOp::Chord: return patternSteps(cg, pn.children[0]) + patternSteps(cg, pn.children[1]);
      default: return 1;
    }
  }

  void compileGesture(GestureId id, const GestureData& d, std::unordered_map<std::string, int>& groups) {
    CGesture cg;
    cg.id = id;
    cg.order = static_cast<int>(id);
    cg.priority = d.priority;
    cg.shared = d.shared;
    cg.hands = d.hands;
    cg.aim = d.aim;
    cg.stabilize = d.stabilize;
    cg.resolve = d.resolve;
    cg.waitWindow = d.waitWindow;
    cg.emitUpdates = d.emitUpdates;
    if (!d.trigger.valid()) {
      error(d.name, "no trigger()");
      return;
    }
    std::string key;
    const int root = compilePattern(d.trigger.node(), cg, true, d.name, key);
    if (root < 0 || !result.ok) return;
    PatternOp rootOp = cg.nodes[root].op;
    if (d.hands == Hands::Both && rootOp != PatternOp::Both && rootOp != PatternOp::Chord) {
      PNode wrap;
      wrap.op = PatternOp::Both;
      wrap.children = {root};
      wrap.continuous = cg.nodes[root].continuous;
      cg.nodes.push_back(wrap);
      rootOp = PatternOp::Both;
      key = "(both" + key + ")";
    }
    cg.chord = rootOp == PatternOp::Both || rootOp == PatternOp::Chord;
    cg.continuous = cg.nodes.back().continuous;
    for (const auto& a : d.arms) cg.arms.emplace_back(internCondition(a.condition, false, d.name), a.minHold);
    for (const auto& g : d.gates) cg.gates.push_back(internCondition(g, false, d.name));
    for (const auto& c : d.cancels) cg.cancels.push_back(internCondition(c, false, d.name));
    if (!result.ok) return;
    // More context and longer patterns are more specific (longest match wins, as in Proton).
    cg.specificity = static_cast<int>(cg.arms.size() + cg.gates.size()) + patternSteps(cg, static_cast<int>(cg.nodes.size()) - 1) - 1;
    if (!d.exclusiveGroup.empty()) {
      if (cg.chord) {
        result.warnings.push_back("gesture '" + d.name + "': exclusive() is ignored for two-handed gestures");
      } else {
        auto it = groups.find(d.exclusiveGroup);
        if (it == groups.end()) it = groups.emplace(d.exclusiveGroup, static_cast<int>(groups.size())).first;
        cg.group = it->second;
      }
    }
    for (const PNode& pn : cg.nodes)
      if (pn.cond >= 0) cg.conds.push_back(pn.cond);
    for (const auto& a : cg.arms) cg.conds.push_back(a.first);
    for (int g : cg.gates) cg.conds.push_back(g);
    for (int c : cg.cancels) cg.conds.push_back(c);
    std::sort(cg.conds.begin(), cg.conds.end());
    cg.conds.erase(std::unique(cg.conds.begin(), cg.conds.end()), cg.conds.end());
    for (int c : cg.conds) cg.signals.push_back(conds[c].signal);
    if (cg.aim == Aim::IndexRay) cg.mask |= Features::AimIndex;
    if (cg.aim == Aim::ShoulderRay) cg.mask |= Features::AimShoulder | Features::AimIndex;

    key += "|h" + std::to_string(static_cast<int>(d.hands)) + "|p" + std::to_string(d.priority) + "|a";
    for (const auto& a : cg.arms) key += std::to_string(a.first) + ":" + floatKey(a.second) + ",";
    key += "|g";
    for (int g : cg.gates) key += std::to_string(g) + ",";
    cg.key = d.shared ? std::string() : key;

    for (auto& in : cg.inst) {
      in.states.assign(cg.nodes.size(), PState{});
      in.outs.assign(cg.nodes.size(), POut{});
    }
    compiledOf[id] = static_cast<int>(gestures.size());
    gestures.push_back(std::move(cg));
  }

  void compile() {
    result = {};
    signals.clear();
    keepAlive.clear();
    signalIndex.clear();
    signalKeys.clear();
    conds.clear();
    condKeys.clear();
    gestures.clear();
    trackedSignalIdx.clear();
    compiledOf.assign(defs.size(), -1);
    std::unordered_map<std::string, int> groups;

    for (const Signal& s : trackedSignals)
      if (s.valid()) trackedSignalIdx.push_back(internSignal(s.node()));
    for (size_t i = 0; i < defs.size(); ++i) {
      if (defs[i].removed) continue;
      const bool okBefore = result.ok;
      result.ok = true;
      compileGesture(static_cast<GestureId>(i), defs[i].data, groups);
      result.ok = result.ok && okBefore;
    }
    groupCount = static_cast<int>(groups.size());

    // static conflict diagnostics
    for (size_t i = 0; i < gestures.size(); ++i)
      for (size_t j = i + 1; j < gestures.size(); ++j)
        if (!gestures[i].key.empty() && gestures[i].key == gestures[j].key)
          result.warnings.push_back("gestures '" + defs[gestures[i].id].data.name + "' and '" +
                                    defs[gestures[j].id].data.name +
                                    "' have identical triggers, context and priority; registration order decides");

    for (int h = 0; h < 2; ++h) {
      sigVal[h].assign(signals.size(), 0.f);
      sigConf[h].assign(signals.size(), 0.f);
      condRt[h].assign(conds.size(), ConditionRuntime{});
    }
    signalNeeded.assign(signals.size(), 0);
    condNeeded.assign(conds.size(), 0);
    owners.assign(static_cast<size_t>(groupCount), {});
    groupWinner.assign(static_cast<size_t>(groupCount) * 2, -1);
    candidates.clear();
    candidates.reserve(gestures.size() * 2 + 4);
    events.clear();
    // Per update at most a few events per gesture instance; accumulating mode holds several updates.
    const size_t perUpdate = gestures.size() * 2 * 4 + 16;
    events.reserve(cfg.eventCapacity > 0 ? cfg.eventCapacity : perUpdate * (cfg.autoClearEvents ? 1 : 8));
    computeNeeded();
    dirty = false;
  }

  void computeNeeded() {
    std::vector<char> prevCond = condNeeded;
    std::fill(signalNeeded.begin(), signalNeeded.end(), char(0));
    std::fill(condNeeded.begin(), condNeeded.end(), char(0));
    mask = 0;
    for (const CGesture& g : gestures) {
      if (!defs[g.id].enabled) continue;
      for (int c : g.conds) condNeeded[c] = 1;
      for (int s : g.signals) signalNeeded[s] = 1;
      mask |= g.mask;
    }
    for (int s : trackedSignalIdx) signalNeeded[s] = 1;
    for (int i = static_cast<int>(signals.size()) - 1; i >= 0; --i) {
      if (!signalNeeded[i]) continue;
      if (signals[i].a >= 0) signalNeeded[signals[i].a] = 1;
      if (signals[i].b >= 0) signalNeeded[signals[i].b] = 1;
      mask |= signals[i].mask;
    }
    for (size_t c = 0; c < conds.size(); ++c)
      if (condNeeded[c] && (c >= prevCond.size() || !prevCond[c])) {
        condRt[0][c].reset();
        condRt[1][c].reset();
      }
  }

  // ---- runtime ----------------------------------------------------------------------------------

  void evalSignals(int h) {
    const HandFeatures& f = trackers[h].features();
    float* val = sigVal[h].data();
    float* conf = sigConf[h].data();
    const float handConf = f.tracked ? f.confidence : 0.f;
    for (size_t i = 0; i < signals.size(); ++i) {
      if (!signalNeeded[i]) continue;
      const CSignal& s = signals[i];
      const auto fi = static_cast<size_t>(s.finger);
      float v = 0.f, c = handConf;
      switch (s.op) {
        case SignalOp::Constant:
          v = s.p[0];
          c = 1.f;
          break;
        case SignalOp::Pinch: {
          if (s.finger == Finger::Thumb) {
            c = 0.f;
            break;
          }
          const float d = f.pinchDistance[fi];
          v = 1.f - smoothstep(s.p[0], s.p[1], d);
          if (s.flag) {
            float other = 1e9f;
            for (size_t g = 1; g < kFingerCount; ++g)
              if (g != fi) other = std::min(other, f.pinchDistance[g]);
            v *= clamp01((other - d) / std::max(s.p[2], 1e-4f));
          }
          c = std::min(handConf, f.pinchConfidence[fi]);
          break;
        }
        case SignalOp::Curl: v = f.curl[fi]; break;
        case SignalOp::Extended: v = 1.f - smoothstep(s.p[0], s.p[1], f.curl[fi]); break;
        case SignalOp::Curled: v = smoothstep(s.p[0], s.p[1], f.curl[fi]); break;
        case SignalOp::Spread: v = smoothstep(s.p[0], s.p[1], f.spread[fi]); break;
        case SignalOp::PalmFacing: {
          Vec3 dir;
          bool ok = true;
          switch (s.target) {
            case Target::Head:
              ok = f.head.valid;
              dir = f.head.pose.position - f.palmPosition;
              break;
            case Target::Gaze:
              ok = f.head.gazeValid || f.head.valid;
              dir = f.head.gazeValid ? -f.head.gaze.direction : f.head.pose.position - f.palmPosition;
              break;
            case Target::Up: dir = {0.f, 1.f, 0.f}; break;
            case Target::Down: dir = {0.f, -1.f, 0.f}; break;
          }
          if (!ok) {
            c = 0.f;
            break;
          }
          const float angle = angleBetween(f.palmNormal, dir);
          v = 1.f - smoothstep(s.p[0] * 0.6f, s.p[0] * 1.4f, angle);
          break;
        }
        case SignalOp::InView: {
          if (!f.head.valid) {
            c = 0.f;
            break;
          }
          const float angle = angleBetween(f.head.forward, f.palmPosition - f.head.pose.position);
          v = 1.f - smoothstep(s.p[0] * 0.6f, s.p[0] * 1.4f, angle);
          break;
        }
        case SignalOp::Still:
          v = std::min(1.f - smoothstep(s.p[0] * 0.6f, s.p[0] * 1.4f, f.palmSpeed),
                       1.f - smoothstep(s.p[1] * 0.6f, s.p[1] * 1.4f, f.angularSpeed));
          break;
        case SignalOp::Grasping: v = f.graspScore; break;
        case SignalOp::Occupied:
          v = f.occupied ? 1.f : 0.f;
          c = 1.f;
          break;
        case SignalOp::Tracked:
          v = handConf;
          c = 1.f;
          break;
        case SignalOp::Custom: v = (s.fn && *s.fn) ? clamp01((*s.fn)(f)) : 0.f; break;
        case SignalOp::And:
        case SignalOp::Or: {
          const float va = val[s.a], vb = val[s.b];
          const bool pickA = s.op == SignalOp::And ? va < vb : va > vb;
          v = pickA ? va : vb;
          // The dominating input decides the confidence; ties take the more conservative one.
          c = va == vb ? std::min(conf[s.a], conf[s.b]) : (pickA ? conf[s.a] : conf[s.b]);
          break;
        }
        case SignalOp::Not:
          v = 1.f - val[s.a];
          c = conf[s.a];
          break;
      }
      val[i] = v;
      conf[i] = c;
    }
  }

  void updateConditions(int h) {
    for (size_t i = 0; i < conds.size(); ++i) {
      if (!condNeeded[i]) continue;
      const CCond& c = conds[i];
      condRt[h][i].update(sigVal[h][c.signal], sigConf[h][c.signal], time, c.params, c.onset, condSerial);
    }
  }

  static void leafOut(POut& o, const ConditionRuntime& cr, int cond) {
    o.token = cr.riseSerial;
    o.tokens.clear();
    o.tokens.add(cr.riseSerial);
    o.strength = cr.value;
    o.primary = cond;
    o.active = cr.active;
    o.cancelled = cr.cancelled;
  }

  void evalNode(CGesture& cg, int ni, int slot) {
    const PNode& pn = cg.nodes[ni];
    Instance& in = cg.inst[slot];
    PState& st = in.states[ni];
    POut& o = in.outs[ni];
    const double t = time;
    o.fired = false;
    o.cancelled = false;
    o.side = slot;
    switch (pn.op) {
      case PatternOp::Press: {
        const ConditionRuntime& cr = condRt[slot][pn.cond];
        leafOut(o, cr, pn.cond);
        o.fired = cr.pressed;
        o.onset = cr.pendingSince;
        o.quiet = cr.quietTime;
        o.progress = cr.active ? 1.f : 0.f;
        break;
      }
      case PatternOp::Release: {
        const ConditionRuntime& cr = condRt[slot][pn.cond];
        leafOut(o, cr, pn.cond);
        o.fired = cr.released;
        o.onset = cr.releaseStart;  // the release motion is the trigger
        o.quiet = cr.activeQuietTime;
        o.progress = 0.f;
        break;
      }
      case PatternOp::Click: {
        const ConditionRuntime& cr = condRt[slot][pn.cond];
        leafOut(o, cr, pn.cond);
        const Vec3 palm = trackers[slot].features().palmPosition;
        if (cr.pressed) {
          st.held = true;
          st.pressTime = t;
          st.pressPos = palm;
          st.travel = 0.f;
          st.pressOnset = cr.pendingSince;
          st.pressQuiet = cr.quietTime;
        }
        if (st.held) {
          st.travel = std::max(st.travel, distance(palm, st.pressPos));
          if (cr.cancelled || t - st.pressTime > pn.duration || st.travel > pn.travel) st.held = false;  // became a drag
        }
        if (cr.released && st.held) {
          st.held = false;
          o.fired = true;
          o.onset = st.pressOnset;
          o.quiet = st.pressQuiet;
        }
        o.progress = st.held && pn.duration > 0.f ? clamp01(static_cast<float>((t - st.pressTime) / pn.duration)) : 0.f;
        break;
      }
      case PatternOp::Active: {
        const ConditionRuntime& cr = condRt[slot][pn.cond];
        leafOut(o, cr, pn.cond);
        o.active = cr.active && (t - cr.activeSince) >= pn.duration - 1e-6;
        o.onset = cr.pendingSince;
        o.quiet = cr.quietTime;
        o.progress = cr.active ? (pn.duration > 0.f ? clamp01(static_cast<float>((t - cr.activeSince) / pn.duration)) : 1.f) : 0.f;
        break;
      }
      case PatternOp::Hold: {
        const ConditionRuntime& cr = condRt[slot][pn.cond];
        leafOut(o, cr, pn.cond);
        o.onset = cr.pendingSince;
        o.quiet = cr.quietTime;
        if (cr.active) {
          const double held = t - cr.activeSince;
          o.progress = pn.duration > 0.f ? clamp01(static_cast<float>(held / pn.duration)) : 1.f;
          if (!st.fired && held >= pn.duration - 1e-6) {
            st.fired = true;
            o.fired = true;
          }
        } else {
          st.fired = false;
          o.progress = 0.f;
        }
        break;
      }
      case PatternOp::Seq: {
        const int n = static_cast<int>(pn.children.size());
        if (st.step > 0 && ((pn.within > 0.f && t - st.t0 > pn.within) || (pn.maxGap > 0.f && t - st.lastStep > pn.maxGap)))
          st.step = 0;
        const POut& c = in.outs[pn.children[st.step]];
        const POut& first = in.outs[pn.children[0]];
        if (c.fired) {
          if (st.step == 0) {
            st.t0 = t;
            st.acc.clear();
            st.accOnset = c.onset;
          }
          st.acc.addAll(c.tokens);
          st.lastStep = t;
          if (++st.step == n) {
            o.fired = true;
            o.tokens = st.acc;
            o.token = c.token;
            o.onset = st.accOnset;
            o.quiet = c.quiet;
            o.strength = c.strength;
            o.primary = c.primary;
            st.step = 0;
          }
        } else if (st.step > 0 && first.fired) {
          st.step = 1;  // restart from the first step
          st.t0 = t;
          st.acc = first.tokens;
          st.accOnset = first.onset;
          st.lastStep = t;
        }
        o.progress = static_cast<float>(st.step) / static_cast<float>(n);
        if (o.fired) o.progress = 1.f;
        o.active = false;
        break;
      }
      case PatternOp::Repeat: {
        if (st.count > 0 && pn.within > 0.f && t - st.t0 > pn.within) st.count = 0;
        const POut& c = in.outs[pn.children[0]];
        if (c.fired) {
          if (st.count == 0) {
            st.t0 = t;
            st.acc.clear();
            st.accOnset = c.onset;
          }
          st.acc.addAll(c.tokens);
          if (++st.count >= pn.count) {
            o.fired = true;
            o.tokens = st.acc;
            o.token = c.token;
            o.onset = st.accOnset;
            o.quiet = c.quiet;
            o.strength = c.strength;
            o.primary = c.primary;
            st.count = 0;
          }
        }
        o.progress = o.fired ? 1.f : static_cast<float>(st.count) / static_cast<float>(std::max(pn.count, 1));
        break;
      }
      case PatternOp::Both:
      case PatternOp::Chord:
        break;  // evaluated by evalChord
    }
  }

  void evalChord(CGesture& cg) {
    const int ni = static_cast<int>(cg.nodes.size()) - 1;
    const PNode& pn = cg.nodes[ni];
    PState& st = cg.inst[0].states[ni];
    POut& o = cg.inst[0].outs[ni];
    const int cl = pn.children[0];
    const int cr = pn.op == PatternOp::Chord ? pn.children[1] : pn.children[0];
    const POut* side[2] = {&cg.inst[0].outs[cl], &cg.inst[1].outs[cr]};
    const double t = time;
    o.fired = false;
    o.cancelled = false;
    if (!pn.continuous) {
      int completed = -1;
      for (int s = 0; s < 2; ++s) {
        if (!side[s]->fired) continue;
        st.sideFire[s] = t;
        st.sideTokens[s] = side[s]->tokens;
        st.sideToken[s] = side[s]->token;
        st.sideStrength[s] = side[s]->strength;
        st.sideOnset[s] = side[s]->onset;
        st.sideQuiet[s] = side[s]->quiet;
        completed = s;
      }
      if (completed >= 0 && std::fabs(st.sideFire[0] - st.sideFire[1]) <= pn.sync + 1e-6) {
        o.fired = true;
        o.side = completed;
        o.tokens = st.sideTokens[0];
        o.tokens.addAll(st.sideTokens[1]);
        o.token = st.sideToken[completed];
        o.onset = std::min(st.sideOnset[0], st.sideOnset[1]);
        o.quiet = st.sideQuiet[completed];
        o.strength = std::min(st.sideStrength[0], st.sideStrength[1]);
        o.primary = -1;
        st.sideFire[0] = st.sideFire[1] = -1e9;
      }
      o.progress = o.fired ? 1.f : 0.5f * ((t - st.sideFire[0] <= pn.sync) + (t - st.sideFire[1] <= pn.sync));
    } else {
      for (int s = 0; s < 2; ++s) {
        const bool a = side[s]->active;
        if (a && !st.sideWasActive[s]) st.sideActiveSince[s] = t;
        st.sideWasActive[s] = a;
      }
      const bool bothActive = side[0]->active && side[1]->active;
      if (!st.begun) {
        if (bothActive && std::fabs(st.sideActiveSince[0] - st.sideActiveSince[1]) <= pn.sync + 1e-6) st.begun = true;
      } else if (!bothActive) {
        st.begun = false;
        o.cancelled = side[0]->cancelled || side[1]->cancelled;
      }
      o.active = st.begun;
      const int later = st.sideActiveSince[1] >= st.sideActiveSince[0] ? 1 : 0;
      o.side = later;
      o.token = side[later]->token;
      o.tokens = side[0]->tokens;
      o.tokens.addAll(side[1]->tokens);
      o.onset = std::max(side[0]->onset, side[1]->onset);
      o.strength = std::min(side[0]->strength, side[1]->strength);
      o.progress = o.active ? 1.f : 0.f;
      o.primary = -1;
    }
  }

  bool condActive(int c, int slot, bool chord) const {
    if (chord) return condRt[0][c].active && condRt[1][c].active;
    return condRt[slot][c].active;
  }

  // Arm conditions must have been held for minHold *before* the trigger's onset.
  bool armedAt(const CGesture& cg, int slot, double onset) const {
    for (const auto& a : cg.arms) {
      for (int s = 0; s < 2; ++s) {
        if (!cg.chord && s != slot) continue;
        if (!condRt[s][a.first].activeAt(onset, a.second)) return false;
      }
    }
    return true;
  }

  bool armedNow(const CGesture& cg, int slot) const {
    for (const auto& a : cg.arms)
      for (int s = 0; s < 2; ++s) {
        if (!cg.chord && s != slot) continue;
        const ConditionRuntime& cr = condRt[s][a.first];
        if (!cr.active || time - cr.activeSince < a.second - 1e-6) return false;
      }
    return true;
  }

  static bool outranks(const CGesture& a, const CGesture& b) {
    if (a.priority != b.priority) return a.priority > b.priority;
    if (a.specificity != b.specificity) return a.specificity > b.specificity;
    return a.order < b.order;
  }

  bool rayFor(const CGesture& cg, int slot, bool discrete, double quiet, Ray& out) const {
    if (cg.aim == Aim::None) return false;
    if (discrete && cg.stabilize == Stabilize::RewindToOnset) {
      const double earliest = time - static_cast<double>(cfg.rewindMax);
      const double from = std::max(std::min(quiet, time), earliest);
      return trackers[slot].stableRayBefore(cg.aim, from, earliest, cfg.rewindStableSpeed, out);
    }
    return trackers[slot].currentRay(cg.aim, out);
  }

  void emit(const CGesture& cg, int slot, Phase phase, float strength, float progress, bool rayValid, const Ray& ray,
            double t) {
    if (events.size() >= events.capacity()) {
      ++dropped;
      return;
    }
    Event e;
    e.id = cg.id;
    e.hand = static_cast<Hand>(slot);
    e.bothHands = cg.chord;
    e.phase = phase;
    e.time = t;
    e.strength = strength;
    e.progress = progress;
    e.rayValid = rayValid;
    e.ray = ray;
    e.serial = ++eventSerial;
    events.push_back(e);
  }

  void processInstance(int gi, int slot) {
    CGesture& cg = gestures[gi];
    Instance& in = cg.inst[slot];
    const POut& root = cg.chord ? cg.inst[0].outs.back() : in.outs.back();
    const int evSlot = cg.chord ? root.side : slot;

    // arming / affordance
    if (!cg.arms.empty()) {
      const bool armed = armedNow(cg, slot);
      if (armed != in.armed) {
        in.armed = armed;
        in.since = time;
        emit(cg, evSlot, armed ? Phase::Armed : Phase::Disarmed, root.strength, root.progress, false, Ray{}, time);
      }
    }
    bool gatesOk = true;
    for (int g : cg.gates) gatesOk = gatesOk && condActive(g, slot, cg.chord);
    bool cancelNow = false;
    for (int c : cg.cancels) cancelNow = cancelNow || condActive(c, slot, false) || (cg.chord && condActive(c, 1, false));
    if (cancelNow) in.pending.valid = false;

    in.progress = root.progress;
    in.strength = root.strength;

    if (cg.continuous) {
      if (in.active) {
        if (!root.active || root.cancelled || cancelNow) {
          in.rayValid = rayFor(cg, evSlot, false, time, in.ray);
          emit(cg, evSlot, (root.cancelled || cancelNow) ? Phase::Cancelled : Phase::Ended, root.strength, root.progress,
               in.rayValid, in.ray, time);
          in.active = false;
          in.since = time;
          if (cancelNow) in.blockedToken = in.activeToken;
          if (cg.group >= 0 && owners[cg.group][slot].g == gi) owners[cg.group][slot] = {};
        } else {
          in.rayValid = rayFor(cg, evSlot, false, time, in.ray);
          if (cg.emitUpdates) emit(cg, evSlot, Phase::Updated, root.strength, root.progress, in.rayValid, in.ray, time);
        }
      } else if (root.active && root.token != in.blockedToken && !cancelNow && gatesOk && armedAt(cg, slot, root.onset)) {
        addCandidate(gi, slot, true, root);
      }
    } else if (root.fired && gatesOk && !cancelNow && armedAt(cg, slot, root.onset)) {
      addCandidate(gi, slot, false, root);
    }
  }

  void addCandidate(int gi, int slot, bool begin, const POut& root) {
    const CGesture& cg = gestures[gi];
    if (cg.group >= 0) {
      const GroupOwner& ow = owners[cg.group][slot];
      if (ow.g >= 0 && ow.g != gi) {
        const bool preempt = begin && ow.continuous && outranks(cg, gestures[ow.g]);
        if (!preempt) return;
      }
    }
    if (candidates.size() >= candidates.capacity()) return;
    Candidate c;
    c.g = gi;
    c.slot = cg.chord ? root.side : slot;
    c.begin = begin;
    c.token = root.token;
    c.tokens = root.tokens;
    c.time = time;
    c.quiet = root.quiet;
    c.strength = root.strength;
    c.progress = root.progress;
    c.primary = root.primary;
    candidates.push_back(c);
  }

  void resolve() {
    std::sort(candidates.begin(), candidates.end(), [&](const Candidate& a, const Candidate& b) {
      if (a.token != b.token) return a.token < b.token;
      return outranks(gestures[a.g], gestures[b.g]);
    });
    // One physical action (token) -> one non-shared gesture.
    for (size_t i = 0; i < candidates.size();) {
      size_t j = i;
      bool taken = false;
      while (j < candidates.size() && candidates[j].token == candidates[i].token) {
        Candidate& c = candidates[j];
        const CGesture& cg = gestures[c.g];
        if (cg.shared || c.token == 0) {
          c.won = true;
        } else if (!taken) {
          c.won = true;
          taken = true;
        }
        ++j;
      }
      i = j;
    }
    // Exclusive groups: at most one winner per (group, hand) per frame.
    std::fill(groupWinner.begin(), groupWinner.end(), -1);
    for (size_t i = 0; i < candidates.size(); ++i) {
      Candidate& c = candidates[i];
      const CGesture& cg = gestures[c.g];
      if (!c.won || cg.group < 0) continue;
      int& w = groupWinner[static_cast<size_t>(cg.group) * 2 + c.slot];
      if (w < 0) {
        w = static_cast<int>(i);
      } else if (outranks(cg, gestures[candidates[w].g])) {
        candidates[w].won = false;
        w = static_cast<int>(i);
      } else {
        c.won = false;
      }
    }
    // Commit, in registration order for determinism.
    std::sort(candidates.begin(), candidates.end(), [&](const Candidate& a, const Candidate& b) {
      return gestures[a.g].order < gestures[b.g].order;
    });
    for (Candidate& c : candidates) {
      CGesture& cg = gestures[c.g];
      Instance& in = cg.inst[cg.chord ? 0 : c.slot];
      if (!c.won) {
        if (c.begin) in.blockedToken = c.token;
        continue;
      }
      Ray ray;
      const bool rayValid = rayFor(cg, c.slot, !c.begin, c.quiet, ray);
      if (c.begin) {
        if (cg.group >= 0) {
          GroupOwner& ow = owners[cg.group][c.slot];
          if (ow.g >= 0 && ow.g != c.g && ow.continuous) {
            CGesture& og = gestures[ow.g];
            Instance& oi = og.inst[c.slot];
            if (oi.active) {
              emit(og, c.slot, Phase::Cancelled, oi.strength, oi.progress, oi.rayValid, oi.ray, time);
              oi.active = false;
              oi.since = time;
              oi.blockedToken = oi.activeToken;
            }
          }
          ow = {c.g, c.primary, true};
        }
        in.active = true;
        in.activeToken = c.token;
        in.since = time;
        in.rayValid = rayValid;
        in.ray = ray;
        emit(cg, c.slot, Phase::Began, c.strength, c.progress, rayValid, ray, time);
      } else {
        if (cg.group >= 0) owners[cg.group][c.slot] = {c.g, c.primary, false};
        in.since = time;
        in.rayValid = rayValid;
        in.ray = ray;
        if (cg.resolve == Resolve::Wait) {
          in.pending.valid = true;
          in.pending.deadline = time + cg.waitWindow;
          in.pending.token = c.token;
          in.pending.event = Event{};
          in.pending.event.id = cg.id;
          in.pending.event.hand = static_cast<Hand>(c.slot);
          in.pending.event.bothHands = cg.chord;
          in.pending.event.phase = Phase::Triggered;
          in.pending.event.time = time;
          in.pending.event.strength = c.strength;
          in.pending.event.progress = c.progress;
          in.pending.event.rayValid = rayValid;
          in.pending.event.ray = ray;
        } else {
          emit(cg, c.slot, Phase::Triggered, c.strength, c.progress, rayValid, ray, time);
        }
      }
    }
    // Waiting gestures are dropped if another gesture consumed their physical action.
    for (const Candidate& c : candidates) {
      if (!c.won || gestures[c.g].shared) continue;
      for (CGesture& g : gestures)
        for (Instance& in : g.inst)
          if (in.pending.valid && &g != &gestures[c.g] && c.tokens.contains(in.pending.token)) in.pending.valid = false;
    }
    for (CGesture& g : gestures)
      for (int s = 0; s < 2; ++s) {
        Instance& in = g.inst[s];
        if (!in.pending.valid || time < in.pending.deadline - 1e-6) continue;
        in.pending.valid = false;
        if (events.size() >= events.capacity()) {
          ++dropped;
          continue;
        }
        Event e = in.pending.event;
        e.serial = ++eventSerial;
        events.push_back(e);
      }
  }

  // A null hand/head means "no new data in this call": the last state is held until fresh data
  // arrives, or until nothing has arrived for cfg.inputTimeout (then the hand counts as untracked).
  // Hands can therefore be fed in separate calls, and repeated samples (hand tracking running
  // slower than the frame loop) are not re-processed.
  void update(double t, const HandSample* left, const HandSample* right, const HeadSample* hs) {
    if (dirty) compile();
    if (cfg.autoClearEvents) events.clear();
    const size_t firstNewEvent = events.size();
    // Hands arriving in separate calls may carry slightly older timestamps than the last call:
    // clamp rather than drop, so time stays monotonic and no data is lost.
    if (time >= 0.0 && t < time) t = time;
    const bool sameTime = t == time;
    time = t;

    if (hs) {
      head = HeadInfo{};
      head.valid = true;
      head.pose = hs->pose;
      head.forward = normalize(rotate(hs->pose.orientation, Vec3{0.f, 0.f, -1.f}));
      if (hs->gaze) {
        head.gazeValid = true;
        head.gaze = *hs->gaze;
        head.gaze.direction = normalize(head.gaze.direction);
      }
      lastHeadTime = t;
    } else if (head.valid && t - lastHeadTime > cfg.inputTimeout) {
      head = HeadInfo{};
    }

    const HandSample* samples[2] = {left, right};
    bool anyProcessed = false;
    for (int h = 0; h < 2; ++h) {
      const HandSample* s = samples[h];
      const double sampleTime = s && s->time > 0.0 ? s->time : t;
      const bool fresh = s != nullptr && sampleTime > lastSampleTime[h];
      const HandFeatures& f = trackers[h].features();
      const bool stale = !fresh && f.tracked && t - lastInputTime[h] > cfg.inputTimeout;
      if (fresh) {
        lastSampleTime[h] = sampleTime;
        lastInputTime[h] = t;
        trackers[h].update(s, t, head, mask, cfg);
      } else if (stale) {
        trackers[h].update(nullptr, t, head, mask, cfg);  // stopped sending: now untracked
      } else if (f.tracked) {
        // No new data: hold. Clear one-frame edges so nothing fires twice from the same data.
        for (ConditionRuntime& cr : condRt[h]) cr.pressed = cr.released = cr.cancelled = false;
        continue;
      }
      // Fresh data, newly stale, or an untracked hand (its conditions need time to run out their grace).
      trackers[h].features().occupied = occupied[h];
      evalSignals(h);
      updateConditions(h);
      anyProcessed = anyProcessed || fresh || stale;
    }
    if (sameTime && !anyProcessed) return;  // repeated call with nothing new
    // Discrete owners of exclusive groups release once their activation ends.
    for (auto& perHand : owners)
      for (int s = 0; s < 2; ++s) {
        GroupOwner& ow = perHand[s];
        if (ow.g >= 0 && !ow.continuous && (ow.primary < 0 || !condRt[s][ow.primary].active)) ow = {};
      }

    candidates.clear();
    for (int gi = 0; gi < static_cast<int>(gestures.size()); ++gi) {
      CGesture& cg = gestures[gi];
      if (!defs[cg.id].enabled) continue;
      const int n = static_cast<int>(cg.nodes.size());
      if (cg.chord) {
        for (int s = 0; s < 2; ++s)
          for (int ni = 0; ni < n - 1; ++ni) evalNode(cg, ni, s);
        evalChord(cg);
        processInstance(gi, 0);
      } else {
        for (int s = 0; s < 2; ++s) {
          if ((cg.hands == Hands::Left && s != 0) || (cg.hands == Hands::Right && s != 1)) continue;
          for (int ni = 0; ni < n; ++ni) evalNode(cg, ni, s);
          processInstance(gi, s);
        }
      }
    }
    resolve();

    for (size_t i = firstNewEvent; i < events.size(); ++i) {
      const Event e = events[i];  // copy: a callback may re-enter add()/setEnabled()
      if (e.id < defs.size() && defs[e.id].data.callback) defs[e.id].data.callback(e);
    }
  }

  void resetRuntime() {
    for (auto& t : trackers) t.reset();
    for (int h = 0; h < 2; ++h) {
      for (auto& c : condRt[h]) c.reset();
      std::fill(sigVal[h].begin(), sigVal[h].end(), 0.f);
      std::fill(sigConf[h].begin(), sigConf[h].end(), 0.f);
    }
    for (auto& g : gestures)
      for (auto& in : g.inst) {
        const size_t n = g.nodes.size();
        in = Instance{};
        in.states.assign(n, PState{});
        in.outs.assign(n, POut{});
      }
    for (auto& o : owners) o = {};
    events.clear();
    time = -1.0;
    head = HeadInfo{};
    lastHeadTime = -1e9;
    for (int h = 0; h < 2; ++h) lastSampleTime[h] = lastInputTime[h] = -1e9;
  }
};

// =====================================================================================================

Composer::Composer(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
Composer::~Composer() = default;
Composer::Composer(Composer&&) noexcept = default;
Composer& Composer::operator=(Composer&&) noexcept = default;

GestureId Composer::add(const GestureDesc& desc) {
  Impl::Def d;
  d.data = desc.data();
  impl_->defs.push_back(std::move(d));
  impl_->dirty = true;
  return static_cast<GestureId>(impl_->defs.size() - 1);
}

void Composer::remove(GestureId id) {
  if (id >= impl_->defs.size()) return;
  impl_->defs[id].removed = true;
  impl_->defs[id].data.callback = nullptr;
  impl_->dirty = true;
}

void Composer::track(const Signal& s) {
  impl_->trackedSignals.push_back(s);
  impl_->dirty = true;
}

const CompileResult& Composer::compile() {
  if (impl_->dirty) {
    impl_->compile();
    impl_->resetRuntime();
  }
  return impl_->result;
}

void Composer::update(const HandSample* left, const HandSample* right, const HeadSample* head) {
  double t = -1.0;
  if (left) t = std::max(t, left->time);
  if (right) t = std::max(t, right->time);
  if (head) t = std::max(t, head->time);
  if (t < 0.0) {
    if (impl_->cfg.autoClearEvents) impl_->events.clear();
    return;
  }
  update(t, left, right, head);
}

void Composer::update(double time, const HandSample* left, const HandSample* right, const HeadSample* head) {
  if (impl_->dirty) compile();
  impl_->update(time, left, right, head);
}

Span<Event> Composer::events() const { return {impl_->events.data(), impl_->events.size()}; }

void Composer::clearEvents() { impl_->events.clear(); }

GestureState Composer::state(GestureId id, Hand hand) const {
  GestureState s;
  if (id >= impl_->compiledOf.size() || impl_->compiledOf[id] < 0) return s;
  const CGesture& g = impl_->gestures[impl_->compiledOf[id]];
  const Instance& in = g.inst[g.chord ? 0 : static_cast<int>(hand)];
  s.armed = g.arms.empty() ? false : in.armed;
  s.active = in.active;
  s.progress = in.progress;
  s.strength = in.strength;
  s.rayValid = in.rayValid;
  s.ray = in.ray;
  s.since = in.since;
  if (g.aim != Aim::None && !in.active) s.rayValid = impl_->trackers[static_cast<int>(hand)].currentRay(g.aim, s.ray);
  return s;
}

SignalValue Composer::signal(const Signal& s, Hand hand) const {
  if (!s.valid()) return {};
  auto it = impl_->signalIndex.find(s.node().get());
  if (it == impl_->signalIndex.end()) return {};
  const int h = static_cast<int>(hand);
  return {impl_->sigVal[h][it->second], impl_->sigConf[h][it->second]};
}

const HandFeatures& Composer::features(Hand hand) const { return impl_->trackers[static_cast<int>(hand)].features(); }

const char* Composer::name(GestureId id) const {
  return id < impl_->defs.size() ? impl_->defs[id].data.name.c_str() : "";
}

GestureId Composer::find(const std::string& n) const {
  for (size_t i = 0; i < impl_->defs.size(); ++i)
    if (!impl_->defs[i].removed && impl_->defs[i].data.name == n) return static_cast<GestureId>(i);
  return kInvalidGesture;
}

void Composer::setEnabled(GestureId id, bool on) {
  if (id >= impl_->defs.size() || impl_->defs[id].enabled == on) return;
  impl_->defs[id].enabled = on;
  if (impl_->dirty || id >= impl_->compiledOf.size() || impl_->compiledOf[id] < 0) return;
  CGesture& g = impl_->gestures[impl_->compiledOf[id]];
  for (int s = 0; s < 2; ++s) {
    Instance& in = g.inst[s];
    in.states.assign(g.nodes.size(), PState{});
    in.outs.assign(g.nodes.size(), POut{});  // same size: no reallocation
    in.active = in.armed = false;
    in.pending.valid = false;
    if (g.group >= 0 && impl_->owners[g.group][s].g == impl_->compiledOf[id]) impl_->owners[g.group][s] = {};
  }
  impl_->computeNeeded();
}

bool Composer::enabled(GestureId id) const { return id < impl_->defs.size() && impl_->defs[id].enabled; }

void Composer::setHandOccupied(Hand hand, bool occ) { impl_->occupied[static_cast<int>(hand)] = occ; }

void Composer::reset() { impl_->resetRuntime(); }

uint32_t Composer::droppedEvents() const { return impl_->dropped; }

FeatureMask Composer::featureMask() const { return impl_->mask; }

}  // namespace gc
