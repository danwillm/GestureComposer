#include "sim.h"
#include "test_framework.h"

using namespace gc;
using namespace gc::gestures;
using namespace std::chrono_literals;
using gc::testing::HandPose;
using gctest::Sim;

namespace {
const int L = 0, R = 1;

void pinchCycle(Sim& sim, int h, Finger f, double closeTime = 0.08, double holdTime = 0.15, double openTime = 0.08,
                double restTime = 0.25) {
  sim.moveTo(h, HandPose::pinch(f, 1.f), closeTime);
  sim.run(holdTime);
  sim.moveTo(h, HandPose::relaxed(), openTime);
  sim.run(restTime);
}
}  // namespace

TEST_CASE("press: one event per pinch, even with tracking jitter") {
  Composer c;
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
  Sim sim(c);
  sim.noise = 0.0015f;
  sim.run(0.5);
  for (int i = 0; i < 6; ++i) pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(sel, Phase::Triggered, R), 6);
  CHECK_EQ(sim.count(sel, Phase::Triggered, L), 0);
}

TEST_CASE("press: hovering at the threshold does not chatter") {
  Composer c;
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
  Sim sim(c);
  sim.noise = 0.002f;
  sim.run(0.5);
  // Close to ~1.8 cm surface distance (around the enter threshold) and dither there.
  HandPose near = HandPose::pinch(Finger::Index, 1.f);
  near.pinchGap = 0.018f;
  sim.moveTo(R, near, 0.3);
  sim.run(1.5);
  CHECK(sim.count(sel, Phase::Triggered) <= 1);
}

TEST_CASE("onset: a hand that appears already pinched does not fire") {
  Composer c;
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
  Sim sim(c);
  sim.hand[R].present = false;
  sim.run(0.3);
  sim.hand[R].pose = HandPose::pinch(Finger::Index);
  sim.hand[R].present = true;
  sim.run(0.6);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 0);
  sim.moveTo(R, HandPose::relaxed(), 0.1);
  sim.run(0.2);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 1);
}

TEST_CASE("arbitration: palm-to-head pinch opens the menu and suppresses select") {
  Composer c;
  auto indexPinch = pinch(Finger::Index);
  auto relaxed = !grasping() && !occupied();
  auto palmToHead = palmFacing(Target::Head) && inView(40.f) && still() &&
                    extended(Finger::Middle, Finger::Ring, Finger::Little);
  GestureId sel = c.add(gesture("select").trigger(press(indexPinch)).gate(relaxed));
  GestureId menu = c.add(gesture("menu").trigger(press(indexPinch)).arm(palmToHead, 250ms).gate(relaxed));
  CHECK(c.compile().ok);
  Sim sim(c);
  sim.hand[R].wrist = {0.10f, 1.40f, -0.30f};
  sim.run(0.5);

  // Palm away: select only.
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 1);
  CHECK_EQ(sim.count(menu, Phase::Triggered), 0);

  // Palm to head, hold, then pinch: menu only.
  sim.palmToHead(R);
  sim.run(0.4);
  CHECK(c.state(menu, Hand::Right).armed);
  CHECK_EQ(sim.count(menu, Phase::Armed), 1);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(menu, Phase::Triggered), 1);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 1);
}

TEST_CASE("hands: a left-only menu never fires on the right hand, which selects instead") {
  Composer c;
  auto indexPinch = pinch(Finger::Index);
  auto palmToHead = palmFacing(Target::Head) && extended(Finger::Middle, Finger::Ring, Finger::Little);
  GestureId sel = c.add(gesture("select").trigger(press(indexPinch)));
  GestureId menu = c.add(gesture("menu").hands(Hands::Left).trigger(press(indexPinch)).arm(palmToHead, 250ms));
  Sim sim(c);
  sim.hand[L].wrist = {-0.10f, 1.40f, -0.30f};
  sim.hand[R].wrist = {0.10f, 1.40f, -0.30f};
  sim.run(0.5);

  // Right hand: palm to head + pinch is just a select.
  sim.palmToHead(R);
  sim.run(0.4);
  CHECK(!c.state(menu, Hand::Right).armed);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(menu, Phase::Triggered), 0);
  CHECK_EQ(sim.count(menu, Phase::Armed), 0);
  CHECK_EQ(sim.count(sel, Phase::Triggered, R), 1);

  // Left hand: the same motion opens the menu, and select does not also fire.
  sim.palmToHead(L);
  sim.run(0.4);
  CHECK(c.state(menu, Hand::Left).armed);
  pinchCycle(sim, L, Finger::Index);
  CHECK_EQ(sim.count(menu, Phase::Triggered, L), 1);
  CHECK_EQ(sim.count(sel, Phase::Triggered, L), 0);
  for (const Event& e : sim.log)
    if (e.id == menu) CHECK(e.hand == Hand::Left);
}

TEST_CASE("full gesture set: left-hand system click fires inside an exclusive group with held pinches") {
  for (bool withGroup : {false, true}) {
    Composer c;
    Signal sIndexPinch = pinch(Finger::Index);
    auto relaxed = !grasping() && !occupied();
    auto palmToHead = palmFacing(Target::Head) && inView(40.f) && still() &&
                      extended(Finger::Middle, Finger::Ring, Finger::Little);
    auto pointPose = extended(Finger::Index) && curled(Finger::Middle, Finger::Ring, Finger::Little);
    const std::string group = withGroup ? "allGestures" : "";
    auto heldPinch = [&](const char* name, const Signal& s) {
      return c.add(gesture(name).trigger(active(Condition(s).onset(true))).gate(relaxed).exclusive(group).updates(false));
    };
    GestureId ip = heldPinch("index_pinch", sIndexPinch);
    heldPinch("middle_pinch", pinch(Finger::Middle));
    c.add(gesture("grab")
              .trigger(active(Condition(curled(Finger::Index, Finger::Middle, Finger::Ring, Finger::Little))
                                  .thresholds(0.6f, 0.3f).releaseDwell(80ms).onset(true)))
              .exclusive(group).updates(false));
    c.add(gesture("index_point").trigger(active(pointPose).minDuration(100ms)).exclusive(group).updates(false));
    GestureId sc = c.add(gesture("system_click").hands(Hands::Left).trigger(press(sIndexPinch))
                             .arm(palmToHead, 250ms).gate(relaxed).exclusive(group));
    Sim sim(c);
    sim.hand[L].wrist = {-0.10f, 1.40f, -0.30f};
    sim.run(0.5);
    sim.palmToHead(L);
    sim.run(0.4);
    CHECK(c.state(sc, Hand::Left).armed);
    pinchCycle(sim, L, Finger::Index);
    CHECK_EQ(sim.count(sc, Phase::Triggered, L), 1);
    CHECK_EQ(sim.count(ip, Phase::Began, L), 0);  // the menu pinch is not also an index pinch
    // Palm down: the same pinch is a normal held index pinch.
    sim.palmDown(L);
    sim.run(0.4);
    pinchCycle(sim, L, Finger::Index);
    CHECK_EQ(sim.count(ip, Phase::Began, L), 1);
    CHECK_EQ(sim.count(sc, Phase::Triggered, L), 1);
  }
}

TEST_CASE("arm-before-trigger: palm arriving together with the pinch does not open the menu") {
  Composer c;
  auto indexPinch = pinch(Finger::Index);
  GestureId menu = c.add(gesture("menu").trigger(press(indexPinch)).arm(palmFacing(Target::Head), 250ms));
  Sim sim(c);
  sim.hand[R].wrist = {0.10f, 1.40f, -0.30f};
  sim.run(0.5);
  sim.palmToHead(R);
  sim.run(0.1);  // only 100 ms of palm-to-head before pinching
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(menu, Phase::Triggered), 0);
}

TEST_CASE("steering wheel: gripping while the palm sweeps past the head never fires") {
  Composer c;
  auto indexPinch = pinch(Finger::Index);
  auto relaxed = !grasping() && !occupied();
  auto palmToHead = palmFacing(Target::Head) && inView(40.f) && still() &&
                    extended(Finger::Middle, Finger::Ring, Finger::Little);
  GestureId sel = c.add(gesture("select").trigger(press(indexPinch)).gate(relaxed));
  GestureId menu = c.add(gesture("menu").trigger(press(indexPinch)).arm(palmToHead, 250ms).gate(relaxed));
  // The naive version: rising edge of a frame-by-frame AND, no onset, no grasp gate, no arming order.
  GestureId naive = c.add(gesture("naive")
                              .trigger(press(Condition(indexPinch && palmFacing(Target::Head)).onset(false)))
                              .shared());
  Sim sim(c);
  sim.noise = 0.0015f;
  sim.hand[R].wrist = {0.20f, 1.25f, -0.40f};
  sim.hand[R].pose = HandPose::grip();
  sim.run(0.5);
  // Turn the wheel: rotate the hand back and forth through "palm facing head" for 6 seconds,
  // loosening and re-tightening the grip as drivers do.
  for (int k = 0; k < 540; ++k) {
    const float a = std::sin(static_cast<float>(k) * 0.035f) * 1.6f;
    const Vec3 toHead = normalize(sim.head.pose.position - sim.hand[R].wrist);
    const Vec3 side = normalize(cross(toHead, Vec3{0.f, 1.f, 0.f}));
    sim.hand[R].palmNormal = normalize(toHead * std::cos(a) + side * std::sin(a));
    sim.hand[R].fingerDir = normalize(cross(sim.hand[R].palmNormal, side));
    HandPose g = HandPose::grip();
    g.pinchAmount = 0.7f + 0.3f * std::sin(static_cast<float>(k) * 0.11f);  // loosen / re-tighten
    sim.hand[R].pose = g;
    sim.step();
  }
  CHECK_EQ(sim.count(sel, Phase::Triggered), 0);
  CHECK_EQ(sim.count(menu, Phase::Triggered), 0);
  CHECK(sim.count(naive, Phase::Triggered) >= 1);  // confirms the scenario reproduces the classic misfire
}

TEST_CASE("app occupancy blocks gated gestures") {
  Composer c;
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))).gate(!occupied()));
  Sim sim(c);
  sim.run(0.3);
  c.setHandOccupied(Hand::Right, true);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 0);
  c.setHandOccupied(Hand::Right, false);
  sim.run(0.2);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 1);
}

TEST_CASE("thumb-to-finger: each pinch fires exactly its own finger") {
  Composer c;
  GestureId ids[5] = {};
  for (Finger f : {Finger::Index, Finger::Middle, Finger::Ring, Finger::Little})
    ids[static_cast<int>(f)] = c.add(gesture(std::string("tool.") + toString(f))
                                         .hands(Hands::Right)
                                         .trigger(press(pinch(f)))
                                         .exclusive("thumb_pinch"));
  Sim sim(c);
  sim.noise = 0.001f;
  sim.run(0.4);
  for (Finger f : {Finger::Index, Finger::Middle, Finger::Ring, Finger::Little, Finger::Middle, Finger::Index}) {
    sim.clearLog();
    pinchCycle(sim, R, f);
    for (int g = 1; g < 5; ++g) CHECK_EQ(sim.count(ids[g], Phase::Triggered), g == static_cast<int>(f) ? 1 : 0);
  }
}

TEST_CASE("exclusive group: sliding from one finger to the next requires a release") {
  Composer c;
  GestureId a = c.add(gesture("a").trigger(press(pinch(Finger::Index, PinchOptions().exclusivity(false)))).exclusive("g"));
  GestureId b = c.add(gesture("b").trigger(press(pinch(Finger::Middle, PinchOptions().exclusivity(false)))).exclusive("g"));
  Sim sim(c);
  sim.run(0.4);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  sim.run(0.15);
  // Slide the thumb across to the middle finger without opening.
  sim.moveTo(R, HandPose::pinch(Finger::Middle), 0.15);
  sim.run(0.2);
  CHECK_EQ(sim.count(a, Phase::Triggered), 1);
  CHECK_EQ(sim.count(b, Phase::Triggered), 0);
}

TEST_CASE("tracking dropout: short gaps are bridged, long gaps cancel") {
  Composer c;
  GestureId drag = c.add(gesture("drag").trigger(active(pinch(Finger::Index))).updates(false));
  Sim sim(c);
  sim.run(0.3);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  sim.run(0.2);
  CHECK_EQ(sim.count(drag, Phase::Began), 1);
  sim.hand[R].present = false;
  sim.run(0.06);
  sim.hand[R].present = true;
  sim.run(0.2);
  CHECK_EQ(sim.count(drag, Phase::Ended) + sim.count(drag, Phase::Cancelled), 0);
  sim.hand[R].present = false;
  sim.run(0.4);
  CHECK_EQ(sim.count(drag, Phase::Cancelled), 1);
  CHECK_EQ(sim.count(drag, Phase::Ended), 0);
}

namespace {
// Drives the hands through separate update() calls, as when each hand's data arrives on its own.
// offset: how far the right hand's timestamps lag the left's (0 = same timestamp in both calls).
struct SplitSim : Sim {
  double offset = 0.0;
  using Sim::Sim;
  void step() {
    t += dt;
    head.time = t;
    HandSample l = sample(0);
    c.update(t, hand[0].present ? &l : nullptr, nullptr, &head);
    for (const Event& e : c.events()) log.push_back(e);
    const double tl = t;
    t = tl - offset;  // the right hand was sampled slightly earlier
    HandSample r = sample(1);
    t = tl;
    c.update(tl - offset, nullptr, hand[1].present ? &r : nullptr, nullptr);
    for (const Event& e : c.events()) log.push_back(e);
  }
  void run(double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / dt + 0.5); ++i) step();
  }
  void moveTo(int h, const HandPose& target, double seconds) {
    const HandPose from = hand[h].pose;
    const int n = std::max(1, static_cast<int>(seconds / dt + 0.5));
    for (int i = 1; i <= n; ++i) {
      hand[h].pose = gctest::mix(from, target, static_cast<float>(i) / n);
      step();
    }
  }
};
}  // namespace

TEST_CASE("input: hands fed in separate update() calls behave like a combined call") {
  for (double offset : {0.0, 0.004}) {
    Composer c;
    GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
    GestureId drag = c.add(gesture("drag").trigger(active(pinch(Finger::Middle))).updates(false));
    SplitSim sim(c);
    sim.offset = offset;
    sim.noise = 0.001f;
    sim.run(0.4);
    // Left hand holds a middle-finger drag while the right hand clicks three times.
    sim.moveTo(L, HandPose::pinch(Finger::Middle), 0.08);
    for (int i = 0; i < 3; ++i) {
      sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
      sim.run(0.15);
      sim.moveTo(R, HandPose::relaxed(), 0.08);
      sim.run(0.25);
    }
    sim.moveTo(L, HandPose::relaxed(), 0.08);
    sim.run(0.3);
    CHECK_EQ(sim.count(sel, Phase::Triggered, R), 3);
    CHECK_EQ(sim.count(sel, Phase::Triggered, L), 0);
    CHECK_EQ(sim.count(drag, Phase::Began, L), 1);
    CHECK_EQ(sim.count(drag, Phase::Ended, L), 1);
    CHECK_EQ(sim.count(drag, Phase::Cancelled), 0);
    CHECK(c.features(Hand::Left).tracked);
    CHECK(c.features(Hand::Right).tracked);
  }
}

TEST_CASE("input: accumulated events survive per-hand update() calls until cleared") {
  // Same motion fed (a) combined in one call per frame and (b) one call per hand, reading events
  // only once per frame. Both must see exactly the same events.
  std::vector<Event> combined, split;
  int callbacks = 0;
  for (int mode = 0; mode < 2; ++mode) {
    Config cfg;
    cfg.autoClearEvents = mode == 0;
    Composer c(cfg);
    c.add(gesture("select").trigger(press(pinch(Finger::Index))).onEvent([&](const Event&) {
      if (mode == 1) ++callbacks;
    }));
    c.add(gesture("hold").trigger(hold(pinch(Finger::Middle), 300ms)));
    Sim sim(c);
    std::vector<Event>& out = mode == 0 ? combined : split;
    for (int i = 0; i < 400; ++i) {
      const float a = 0.5f + 0.5f * std::sin(i * 0.06f);
      sim.hand[0].pose = gctest::mix(HandPose::relaxed(), HandPose::pinch(Finger::Middle), a);
      sim.hand[1].pose = gctest::mix(HandPose::relaxed(), HandPose::pinch(Finger::Index), 1.f - a);
      sim.t += sim.dt;
      sim.head.time = sim.t;
      HandSample l = sim.sample(0), r = sim.sample(1);
      if (mode == 0) {
        c.update(sim.t, &l, &r, &sim.head);
      } else {
        c.update(sim.t, &l, nullptr, &sim.head);  // left hand's data...
        c.update(sim.t, nullptr, &r);             // ...right hand's data, then read once
      }
      for (const Event& e : c.events()) out.push_back(e);
      if (mode == 1) c.clearEvents();
    }
    CHECK(c.droppedEvents() == 0);
  }
  CHECK(!combined.empty());
  CHECK_EQ(split.size(), combined.size());
  for (size_t i = 0; i < std::min(split.size(), combined.size()); ++i) {
    CHECK(split[i].id == combined[i].id);
    CHECK(split[i].hand == combined[i].hand);
    CHECK(split[i].phase == combined[i].phase);
  }
  int selects = 0;
  for (const Event& e : split) selects += e.id == 0 ? 1 : 0;
  CHECK_EQ(callbacks, selects);  // each callback fired exactly once
}

TEST_CASE("input: a hand that stops sending data is cancelled after the input timeout") {
  Composer c;
  GestureId drag = c.add(gesture("drag").trigger(active(pinch(Finger::Middle))).updates(false));
  SplitSim sim(c);
  sim.run(0.3);
  sim.moveTo(L, HandPose::pinch(Finger::Middle), 0.08);
  sim.run(0.2);
  CHECK_EQ(sim.count(drag, Phase::Began, L), 1);
  sim.hand[L].present = false;  // left data stops arriving; right keeps updating
  sim.run(0.08);                // within inputTimeout: held
  CHECK_EQ(sim.count(drag, Phase::Cancelled), 0);
  CHECK(c.features(Hand::Left).tracked);
  sim.run(0.4);                 // timeout (0.1 s) + grace (0.12 s) elapsed
  CHECK_EQ(sim.count(drag, Phase::Cancelled, L), 1);
  CHECK(!c.features(Hand::Left).tracked);
}

TEST_CASE("input: hand tracking slower than the update loop (repeated samples)") {
  Composer c;
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
  Sim sim(c);
  sim.dt = 1.0 / 90.0;
  sim.run(0.3);
  // 30 Hz hand data delivered to a 90 Hz loop: each sample is passed three times.
  HandSample held;
  int k = 0;
  auto frame = [&](const HandPose& p) {
    sim.t += sim.dt;
    sim.head.time = sim.t;
    if (k++ % 3 == 0) {
      sim.hand[R].pose = p;
      held = sim.sample(R);
    }
    c.update(sim.t, nullptr, &held, &sim.head);
    for (const Event& e : c.events()) sim.log.push_back(e);
  };
  for (int cycle = 0; cycle < 4; ++cycle) {
    for (int i = 1; i <= 9; ++i) frame(gctest::mix(HandPose::relaxed(), HandPose::pinch(Finger::Index), i / 9.f));
    for (int i = 0; i < 15; ++i) frame(HandPose::pinch(Finger::Index));
    for (int i = 1; i <= 9; ++i) frame(gctest::mix(HandPose::pinch(Finger::Index), HandPose::relaxed(), i / 9.f));
    for (int i = 0; i < 25; ++i) frame(HandPose::relaxed());
  }
  CHECK_EQ(sim.count(sel, Phase::Triggered, R), 4);
}

TEST_CASE("grab: whole-hand and pinch grabs") {
  Composer c;
  auto fist = curled(Finger::Index, Finger::Middle, Finger::Ring, Finger::Little);
  GestureId grab = c.add(gesture("grab")
                             .trigger(active(Condition(fist).thresholds(0.6f, 0.3f).releaseDwell(80ms).onset(true)))
                             .exclusive("grab")
                             .updates(false));
  // Shares the index pinch with select: .shared() lets both fire (the app decides grab vs click).
  GestureId pinchGrab = c.add(gesture("pinch_grab")
                                  .trigger(active(Condition(pinch(Finger::Index)).onset(true)))
                                  .exclusive("grab")
                                  .shared()
                                  .updates(false));
  GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))).gate(!grasping() && !occupied()));
  Sim sim(c);
  sim.noise = 0.001f;
  sim.run(0.4);

  // Close the hand: grab begins; select is suppressed while grabbing.
  sim.moveTo(R, HandPose::fist(), 0.15);
  sim.run(0.5);
  CHECK_EQ(sim.count(grab, Phase::Began, R), 1);
  CHECK(c.state(grab, Hand::Right).active);
  sim.moveTo(R, HandPose::relaxed(), 0.15);
  sim.run(0.3);
  CHECK_EQ(sim.count(grab, Phase::Ended, R), 1);
  CHECK_EQ(sim.count(sel, Phase::Triggered), 0);

  // Pinch grab for small objects.
  sim.run(0.4);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  sim.run(0.3);
  CHECK_EQ(sim.count(pinchGrab, Phase::Began, R), 1);
  CHECK_EQ(sim.count(sel, Phase::Triggered, R), 1);
  sim.moveTo(R, HandPose::relaxed(), 0.08);
  sim.run(0.3);
  CHECK_EQ(sim.count(pinchGrab, Phase::Ended, R), 1);

  // A hand that appears already closed does not grab (onset required).
  sim.hand[L].present = false;
  sim.run(0.3);
  sim.hand[L].pose = HandPose::fist();
  sim.hand[L].present = true;
  sim.run(0.5);
  CHECK_EQ(sim.count(grab, Phase::Began, L), 0);
}

TEST_CASE("click: fails when the hand drags") {
  Composer c;
  GestureId clk = c.add(gesture("click").trigger(click(pinch(Finger::Index))));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(sim.count(clk, Phase::Triggered), 1);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  for (int i = 0; i < 20; ++i) {
    sim.hand[R].wrist.x += 0.006f;
    sim.step();
  }
  sim.moveTo(R, HandPose::relaxed(), 0.08);
  sim.run(0.2);
  CHECK_EQ(sim.count(clk, Phase::Triggered), 1);
}

TEST_CASE("hold: progress reported, fires once at the duration") {
  Composer c;
  GestureId h = c.add(gesture("hold").trigger(hold(pinch(Finger::Index), 500ms)));
  Sim sim(c);
  sim.run(0.3);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  sim.run(0.25);
  CHECK(c.state(h, Hand::Right).progress > 0.3f);
  CHECK(c.state(h, Hand::Right).progress < 0.8f);
  CHECK_EQ(sim.count(h, Phase::Triggered), 0);
  sim.run(0.5);
  CHECK_EQ(sim.count(h, Phase::Triggered), 1);
}

TEST_CASE("repeat: double pinch within the window") {
  Composer c;
  GestureId dbl = c.add(gesture("undo").trigger(repeat(click(pinch(Finger::Index)), 2).within(450ms)));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.06);
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.4);
  CHECK_EQ(sim.count(dbl, Phase::Triggered), 1);
  // Too slow: two pinches 0.8 s apart.
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.6);
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.4);
  CHECK_EQ(sim.count(dbl, Phase::Triggered), 1);
}

TEST_CASE("seq: middle then ring within the window") {
  Composer c;
  GestureId combo = c.add(
      gesture("combo").trigger(seq(click(pinch(Finger::Middle)), click(pinch(Finger::Ring))).within(800ms)));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Middle, 0.06, 0.08, 0.06, 0.1);
  pinchCycle(sim, R, Finger::Ring, 0.06, 0.08, 0.06, 0.3);
  CHECK_EQ(sim.count(combo, Phase::Triggered), 1);
  // Wrong order.
  pinchCycle(sim, R, Finger::Ring, 0.06, 0.08, 0.06, 0.1);
  pinchCycle(sim, R, Finger::Middle, 0.06, 0.08, 0.06, 0.9);
  CHECK_EQ(sim.count(combo, Phase::Triggered), 1);
}

TEST_CASE("both: two-handed chord requires synchronised onsets") {
  Composer c;
  GestureId scale = c.add(gesture("scale").trigger(both(active(pinch(Finger::Index))).sync(150ms)));
  Sim sim(c);
  sim.run(0.3);
  // Simultaneous.
  const HandPose p = HandPose::pinch(Finger::Index);
  for (int i = 1; i <= 8; ++i) {
    sim.hand[L].pose = gctest::mix(HandPose::relaxed(), p, i / 8.f);
    sim.hand[R].pose = sim.hand[L].pose;
    sim.step();
  }
  sim.run(0.2);
  CHECK_EQ(sim.count(scale, Phase::Began), 1);
  CHECK(sim.log.back().bothHands);
  sim.moveTo(L, HandPose::relaxed(), 0.08);
  sim.run(0.1);
  CHECK_EQ(sim.count(scale, Phase::Ended), 1);
  sim.moveTo(R, HandPose::relaxed(), 0.08);
  sim.run(0.3);
  // Staggered by 0.5 s: no chord.
  sim.moveTo(L, p, 0.08);
  sim.run(0.5);
  sim.moveTo(R, p, 0.08);
  sim.run(0.3);
  CHECK_EQ(sim.count(scale, Phase::Began), 1);
}

TEST_CASE("Resolve::Wait: single pinch waits for the double-pinch window") {
  Composer c;
  auto ip = pinch(Finger::Index);
  GestureId single = c.add(gesture("single").trigger(click(ip)).ambiguity(Resolve::Wait, 400ms));
  GestureId dbl = c.add(gesture("double").trigger(repeat(click(ip), 2).within(450ms)));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.6);
  CHECK_EQ(sim.count(single, Phase::Triggered), 1);
  CHECK_EQ(sim.count(dbl, Phase::Triggered), 0);
  sim.clearLog();
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.06);
  pinchCycle(sim, R, Finger::Index, 0.06, 0.06, 0.06, 0.6);
  CHECK_EQ(sim.count(dbl, Phase::Triggered), 1);
  // The double consumed both physical clicks: the waiting single is cancelled, the second single lost arbitration.
  CHECK_EQ(sim.count(single, Phase::Triggered), 0);
}

TEST_CASE("point + aim: continuous ray along the index finger, rewound on confirm") {
  Composer c;
  auto pointPose = extended(Finger::Index) && curled(Finger::Middle, Finger::Ring, Finger::Little);
  GestureId aim = c.add(gesture("aim").trigger(active(pointPose).minDuration(100ms)).aim(Aim::IndexRay));
  GestureId go = c.add(gesture("go")
                           .trigger(release(pointPose))
                           .arm(pointPose, 100ms)
                           .aim(Aim::IndexRay, Stabilize::RewindToOnset));
  CHECK(c.compile().ok);
  Sim sim(c);
  sim.run(0.3);
  sim.moveTo(R, HandPose::point(), 0.1);
  sim.run(0.5);
  CHECK_EQ(sim.count(aim, Phase::Began), 1);
  const GestureState st = c.state(aim, Hand::Right);
  CHECK(st.active && st.rayValid);
  CHECK(st.ray.direction.z < -0.95f);
  // Confirm by curling the index finger; the hand rotates downward as it does (Heisenberg effect).
  for (int i = 0; i < 12; ++i) {
    sim.hand[R].fingerDir = normalize(sim.hand[R].fingerDir + Vec3{0.f, -0.04f, 0.f});
    HandPose p = HandPose::point();
    p.curl[1] = 0.05f + 0.07f * i;
    sim.hand[R].pose = p;
    sim.step();
  }
  sim.run(0.2);
  CHECK_EQ(sim.count(aim, Phase::Ended), 1);
  CHECK_EQ(sim.count(go, Phase::Triggered), 1);
  for (const Event& e : sim.log)
    if (e.id == go && e.phase == Phase::Triggered) {
      CHECK(e.rayValid);
      CHECK(e.ray.direction.y > -0.15f);  // ray from before the confirm motion, not after
    }
}

TEST_CASE("hand-size invariance: small and large hands pinch with the same definition") {
  for (float scale : {0.75f, 1.25f}) {
    Composer c;
    GestureId sel = c.add(gesture("select").trigger(press(pinch(Finger::Index))));
    Sim sim(c);
    HandPose rest = HandPose::relaxed();
    rest.scale = scale;
    sim.hand[R].pose = rest;
    sim.run(2.0);
    HandPose p = HandPose::pinch(Finger::Index);
    p.scale = scale;
    for (int i = 0; i < 3; ++i) {
      sim.moveTo(R, p, 0.08);
      sim.run(0.15);
      sim.moveTo(R, rest, 0.08);
      sim.run(0.25);
    }
    CHECK_EQ(sim.count(sel, Phase::Triggered), 3);
  }
}

TEST_CASE("cancelIf ends a continuous gesture") {
  Composer c;
  GestureId drag = c.add(gesture("drag").trigger(active(pinch(Finger::Index))).cancelIf(occupied()).updates(false));
  Sim sim(c);
  sim.run(0.3);
  sim.moveTo(R, HandPose::pinch(Finger::Index), 0.08);
  sim.run(0.2);
  c.setHandOccupied(Hand::Right, true);
  sim.run(0.2);
  CHECK_EQ(sim.count(drag, Phase::Began), 1);
  CHECK_EQ(sim.count(drag, Phase::Cancelled), 1);
  c.setHandOccupied(Hand::Right, false);
  sim.run(0.2);
  CHECK_EQ(sim.count(drag, Phase::Began), 1);  // same physical pinch does not restart
}

TEST_CASE("callbacks receive events") {
  Composer c;
  int calls = 0;
  c.add(gesture("select").trigger(press(pinch(Finger::Index))).onEvent([&](const Event& e) {
    if (e.phase == Phase::Triggered) ++calls;
  }));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Index);
  CHECK_EQ(calls, 1);
}

TEST_CASE("compile diagnostics") {
  {
    Composer c;
    c.add(gesture("bad").trigger(seq(active(pinch(Finger::Index)), press(pinch(Finger::Middle)))));
    CHECK(!c.compile().ok);
  }
  {
    Composer c;
    c.add(gesture("bad").trigger(seq(both(press(pinch(Finger::Index))), press(pinch(Finger::Middle)))));
    CHECK(!c.compile().ok);
  }
  {
    Composer c;
    c.add(gesture("bad").trigger(press(pinch(Finger::Index)).within(100ms)));
    CHECK(!c.compile().ok);
  }
  {
    Composer c;
    c.add(gesture("missing"));
    CHECK(!c.compile().ok);
  }
  {
    Composer c;
    c.add(gesture("a").trigger(press(pinch(Finger::Index))));
    c.add(gesture("b").trigger(press(pinch(Finger::Index))));
    const CompileResult& r = c.compile();
    CHECK(r.ok);
    CHECK_EQ(r.warnings.size(), size_t(1));
  }
}

TEST_CASE("hash-consing: identical sub-expressions share one condition") {
  Composer c;
  GestureId a = c.add(gesture("a").trigger(press(pinch(Finger::Index))).priority(1));
  GestureId b = c.add(gesture("b").trigger(press(pinch(Finger::Index))));
  Sim sim(c);
  sim.run(0.3);
  pinchCycle(sim, R, Finger::Index);
  // Separately-constructed but identical pinches are the same physical action: only the winner fires.
  CHECK_EQ(sim.count(a, Phase::Triggered), 1);
  CHECK_EQ(sim.count(b, Phase::Triggered), 0);
}

TEST_CASE("predictive onset fires earlier") {
  double tPlain = 0, tPredict = 0;
  for (int mode = 0; mode < 2; ++mode) {
    Composer c;
    Condition cond = pinch(Finger::Index);
    if (mode == 1) cond.predict(30ms);
    GestureId g = c.add(gesture("g").trigger(press(cond)));
    Sim sim(c);
    sim.run(0.3);
    sim.moveTo(R, HandPose::pinch(Finger::Index), 0.2);
    sim.run(0.1);
    for (const Event& e : sim.log)
      if (e.id == g) (mode ? tPredict : tPlain) = e.time;
  }
  CHECK(tPlain > 0 && tPredict > 0);
  CHECK(tPredict < tPlain);
}
