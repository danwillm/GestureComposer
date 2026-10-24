#pragma once
// Scripted hand-motion simulation for tests.

#include <random>
#include <vector>

#include "gesture_composer/gesture_composer.h"
#include "gesture_composer/testing/synthetic_hand.h"

namespace gctest {

using namespace gc;
using gc::testing::HandPose;

inline HandPose mix(const HandPose& a, const HandPose& b, float t) {
  HandPose p = t < 0.5f ? a : b;
  for (int i = 0; i < kFingerCount; ++i) {
    p.curl[i] = lerpf(a.curl[i], b.curl[i], t);
    p.spread[i] = lerpf(a.spread[i], b.spread[i], t);
  }
  if (a.pinchFinger == b.pinchFinger) {
    p.pinchFinger = a.pinchFinger;
    p.pinchAmount = lerpf(a.pinchAmount, b.pinchAmount, t);
  } else {
    // Different target fingers: the thumb leaves one and approaches the other.
    const float ca = a.pinchFinger == Finger::Thumb ? 0.f : a.pinchAmount * (1.f - t);
    const float cb = b.pinchFinger == Finger::Thumb ? 0.f : b.pinchAmount * t;
    const bool useA = a.pinchFinger != Finger::Thumb && (ca > cb || b.pinchFinger == Finger::Thumb);
    p.pinchFinger = useA ? a.pinchFinger : b.pinchFinger;
    p.pinchAmount = useA ? ca : cb;
  }
  p.pinchGap = lerpf(a.pinchGap, b.pinchGap, t);
  p.scale = lerpf(a.scale, b.scale, t);
  return p;
}

struct HandState {
  bool present = true;
  HandPose pose = HandPose::relaxed();
  Vec3 wrist{0.15f, 1.30f, -0.35f};
  Vec3 fingerDir{0.f, 0.f, -1.f};
  Vec3 palmNormal{0.f, -1.f, 0.f};
};

struct Sim {
  Composer& c;
  double t = 1.0;
  double dt = 1.0 / 90.0;
  float noise = 0.f;  // metres of per-joint positional jitter
  bool headPresent = true;
  HeadSample head;
  HandState hand[2];
  std::vector<Event> log;
  std::mt19937 rng{1234};

  explicit Sim(Composer& composer) : c(composer) {
    head.pose.position = {0.f, 1.6f, 0.f};
    hand[0].wrist = {-0.15f, 1.30f, -0.35f};
  }

  HandSample sample(int h) {
    const HandState& s = hand[h];
    HandSample hs = gc::testing::makeHand(static_cast<Hand>(h), gc::testing::wristPose(s.wrist, s.fingerDir, s.palmNormal),
                                          s.pose, t);
    if (noise > 0.f) {
      std::uniform_real_distribution<float> u(-noise, noise);
      for (auto& j : hs.joints) j.pose.position += Vec3{u(rng), u(rng), u(rng)};
    }
    return hs;
  }

  void step() {
    t += dt;
    head.time = t;
    HandSample l = sample(0), r = sample(1);
    c.update(t, hand[0].present ? &l : nullptr, hand[1].present ? &r : nullptr, headPresent ? &head : nullptr);
    for (const Event& e : c.events()) log.push_back(e);
  }

  void run(double seconds) {
    const int n = static_cast<int>(seconds / dt + 0.5);
    for (int i = 0; i < n; ++i) step();
  }

  // Blend hand h's pose to `target` over `seconds`.
  void moveTo(int h, const HandPose& target, double seconds) {
    const HandPose from = hand[h].pose;
    const int n = std::max(1, static_cast<int>(seconds / dt + 0.5));
    for (int i = 1; i <= n; ++i) {
      hand[h].pose = mix(from, target, static_cast<float>(i) / n);
      step();
    }
  }

  // Palm faces the head, fingers up.
  void palmToHead(int h) {
    const Vec3 toHead = normalize(head.pose.position - hand[h].wrist);
    hand[h].palmNormal = toHead;
    hand[h].fingerDir = normalize(cross(cross(toHead, Vec3{0.f, 1.f, 0.f}), toHead));  // world up, projected
  }
  void palmDown(int h) {
    hand[h].palmNormal = {0.f, -1.f, 0.f};
    hand[h].fingerDir = {0.f, 0.f, -1.f};
  }

  int count(GestureId id, Phase phase, int handFilter = -1) const {
    int n = 0;
    for (const Event& e : log)
      if (e.id == id && e.phase == phase && (handFilter < 0 || static_cast<int>(e.hand) == handFilter)) ++n;
    return n;
  }
  void clearLog() { log.clear(); }
};

}  // namespace gctest
