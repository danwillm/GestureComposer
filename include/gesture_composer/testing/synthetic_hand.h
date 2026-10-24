#pragma once
// Procedural hand skeleton for unit tests, examples and benchmarks.
// Forward kinematics over a reference adult hand (palm width 7 cm, palm length 9.5 cm).
// Local frame (before the wrist pose): fingers along -Z, palm facing -Y, right thumb on -X.

#include <array>

#include "../types.h"

namespace gc {
namespace testing {

struct HandPose {
  std::array<float, kFingerCount> curl{0.1f, 0.1f, 0.1f, 0.1f, 0.1f};  // 0..1; thumb = tuck across the palm
  std::array<float, kFingerCount> spread{};  // extra abduction, radians (index..little)
  Finger pinchFinger = Finger::Thumb;        // Thumb = no pinch
  float pinchAmount = 0.f;                   // 0 = rest thumb, 1 = thumb surface touching the finger pad
  float pinchGap = 0.f;                      // surface gap (m) at pinchAmount = 1
  float scale = 1.f;                         // hand size relative to the reference

  static HandPose open();     // flat, fingers straight
  static HandPose relaxed();  // natural slight curl
  static HandPose point();    // index straight, others curled
  static HandPose fist();
  static HandPose pinch(Finger f, float amount = 1.f);
  static HandPose grip();     // power grasp around a ~3.5 cm bar (steering wheel rim), thumb wrapped onto index
};

// Wrist pose whose fingers point along `fingerDirection` and whose palm faces `palmNormal`.
Pose wristPose(Vec3 position, Vec3 fingerDirection, Vec3 palmNormal);

HandSample makeHand(Hand hand, const Pose& wrist, const HandPose& pose, double time);

}  // namespace testing
}  // namespace gc
