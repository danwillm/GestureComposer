#pragma once
// Per-hand derived features. Computed once per frame, only for what registered gestures need.

#include <array>
#include <cstdint>

#include "types.h"

namespace gc {

// Feature groups a signal depends on. The composer computes the union of the masks of enabled gestures.
namespace Features {
enum : uint32_t {
  None = 0,
  Curl = 1u << 0,         // per-finger curl
  Spread = 1u << 1,       // abduction between adjacent fingers
  Pinch = 1u << 2,        // thumb-to-finger capsule distances + exclusivity
  Kinematics = 1u << 3,   // palm linear/angular velocity, articulation speed
  Grasp = 1u << 4,        // occupied-hand score (implies Curl | Pinch | Kinematics)
  AimIndex = 1u << 5,     // index-finger ray
  AimShoulder = 1u << 6,  // shoulder-through-pinch ray (needs head)
  All = 0x7Fu
};
}  // namespace Features
using FeatureMask = uint32_t;

struct HeadInfo {
  bool valid = false;
  Pose pose;
  Vec3 forward{0.f, 0.f, -1.f};
  bool gazeValid = false;
  Ray gaze;
};

// Read-only view of everything the library knows about one hand this frame.
struct HandFeatures {
  Hand hand = Hand::Left;
  double time = 0.0;
  bool tracked = false;      // tracked this frame (false while holding through a dropout)
  double lastTrackedTime = -1.0;
  float confidence = 0.f;    // overall hand confidence 0..1
  float handScale = 1.f;     // measured hand size relative to the reference adult hand
  bool scaleLocked = false;

  // Joint positions after validity fill-in (world, metres) and per-joint confidence.
  std::array<Vec3, kJointCount> positions{};
  std::array<float, kJointCount> radii{};
  std::array<float, kJointCount> jointConfidence{};

  // Palm frame (position-derived so it is identical across platforms).
  Vec3 palmPosition;
  Vec3 palmNormal{0.f, -1.f, 0.f};   // points out of the palm
  Vec3 palmForward{0.f, 0.f, -1.f};  // wrist -> middle knuckle
  Vec3 palmLateral{1.f, 0.f, 0.f};   // flexion axis = cross(forward, normal)

  std::array<float, kFingerCount> curl{};       // 0 = straight, 1 = fully curled
  std::array<float, kFingerCount> spread{};     // radians between finger f and f+1 (index..ring)
  std::array<float, kFingerCount> pinchDistance{};  // thumb -> finger surface distance, scaled metres
  std::array<float, kFingerCount> pinchConfidence{};
  std::array<float, kFingerCount> pinchExclusivity{};  // 0..1 margin over the next-closest finger

  Vec3 palmVelocity;           // m/s, filtered
  float palmSpeed = 0.f;       // m/s
  float angularSpeed = 0.f;    // rad/s
  float articulationSpeed = 0.f;  // mean fingertip speed in the palm frame, m/s
  float graspScore = 0.f;      // 0..1, includes post-grasp hold-off
  bool occupied = false;       // app-provided occupancy

  bool indexRayValid = false;
  Ray indexRay;
  bool shoulderRayValid = false;
  Ray shoulderRay;

  HeadInfo head;

  const Vec3& position(Joint j) const { return positions[jointIndex(j)]; }
  float curlOf(Finger f) const { return curl[static_cast<size_t>(f)]; }
  float pinchDistanceTo(Finger f) const { return pinchDistance[static_cast<size_t>(f)]; }
};

}  // namespace gc
