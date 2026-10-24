#pragma once
// Canonical, platform-independent hand input types.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "math.h"

namespace gc {

enum class Hand : uint8_t { Left = 0, Right = 1 };
enum class Finger : uint8_t { Thumb = 0, Index, Middle, Ring, Little };
constexpr int kFingerCount = 5;

// Which hands a gesture is evaluated on.
enum class Hands : uint8_t {
  Left,    // left hand only
  Right,   // right hand only
  Either,  // evaluated independently on each hand
  Both     // two-handed: the trigger must occur on both hands (see both())
};

// Canonical joint order = OpenXR XR_EXT_hand_tracking (26 joints) + 2 optional forearm joints.
enum class Joint : uint8_t {
  Palm, Wrist,
  ThumbMetacarpal, ThumbProximal, ThumbDistal, ThumbTip,
  IndexMetacarpal, IndexProximal, IndexIntermediate, IndexDistal, IndexTip,
  MiddleMetacarpal, MiddleProximal, MiddleIntermediate, MiddleDistal, MiddleTip,
  RingMetacarpal, RingProximal, RingIntermediate, RingDistal, RingTip,
  LittleMetacarpal, LittleProximal, LittleIntermediate, LittleDistal, LittleTip,
  ForearmWrist, ForearmArm,  // visionOS only; invalid elsewhere
  Count
};
constexpr size_t kJointCount = static_cast<size_t>(Joint::Count);
constexpr size_t kCoreJointCount = 26;

constexpr size_t jointIndex(Joint j) { return static_cast<size_t>(j); }
constexpr uint32_t jointBit(Joint j) { return 1u << static_cast<uint32_t>(j); }

// Segment of a finger, from base to tip. Thumb has no Intermediate (its "Metacarpal"..."Tip" are 4 joints).
enum class Segment : uint8_t { Metacarpal, Proximal, Intermediate, Distal, Tip };

// Joint for a given finger/segment. For the thumb, Intermediate maps to ThumbProximal's successor (ThumbDistal).
constexpr Joint fingerJoint(Finger f, Segment s) {
  if (f == Finger::Thumb) {
    switch (s) {
      case Segment::Metacarpal: return Joint::ThumbMetacarpal;
      case Segment::Proximal: return Joint::ThumbProximal;
      case Segment::Intermediate:
      case Segment::Distal: return Joint::ThumbDistal;
      case Segment::Tip: return Joint::ThumbTip;
    }
  }
  const auto base = static_cast<uint8_t>(Joint::IndexMetacarpal) + (static_cast<uint8_t>(f) - 1) * 5;
  return static_cast<Joint>(base + static_cast<uint8_t>(s));
}

struct JointSample {
  Pose pose;                // world space, metres
  float radius = 0.f;       // metres; 0 = unknown (anatomical default scaled by hand size)
  float confidence = 1.f;   // 0..1
};

struct HandSample {
  Hand hand = Hand::Left;
  double time = 0.0;        // seconds, monotonic; the *sample* time, not predicted display time
  bool tracked = false;
  uint32_t validMask = 0;   // one bit per Joint (jointBit)
  std::array<JointSample, kJointCount> joints{};

  bool valid(Joint j) const { return (validMask & jointBit(j)) != 0; }
  const JointSample& operator[](Joint j) const { return joints[jointIndex(j)]; }
  JointSample& operator[](Joint j) { return joints[jointIndex(j)]; }
};

struct HeadSample {
  double time = 0.0;
  Pose pose;                 // eye-centre pose; looks down local -Z
  std::optional<Ray> gaze;   // if the platform exposes eye gaze
};

// Time span in seconds; implicitly constructible from any std::chrono duration (e.g. 250ms).
struct Seconds {
  float value = 0.f;
  constexpr Seconds() = default;
  constexpr explicit Seconds(float s) : value(s) {}
  template <class Rep, class Period>
  constexpr Seconds(std::chrono::duration<Rep, Period> d)  // NOLINT(google-explicit-constructor)
      : value(std::chrono::duration_cast<std::chrono::duration<float>>(d).count()) {}
};

const char* toString(Finger f);
const char* toString(Hand h);
const char* toString(Joint j);

}  // namespace gc
