#pragma once
// Apple visionOS ARKit HandSkeleton -> gc::HandSample. Header-only, no Apple dependency, so it can be
// called from Swift through C++ interop (or a C shim) and unit-tested on any platform.
//
// Swift side, per HandAnchor:
//   for (i, name) in gcJointOrder.enumerated() {          // order below, NOT HandSkeleton.JointName.allCases
//       let joint = anchor.handSkeleton!.joint(name)
//       let world = anchor.originFromAnchorTransform * joint.anchorFromJointTransform
//       withUnsafeBytes(of: world) { memcpy(&matrices[i * 16], $0.baseAddress!, 64) }   // column-major
//       tracked[i] = joint.isTracked
//   }
//   gc::visionos::convert(matrices, tracked, anchor.isTracked, hand, timestamp, sample)

#include <cmath>
#include <cstdint>

#include "gesture_composer/math.h"
#include "gesture_composer/types.h"

namespace gc {
namespace visionos {

// ARKit joint names in the order the adapter expects them.
enum class ARKitJoint : uint8_t {
  wrist,
  thumbKnuckle, thumbIntermediateBase, thumbIntermediateTip, thumbTip,
  indexFingerMetacarpal, indexFingerKnuckle, indexFingerIntermediateBase, indexFingerIntermediateTip, indexFingerTip,
  middleFingerMetacarpal, middleFingerKnuckle, middleFingerIntermediateBase, middleFingerIntermediateTip, middleFingerTip,
  ringFingerMetacarpal, ringFingerKnuckle, ringFingerIntermediateBase, ringFingerIntermediateTip, ringFingerTip,
  littleFingerMetacarpal, littleFingerKnuckle, littleFingerIntermediateBase, littleFingerIntermediateTip, littleFingerTip,
  forearmWrist, forearmArm,
  Count
};
constexpr int kARKitJointCount = static_cast<int>(ARKitJoint::Count);

// ARKitJoint -> canonical joint. ARKit's thumb "knuckle" is the CMC joint (OpenXR ThumbMetacarpal).
constexpr Joint kJointMap[kARKitJointCount] = {
    Joint::Wrist,
    Joint::ThumbMetacarpal, Joint::ThumbProximal, Joint::ThumbDistal, Joint::ThumbTip,
    Joint::IndexMetacarpal, Joint::IndexProximal, Joint::IndexIntermediate, Joint::IndexDistal, Joint::IndexTip,
    Joint::MiddleMetacarpal, Joint::MiddleProximal, Joint::MiddleIntermediate, Joint::MiddleDistal, Joint::MiddleTip,
    Joint::RingMetacarpal, Joint::RingProximal, Joint::RingIntermediate, Joint::RingDistal, Joint::RingTip,
    Joint::LittleMetacarpal, Joint::LittleProximal, Joint::LittleIntermediate, Joint::LittleDistal, Joint::LittleTip,
    Joint::ForearmWrist, Joint::ForearmArm,
};

// Column-major 4x4 (simd_float4x4 layout) -> Pose. Scale is removed from the rotation columns.
inline Pose poseFromMatrix(const float* m) {
  Vec3 x{m[0], m[1], m[2]}, y{m[4], m[5], m[6]}, z{m[8], m[9], m[10]};
  x = normalize(x, {1.f, 0.f, 0.f});
  y = normalize(y - x * dot(x, y), {0.f, 1.f, 0.f});
  z = cross(x, y);
  return {{m[12], m[13], m[14]}, quatFromBasis(x, y, z)};
}

// matrices: kARKitJointCount * 16 floats, world space (originFromAnchor * anchorFromJoint).
// Joint orientations keep ARKit's axis convention; GestureComposer derives its frames from positions,
// so features are identical across platforms. The palm joint, absent in ARKit, is synthesised.
inline void convert(const float* matrices, const bool* jointTracked, bool anchorTracked, Hand hand,
                    double timeSeconds, HandSample& out) {
  out = HandSample{};
  out.hand = hand;
  out.time = timeSeconds;
  out.tracked = anchorTracked;
  for (int i = 0; i < kARKitJointCount; ++i) {
    const Joint j = kJointMap[i];
    JointSample& js = out[j];
    js.pose = poseFromMatrix(matrices + i * 16);
    js.radius = 0.f;  // not provided: library defaults scaled by measured hand size
    js.confidence = jointTracked[i] ? 1.f : 0.f;
    if (jointTracked[i]) out.validMask |= jointBit(j);
  }
  const JointSample& mm = out[Joint::MiddleMetacarpal];
  const JointSample& mp = out[Joint::MiddleProximal];
  if (out.valid(Joint::MiddleMetacarpal) && out.valid(Joint::MiddleProximal)) {
    JointSample& palm = out[Joint::Palm];
    palm.pose.position = lerp(mm.pose.position, mp.pose.position, 0.5f);
    palm.pose.orientation = mm.pose.orientation;
    palm.confidence = 1.f;
    out.validMask |= jointBit(Joint::Palm);
  }
}

}  // namespace visionos
}  // namespace gc
