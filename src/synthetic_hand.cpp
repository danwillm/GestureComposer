#include "gesture_composer/testing/synthetic_hand.h"

namespace gc {
namespace testing {

namespace {

struct FingerDef {
  Vec3 base;      // metacarpal base
  Vec3 knuckle;   // proximal (MCP)
  float len[3];   // proximal, intermediate, distal phalanx
  float spreadSign;
};

// Right hand, local frame.
const FingerDef kFingers[4] = {
    {{-0.014f, 0.f, -0.025f}, {-0.033f, 0.f, -0.092f}, {0.042f, 0.025f, 0.022f}, -1.f},
    {{-0.004f, 0.f, -0.025f}, {-0.011f, 0.f, -0.095f}, {0.046f, 0.028f, 0.024f}, 0.f},
    {{0.006f, 0.f, -0.025f}, {0.011f, 0.f, -0.090f}, {0.043f, 0.027f, 0.023f}, 1.f},
    {{0.015f, 0.f, -0.025f}, {0.037f, 0.f, -0.082f}, {0.034f, 0.020f, 0.020f}, 1.f},
};

// Thumb: metacarpal (CMC), proximal (MCP), distal (IP), tip. Rest and tucked-across-palm positions.
const Vec3 kThumbRest[4] = {{-0.020f, -0.010f, -0.025f}, {-0.042f, -0.018f, -0.050f}, {-0.055f, -0.022f, -0.080f},
                            {-0.062f, -0.024f, -0.103f}};
const Vec3 kThumbTucked[4] = {{-0.020f, -0.010f, -0.025f}, {-0.040f, -0.024f, -0.050f}, {-0.030f, -0.034f, -0.068f},
                              {-0.010f, -0.036f, -0.075f}};

float defaultRadius(Joint j) {
  switch (j) {
    case Joint::Palm:
    case Joint::Wrist: return 0.020f;
    case Joint::ThumbTip:
    case Joint::IndexTip:
    case Joint::MiddleTip:
    case Joint::RingTip:
    case Joint::LittleTip: return 0.0075f;
    case Joint::ThumbDistal:
    case Joint::IndexDistal:
    case Joint::MiddleDistal:
    case Joint::RingDistal:
    case Joint::LittleDistal: return 0.0085f;
    default: return 0.010f;
  }
}

}  // namespace

HandPose HandPose::open() {
  HandPose p;
  p.curl = {0.f, 0.f, 0.f, 0.f, 0.f};
  return p;
}
HandPose HandPose::relaxed() { return HandPose{}; }
HandPose HandPose::point() {
  HandPose p;
  p.curl = {0.5f, 0.02f, 0.85f, 0.85f, 0.85f};
  return p;
}
HandPose HandPose::fist() {
  HandPose p;
  p.curl = {0.8f, 0.9f, 0.9f, 0.9f, 0.9f};
  return p;
}
HandPose HandPose::pinch(Finger f, float amount) {
  HandPose p;
  p.curl = {0.f, 0.08f, 0.1f, 0.1f, 0.1f};
  p.curl[static_cast<size_t>(f)] = 0.25f + 0.15f * amount;
  p.pinchFinger = f;
  p.pinchAmount = amount;
  return p;
}
HandPose HandPose::grip() {
  HandPose p;
  p.curl = {0.6f, 0.75f, 0.78f, 0.8f, 0.8f};
  p.pinchFinger = Finger::Index;  // thumb resting on the index finger around the rim
  p.pinchAmount = 1.f;
  return p;
}

Pose wristPose(Vec3 position, Vec3 fingerDirection, Vec3 palmNormal) {
  // Local -Z -> fingerDirection, local +Y -> -palmNormal (so local -Y, the palm, faces palmNormal).
  return {position, quatLookRotation(fingerDirection, -palmNormal)};
}

HandSample makeHand(Hand hand, const Pose& wrist, const HandPose& pose, double time) {
  HandSample s;
  s.hand = hand;
  s.time = time;
  s.tracked = true;
  const float k = pose.scale;
  const float mirror = hand == Hand::Left ? -1.f : 1.f;
  const Vec3 palmN{0.f, -1.f, 0.f};

  std::array<Vec3, kJointCount> local{};
  std::array<Quat, kJointCount> localRot{};
  auto setJoint = [&](Joint j, Vec3 p, Vec3 boneDir, Vec3 dorsal) {
    local[jointIndex(j)] = p;
    localRot[jointIndex(j)] = quatLookRotation(boneDir, dorsal);
  };

  local[jointIndex(Joint::Wrist)] = {0.f, 0.f, 0.f};
  localRot[jointIndex(Joint::Wrist)] = quatLookRotation({0.f, 0.f, -1.f}, {0.f, 1.f, 0.f});

  std::array<Vec3, kFingerCount> tip{}, distal{}, palmar{};
  for (int fi = 1; fi < kFingerCount; ++fi) {
    const FingerDef& fd = kFingers[fi - 1];
    const float c = clamp01(pose.curl[fi]);
    const Quat qs = quatFromAxisAngle({0.f, 1.f, 0.f}, -fd.spreadSign * pose.spread[fi]);
    const Vec3 dir0 = rotate(qs, Vec3{0.f, 0.f, -1.f});
    const Vec3 axis = normalize(cross(dir0, palmN));
    const float a1 = c * degToRad(90.f), a2 = c * degToRad(100.f), a3 = c * degToRad(80.f);
    const Vec3 d1 = rotate(quatFromAxisAngle(axis, a1), dir0);
    const Vec3 d2 = rotate(quatFromAxisAngle(axis, a1 + a2), dir0);
    const Vec3 d3 = rotate(quatFromAxisAngle(axis, a1 + a2 + a3), dir0);
    const Vec3 m = fd.base * k;
    const Vec3 p = fd.knuckle * k;
    const Vec3 i = p + d1 * (fd.len[0] * k);
    const Vec3 d = i + d2 * (fd.len[1] * k);
    const Vec3 t = d + d3 * (fd.len[2] * k);
    const auto f = static_cast<Finger>(fi);
    const Vec3 up{0.f, 1.f, 0.f};
    setJoint(fingerJoint(f, Segment::Metacarpal), m, normalize(p - m), up);
    setJoint(fingerJoint(f, Segment::Proximal), p, d1, cross(d1, axis));
    setJoint(fingerJoint(f, Segment::Intermediate), i, d2, cross(d2, axis));
    setJoint(fingerJoint(f, Segment::Distal), d, d3, cross(d3, axis));
    setJoint(fingerJoint(f, Segment::Tip), t, d3, cross(d3, axis));
    tip[fi] = t;
    distal[fi] = d;
    palmar[fi] = normalize(cross(axis, d3));
  }

  // Thumb: blend rest -> tucked by curl, then towards the pinch target.
  Vec3 thumb[4];
  const float tc = clamp01(pose.curl[0]);
  for (int j = 0; j < 4; ++j) thumb[j] = lerp(kThumbRest[j], kThumbTucked[j], tc) * k;
  if (pose.pinchFinger != Finger::Thumb && pose.pinchAmount > 0.f) {
    const int fi = static_cast<int>(pose.pinchFinger);
    const float rsum = (defaultRadius(Joint::ThumbTip) + defaultRadius(fingerJoint(pose.pinchFinger, Segment::Tip))) * k;
    const Vec3 pad = lerp(distal[fi], tip[fi], 0.8f);
    const Vec3 target = pad + palmar[fi] * (rsum + pose.pinchGap);
    const float a = clamp01(pose.pinchAmount);
    const Vec3 tipPos = lerp(thumb[3], target, a);
    const Vec3 dOff = thumb[2] - thumb[3];
    const Vec3 pOff = thumb[1] - thumb[3];
    thumb[2] = lerp(thumb[2], tipPos + dOff, a);
    thumb[1] = lerp(thumb[1], lerp(thumb[1], tipPos + pOff, 0.5f), a);
    thumb[3] = tipPos;
  }
  const Joint tj[4] = {Joint::ThumbMetacarpal, Joint::ThumbProximal, Joint::ThumbDistal, Joint::ThumbTip};
  for (int j = 0; j < 4; ++j) {
    const Vec3 dir = normalize(j < 3 ? thumb[j + 1] - thumb[j] : thumb[3] - thumb[2]);
    setJoint(tj[j], thumb[j], dir, Vec3{1.f, 0.f, 0.f});
  }

  local[jointIndex(Joint::Palm)] =
      lerp(local[jointIndex(Joint::MiddleMetacarpal)], local[jointIndex(Joint::MiddleProximal)], 0.5f);
  localRot[jointIndex(Joint::Palm)] = localRot[jointIndex(Joint::Wrist)];

  for (size_t i = 0; i < kCoreJointCount; ++i) {
    Vec3 p = local[i];
    p.x *= mirror;
    Quat q = localRot[i];
    if (hand == Hand::Left) q = {q.x, -q.y, -q.z, q.w};  // mirror across the YZ plane
    JointSample& js = s.joints[i];
    js.pose.position = wrist.position + rotate(wrist.orientation, p);
    js.pose.orientation = normalize(wrist.orientation * q);
    js.radius = defaultRadius(static_cast<Joint>(i)) * k;
    js.confidence = 1.f;
    s.validMask |= 1u << i;
  }
  return s;
}

}  // namespace testing
}  // namespace gc
