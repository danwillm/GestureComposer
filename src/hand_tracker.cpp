#include "hand_tracker.h"

#include <algorithm>

namespace gc {
namespace detail {

namespace {

constexpr Joint kRequired[] = {Joint::Wrist, Joint::IndexProximal, Joint::MiddleProximal, Joint::LittleProximal};

float defaultRadius(Joint j) {
  switch (j) {
    case Joint::Palm: return 0.020f;
    case Joint::Wrist: return 0.020f;
    case Joint::ForearmWrist:
    case Joint::ForearmArm: return 0.025f;
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

Quat palmRotation(Vec3 lateral, Vec3 normal, Vec3 forward) { return quatFromBasis(lateral, normal, -forward); }

}  // namespace

HandTracker::HandTracker(Hand hand) {
  f_.hand = hand;
  initDefaults();
}

void HandTracker::reset() { *this = HandTracker(f_.hand); }

void HandTracker::initDefaults() {
  for (auto& c : curlFilters_) c = OneEuro(2.f, 4.f);
  // Pinch distance in metres: high beta so the onset is not delayed by smoothing.
  for (auto& p : pinchFilters_) p = OneEuro(2.f, 50.f);
  for (size_t i = 0; i < kJointCount; ++i) f_.radii[i] = defaultRadius(static_cast<Joint>(i));
  f_.pinchDistance.fill(1.f);
}

void HandTracker::update(const HandSample* s, double time, const HeadInfo& head, FeatureMask mask,
                         const Config& cfg) {
  f_.head = head;
  const float dt = prevTime_ >= 0.0 ? static_cast<float>(time - prevTime_) : 0.f;
  if (prevTime_ >= 0.0 && time <= prevTime_) return;  // reject duplicate / non-monotonic frames
  prevTime_ = time;
  f_.time = time;

  if (mask & Features::Grasp) mask |= Features::Curl | Features::Pinch | Features::Kinematics;

  bool usable = s != nullptr && s->tracked;
  if (usable) {
    for (Joint j : kRequired) usable = usable && s->valid(j);
  }

  if (!usable) {
    // Hold features through the dropout; conditions see confidence 0 and apply their grace period.
    f_.tracked = false;
    f_.confidence = 0.f;
    for (auto& c : f_.pinchConfidence) c = 0.f;
    if (f_.lastTrackedTime >= 0.0 && time - f_.lastTrackedTime > 10.0) {
      scaleCount_ = 0;  // likely a different user / long absence: re-measure
      f_.scaleLocked = false;
    }
    havePrevFrame_ = false;
    haveKinematicsPrev_ = false;
    return;
  }

  // ---- fill positions / confidence --------------------------------------------------------------
  float confSum = 0.f;
  for (size_t i = 0; i < kJointCount; ++i) {
    const auto j = static_cast<Joint>(i);
    const JointSample& js = s->joints[i];
    if (s->valid(j)) {
      f_.positions[i] = js.pose.position;
      f_.jointConfidence[i] = clamp01(js.confidence);
    } else {
      f_.jointConfidence[i] = 0.f;  // keep last known position
    }
    f_.radii[i] = js.radius > 0.f ? js.radius : defaultRadius(j) * f_.handScale;
    if (i < kCoreJointCount) confSum += f_.jointConfidence[i];
  }
  if (!s->valid(Joint::Palm) && s->valid(Joint::MiddleMetacarpal)) {
    f_.positions[jointIndex(Joint::Palm)] =
        lerp(f_.position(Joint::MiddleMetacarpal), f_.position(Joint::MiddleProximal), 0.5f);
    f_.jointConfidence[jointIndex(Joint::Palm)] =
        std::min(f_.jointConfidence[jointIndex(Joint::MiddleMetacarpal)],
                 f_.jointConfidence[jointIndex(Joint::MiddleProximal)]);
  }
  f_.tracked = true;
  f_.lastTrackedTime = time;
  f_.confidence = confSum / static_cast<float>(kCoreJointCount);

  if (!havePrevFrame_) {
    // Re-acquired after a dropout (or first frame): start filters fresh instead of from stale values.
    normalFilter_.reset();
    forwardFilter_.reset();
    for (auto& c : curlFilters_) c.reset();
    for (auto& p : pinchFilters_) p.reset();
    indexDirFilter_.reset();
    indexOriginFilter_.reset();
    shoulderDirFilter_.reset();
    shoulderOriginFilter_.reset();
  }
  const float fdt = havePrevFrame_ ? dt : 0.f;
  updateScale(time);
  computePalmFrame(fdt);
  if (mask & Features::Curl) computeCurl(fdt);
  if (mask & Features::Spread) computeSpread();
  if (mask & Features::Pinch) computePinch(fdt);
  if (mask & Features::Kinematics) computeKinematics(fdt);
  if (mask & Features::Grasp) computeGrasp(time, cfg);
  if (mask & (Features::AimIndex | Features::AimShoulder)) computeRays(fdt, cfg);

  // ray history
  if (mask & (Features::AimIndex | Features::AimShoulder)) {
    RaySample& r = rayHistory_[rayHead_];
    r.time = time;
    r.index = f_.indexRay;
    r.indexValid = f_.indexRayValid;
    r.shoulder = f_.shoulderRay;
    r.shoulderValid = f_.shoulderRayValid;
    rayHead_ = (rayHead_ + 1) % kRayHistory;
    rayCount_ = std::min(rayCount_ + 1, kRayHistory);
  }
  havePrevFrame_ = true;
}

void HandTracker::updateScale(double time) {
  if (f_.scaleLocked) return;
  const float conf = std::min({f_.jointConfidence[jointIndex(Joint::IndexProximal)],
                               f_.jointConfidence[jointIndex(Joint::LittleProximal)],
                               f_.jointConfidence[jointIndex(Joint::Wrist)],
                               f_.jointConfidence[jointIndex(Joint::MiddleProximal)]});
  if (conf < 0.5f) return;
  if (lastScaleSampleTime_ >= 0.0 && time - lastScaleSampleTime_ < 0.05) return;
  const float width = distance(f_.position(Joint::IndexProximal), f_.position(Joint::LittleProximal));
  const float len = distance(f_.position(Joint::Wrist), f_.position(Joint::MiddleProximal));
  const float raw = 0.5f * (width / kRefPalmWidth + len / kRefPalmLength);
  if (raw < 0.5f || raw > 1.8f) return;  // implausible: tracking glitch
  lastScaleSampleTime_ = time;
  scaleSamples_[scaleCount_++] = raw;
  std::array<float, kScaleSamples> tmp = scaleSamples_;
  std::nth_element(tmp.begin(), tmp.begin() + scaleCount_ / 2, tmp.begin() + scaleCount_);
  f_.handScale = tmp[scaleCount_ / 2];
  if (scaleCount_ == kScaleSamples) f_.scaleLocked = true;
}

void HandTracker::computePalmFrame(float dt) {
  const Vec3 wrist = f_.position(Joint::Wrist);
  const Vec3 fwdRaw = normalize(f_.position(Joint::MiddleProximal) - wrist);
  const Vec3 across = f_.position(Joint::LittleProximal) - f_.position(Joint::IndexProximal);
  Vec3 n = f_.hand == Hand::Right ? cross(fwdRaw, across) : cross(across, fwdRaw);
  n = normalize(normalFilter_.filter(normalize(n), dt), f_.palmNormal);
  const Vec3 fwd = normalize(forwardFilter_.filter(fwdRaw, dt), f_.palmForward);
  const Vec3 lateral = normalize(cross(fwd, n), f_.palmLateral);
  f_.palmNormal = n;
  f_.palmLateral = lateral;
  f_.palmForward = cross(n, lateral);
  f_.palmPosition = f_.position(Joint::Palm);
}

void HandTracker::computeCurl(float dt) {
  // Fingers: flexion at MCP/PIP/DIP measured about the palm's lateral axis (removes abduction).
  constexpr float kRange = degToRad(90.f + 100.f + 80.f);
  for (int fi = 1; fi < kFingerCount; ++fi) {
    const auto fg = static_cast<Finger>(fi);
    const Vec3 m = f_.position(fingerJoint(fg, Segment::Metacarpal));
    const Vec3 p = f_.position(fingerJoint(fg, Segment::Proximal));
    const Vec3 i = f_.position(fingerJoint(fg, Segment::Intermediate));
    const Vec3 d = f_.position(fingerJoint(fg, Segment::Distal));
    const Vec3 t = f_.position(fingerJoint(fg, Segment::Tip));
    const Vec3 axis = f_.palmLateral;
    const float a1 = std::max(0.f, signedAngleAbout(p - m, i - p, axis));
    const float a2 = std::max(0.f, signedAngleAbout(i - p, d - i, axis));
    const float a3 = std::max(0.f, signedAngleAbout(d - i, t - d, axis));
    f_.curl[fi] = clamp01(curlFilters_[fi].filter((a1 + a2 + a3) / kRange, dt));
  }
  // Thumb: unsigned bend at MCP and IP (its flexion axis is not aligned with the palm).
  constexpr float kThumbRange = degToRad(60.f + 80.f);
  const Vec3 m = f_.position(Joint::ThumbMetacarpal);
  const Vec3 p = f_.position(Joint::ThumbProximal);
  const Vec3 d = f_.position(Joint::ThumbDistal);
  const Vec3 t = f_.position(Joint::ThumbTip);
  const float bend = angleBetween(p - m, d - p) + angleBetween(d - p, t - d);
  f_.curl[0] = clamp01(curlFilters_[0].filter(bend / kThumbRange, dt));
}

void HandTracker::computeSpread() {
  for (int fi = 1; fi < kFingerCount - 1; ++fi) {
    const auto a = static_cast<Finger>(fi);
    const auto b = static_cast<Finger>(fi + 1);
    const Vec3 da = f_.position(fingerJoint(a, Segment::Intermediate)) - f_.position(fingerJoint(a, Segment::Proximal));
    const Vec3 db = f_.position(fingerJoint(b, Segment::Intermediate)) - f_.position(fingerJoint(b, Segment::Proximal));
    f_.spread[fi] = std::fabs(signedAngleAbout(da, db, f_.palmNormal));
  }
  f_.spread[0] = 0.f;
  f_.spread[kFingerCount - 1] = 0.f;
}

void HandTracker::computePinch(float dt) {
  const Vec3 t0 = f_.position(Joint::ThumbDistal);
  const Vec3 t1 = f_.position(Joint::ThumbTip);
  const float rThumb = f_.radii[jointIndex(Joint::ThumbTip)];
  const float cThumb = std::min(f_.jointConfidence[jointIndex(Joint::ThumbTip)],
                                f_.jointConfidence[jointIndex(Joint::ThumbDistal)]);
  const float scale = std::max(f_.handScale, 0.3f);
  for (int fi = 1; fi < kFingerCount; ++fi) {
    const auto fg = static_cast<Finger>(fi);
    const Joint dj = fingerJoint(fg, Segment::Distal);
    const Joint tj = fingerJoint(fg, Segment::Tip);
    // Capsule-capsule surface distance between distal segments approximates skin contact.
    const float centre = segmentSegmentDistance(t0, t1, f_.position(dj), f_.position(tj));
    const float surface = centre - rThumb - f_.radii[jointIndex(tj)];
    const float scaled = std::max(surface / scale, -0.02f);
    f_.pinchDistance[fi] = pinchFilters_[fi].filter(scaled, dt);
    f_.pinchConfidence[fi] =
        std::min({cThumb, f_.jointConfidence[jointIndex(dj)], f_.jointConfidence[jointIndex(tj)]});
  }
  f_.pinchDistance[0] = 1.f;
  for (int fi = 1; fi < kFingerCount; ++fi) {
    float other = 1e9f;
    for (int g = 1; g < kFingerCount; ++g)
      if (g != fi) other = std::min(other, f_.pinchDistance[g]);
    f_.pinchExclusivity[fi] = clamp01((other - f_.pinchDistance[fi]) / 0.012f);
  }
}

void HandTracker::computeKinematics(float dt) {
  const Quat rot = palmRotation(f_.palmLateral, f_.palmNormal, f_.palmForward);
  const Quat inv = conjugate(rot);
  std::array<Vec3, kFingerCount> tipLocal;
  for (int fi = 0; fi < kFingerCount; ++fi)
    tipLocal[fi] = rotate(inv, f_.position(fingerJoint(static_cast<Finger>(fi), Segment::Tip)) - f_.palmPosition);

  if (haveKinematicsPrev_ && dt > 0.f) {
    constexpr float kCut = 8.f;
    const Vec3 v = (f_.palmPosition - prevPalmPos_) / dt;
    f_.palmVelocity = {velX_.filter(v.x, dt, kCut), velY_.filter(v.y, dt, kCut), velZ_.filter(v.z, dt, kCut)};
    f_.palmSpeed = length(f_.palmVelocity);
    const Quat dq = rot * conjugate(prevPalmRot_);
    const float angle = 2.f * std::acos(clampf(std::fabs(dq.w), 0.f, 1.f));
    f_.angularSpeed = angSpeed_.filter(angle / dt, dt, kCut);
    float art = 0.f;
    for (int fi = 0; fi < kFingerCount; ++fi) art += length(tipLocal[fi] - prevTipLocal_[fi]);
    f_.articulationSpeed = artSpeed_.filter(art / (kFingerCount * dt), dt, kCut);
  }
  prevPalmPos_ = f_.palmPosition;
  prevPalmRot_ = rot;
  prevTipLocal_ = tipLocal;
  haveKinematicsPrev_ = true;
}

void HandTracker::computeGrasp(double time, const Config& cfg) {
  // (a) power grasp: all four fingers curled.
  float minCurl = 1.f;
  for (int fi = 1; fi < kFingerCount; ++fi) minCurl = std::min(minCurl, f_.curl[fi]);
  const float powerGrip = smoothstep(0.45f, 0.7f, minCurl);
  // (b) thumb touching two or more fingers at once (mug handle, pen, wheel rim).
  std::array<float, 4> d{f_.pinchDistance[1], f_.pinchDistance[2], f_.pinchDistance[3], f_.pinchDistance[4]};
  std::sort(d.begin(), d.end());
  const float multiContact = 1.f - smoothstep(0.012f, 0.03f, d[1]);
  // (c) rigid-body motion: the hand rotates while the fingers do not move relative to the palm.
  const float rigid = smoothstep(degToRad(60.f), degToRad(150.f), f_.angularSpeed) *
                      (1.f - smoothstep(0.05f, 0.15f, f_.articulationSpeed));
  const float partialCurl = smoothstep(0.3f, 0.55f, minCurl);
  const float raw = std::max({powerGrip, multiContact, partialCurl * rigid});
  if (raw > 0.6f) lastGraspTime_ = time;
  // Hold-off: letting go produces finger motion that resembles a gesture onset.
  f_.graspScore = (time - lastGraspTime_) < cfg.graspHoldoff ? 1.f : raw;
}

void HandTracker::computeRays(float dt, const Config& cfg) {
  const Vec3 wrist = f_.position(Joint::Wrist);
  const Vec3 iprox = f_.position(Joint::IndexProximal);
  const Vec3 itip = f_.position(Joint::IndexTip);
  const Vec3 dir = normalize(normalize(itip - iprox) * 0.6f + normalize(iprox - wrist) * 0.4f);
  f_.indexRay.direction = normalize(indexDirFilter_.filter(dir, dt), dir);
  f_.indexRay.origin = indexOriginFilter_.filter(itip, dt);
  f_.indexRayValid = true;

  f_.shoulderRayValid = false;
  if (f_.head.valid) {
    const Vec3 up{0.f, 1.f, 0.f};
    Vec3 fwdH = f_.head.forward;
    fwdH.y = 0.f;
    fwdH = normalize(fwdH);
    const Vec3 right = normalize(cross(fwdH, up));
    const float side = f_.hand == Hand::Right ? 1.f : -1.f;
    const Vec3 shoulder = f_.head.pose.position + right * (cfg.shoulderOffset * side) - up * cfg.shoulderDrop;
    const Vec3 pinchPoint = lerp(f_.position(Joint::ThumbTip), itip, 0.5f);
    const Vec3 sdir = normalize(pinchPoint - shoulder);
    f_.shoulderRay.direction = normalize(shoulderDirFilter_.filter(sdir, dt), sdir);
    f_.shoulderRay.origin = shoulderOriginFilter_.filter(pinchPoint, dt);
    f_.shoulderRayValid = true;
  }
}

bool HandTracker::currentRay(Aim aim, Ray& out) const {
  if (aim == Aim::IndexRay && f_.indexRayValid) {
    out = f_.indexRay;
    return true;
  }
  if (aim == Aim::ShoulderRay) {
    if (f_.shoulderRayValid) {
      out = f_.shoulderRay;
      return true;
    }
    if (f_.indexRayValid) {  // no head pose: fall back
      out = f_.indexRay;
      return true;
    }
  }
  return false;
}

bool HandTracker::rayAt(Aim aim, double time, Ray& out) const {
  for (int k = 0; k < rayCount_; ++k) {
    const RaySample& r = rayHistory_[(rayHead_ - 1 - k + kRayHistory) % kRayHistory];
    // Newest sample at or before `time`; if history is shorter than requested, use the oldest.
    if (r.time <= time + 1e-6 || k == rayCount_ - 1) {
      if (aim == Aim::ShoulderRay && r.shoulderValid) {
        out = r.shoulder;
        return true;
      }
      if (r.indexValid) {
        out = r.index;
        return true;
      }
      break;
    }
  }
  return currentRay(aim, out);
}

bool HandTracker::stableRayBefore(Aim aim, double time, double earliest, float maxAngularSpeed, Ray& out) const {
  auto pick = [aim](const RaySample& r, Ray& o) {
    if (aim == Aim::ShoulderRay && r.shoulderValid) {
      o = r.shoulder;
      return true;
    }
    if (r.indexValid) {
      o = r.index;
      return true;
    }
    return false;
  };
  int k = 0;
  while (k < rayCount_ && rayHistory_[(rayHead_ - 1 - k + kRayHistory) % kRayHistory].time > time + 1e-6) ++k;
  if (k >= rayCount_) return currentRay(aim, out);
  for (; k < rayCount_ - 1; ++k) {
    const RaySample& cur = rayHistory_[(rayHead_ - 1 - k + kRayHistory) % kRayHistory];
    const RaySample& older = rayHistory_[(rayHead_ - 2 - k + kRayHistory) % kRayHistory];
    Ray a, b;
    if (older.time < earliest || !pick(cur, a) || !pick(older, b)) return pick(cur, out) || currentRay(aim, out);
    const double dt = cur.time - older.time;
    if (dt > 0.0 && angleBetween(a.direction, b.direction) / static_cast<float>(dt) < maxAngularSpeed)
      return pick(older, out);
  }
  return pick(rayHistory_[(rayHead_ - 1 - k + kRayHistory) % kRayHistory], out) || currentRay(aim, out);
}

}  // namespace detail
}  // namespace gc
