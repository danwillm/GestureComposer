#pragma once
// Per-hand conditioning (validity, dropout hold, filtering, hand scale) and feature extraction.

#include <array>

#include "filters.h"
#include "gesture_composer/composer.h"
#include "gesture_composer/features.h"

namespace gc {
namespace detail {

constexpr float kRefPalmWidth = 0.070f;   // index knuckle <-> little knuckle, reference adult hand
constexpr float kRefPalmLength = 0.095f;  // wrist <-> middle knuckle

class HandTracker {
 public:
  explicit HandTracker(Hand hand);

  void reset();
  // sample may be null (hand unavailable). `time` is the frame time.
  void update(const HandSample* sample, double time, const HeadInfo& head, FeatureMask mask, const Config& cfg);

  const HandFeatures& features() const { return f_; }
  HandFeatures& features() { return f_; }

  // Ray at (or just before) the given time, from the history ring. Falls back to the current ray.
  bool rayAt(Aim aim, double time, Ray& out) const;
  bool currentRay(Aim aim, Ray& out) const;
  // Walks back from `time` to the most recent moment the ray was stable (angular speed below
  // maxAngularSpeed), not earlier than `earliest`: the ray from before the confirming motion began.
  bool stableRayBefore(Aim aim, double time, double earliest, float maxAngularSpeed, Ray& out) const;

 private:
  void initDefaults();
  void updateScale(double time);
  void computePalmFrame(float dt);
  void computeCurl(float dt);
  void computeSpread();
  void computePinch(float dt);
  void computeKinematics(float dt);
  void computeGrasp(double time, const Config& cfg);
  void computeRays(float dt, const Config& cfg);

  HandFeatures f_;
  double prevTime_ = -1.0;
  bool havePrevFrame_ = false;

  // scale estimation
  static constexpr int kScaleSamples = 31;
  std::array<float, kScaleSamples> scaleSamples_{};
  int scaleCount_ = 0;
  double lastScaleSampleTime_ = -1.0;

  OneEuroVec3 normalFilter_{1.5f, 2.f};
  OneEuroVec3 forwardFilter_{1.5f, 2.f};
  std::array<OneEuro, kFingerCount> curlFilters_;
  std::array<OneEuro, kFingerCount> pinchFilters_;

  // kinematics
  Vec3 prevPalmPos_;
  Quat prevPalmRot_;
  std::array<Vec3, kFingerCount> prevTipLocal_{};
  bool haveKinematicsPrev_ = false;
  LowPass velX_, velY_, velZ_, angSpeed_, artSpeed_;

  double lastGraspTime_ = -1e9;

  OneEuroVec3 indexDirFilter_{1.0f, 1.5f};
  OneEuroVec3 indexOriginFilter_{1.0f, 10.f};
  OneEuroVec3 shoulderDirFilter_{1.0f, 1.5f};
  OneEuroVec3 shoulderOriginFilter_{1.0f, 10.f};

  struct RaySample {
    double time = -1.0;
    Ray index, shoulder;
    bool indexValid = false, shoulderValid = false;
  };
  static constexpr int kRayHistory = 48;
  std::array<RaySample, kRayHistory> rayHistory_{};
  int rayHead_ = 0;
  int rayCount_ = 0;
};

}  // namespace detail
}  // namespace gc
