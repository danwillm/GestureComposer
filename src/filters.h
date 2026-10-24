#pragma once
// One Euro filter (Casiez, Roussel, Vogel, CHI 2012) and a first-order low-pass.

#include <cmath>

#include "gesture_composer/math.h"

namespace gc {
namespace detail {

inline float smoothingAlpha(float cutoffHz, float dt) {
  const float tau = 1.f / (2.f * kPi * cutoffHz);
  return 1.f / (1.f + tau / dt);
}

struct LowPass {
  float y = 0.f;
  bool init = false;
  float filter(float x, float dt, float cutoffHz) {
    if (!init || dt <= 0.f) {
      if (!init) y = x;
      init = true;
      return y;
    }
    y += smoothingAlpha(cutoffHz, dt) * (x - y);
    return y;
  }
  void reset() { init = false; }
};

struct OneEuro {
  float minCutoff = 1.f;
  float beta = 0.f;
  float dCutoff = 1.f;
  float x = 0.f, dx = 0.f;
  bool init = false;

  OneEuro() = default;
  OneEuro(float minCutoffHz, float b, float dCutoffHz = 1.f) : minCutoff(minCutoffHz), beta(b), dCutoff(dCutoffHz) {}

  float filter(float v, float dt) {
    if (!init) {
      x = v;
      dx = 0.f;
      init = true;
      return x;
    }
    if (dt <= 0.f) return x;
    const float d = (v - x) / dt;
    dx += smoothingAlpha(dCutoff, dt) * (d - dx);
    const float cutoff = minCutoff + beta * std::fabs(dx);
    x += smoothingAlpha(cutoff, dt) * (v - x);
    return x;
  }
  void reset() { init = false; }
};

// Vector variant: the adaptive cutoff is driven by the speed (magnitude of the derivative).
struct OneEuroVec3 {
  float minCutoff = 1.f;
  float beta = 0.f;
  float dCutoff = 1.f;
  Vec3 x, dx;
  bool init = false;

  OneEuroVec3() = default;
  OneEuroVec3(float minCutoffHz, float b, float dCutoffHz = 1.f) : minCutoff(minCutoffHz), beta(b), dCutoff(dCutoffHz) {}

  Vec3 filter(Vec3 v, float dt) {
    if (!init) {
      x = v;
      dx = {};
      init = true;
      return x;
    }
    if (dt <= 0.f) return x;
    const Vec3 d = (v - x) / dt;
    dx += (d - dx) * smoothingAlpha(dCutoff, dt);
    const float cutoff = minCutoff + beta * length(dx);
    x += (v - x) * smoothingAlpha(cutoff, dt);
    return x;
  }
  void reset() { init = false; }
};

}  // namespace detail
}  // namespace gc
