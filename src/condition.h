#pragma once
// Debounced boolean over a soft signal. One instance per (condition, hand).
//
//   Unarmed --(below exit for rearmTime)--> Ready --(above enter)--> Pending --(adaptive dwell)--> Active
//   Active --(below exit)--> Releasing --(releaseDwell)--> Refractory --(refractory)--> Ready / Unarmed
//   Low confidence freezes the machine; an Active condition is cancelled once the grace period expires.
// Conditions without an onset requirement skip Unarmed.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "filters.h"
#include "gesture_composer/gestures.h"

namespace gc {
namespace detail {

enum class CondState : uint8_t { Unarmed, Ready, Pending, Active, Releasing, Refractory };

struct ConditionRuntime {
  // persistent
  CondState state = CondState::Unarmed;
  bool initialised = false;
  bool suspended = false;
  double suspendStart = 0.0;
  double openStart = -1.0;      // Unarmed: when the signal dropped below exit
  double lastOpenTime = -1e9;   // last time the signal was observed below exit
  double pendingSince = 0.0;    // when the signal crossed enter (onset time)
  double activeSince = 0.0;
  double inactiveSince = -1e9;  // when the last activation ended
  double releaseStart = 0.0;
  double refractoryStart = 0.0;
  double quietTime = 0.0;       // last time the signal was not moving while inactive
  double activeQuietTime = 0.0; // last time the signal was not falling while active
  double lastTime = -1.0;
  float prevValue = 0.f;
  float peakRate = 0.f;
  float peakFallRate = 0.f;
  LowPass rateFilter;
  float rate = 0.f;
  uint32_t riseSerial = 0;      // token identifying the current physical activation

  // per-frame outputs
  float value = 0.f;
  float confidence = 0.f;
  bool active = false;
  bool pressed = false;
  bool released = false;
  bool cancelled = false;

  void reset() { *this = ConditionRuntime{}; }

  // Was this condition continuously active over [when - minHold, when]?
  bool activeAt(double when, double minHold) const {
    if (activeSince + minHold > when + 1e-6) return false;
    return active || inactiveSince >= when - 1e-6;
  }

  bool isActiveState() const { return state == CondState::Active || state == CondState::Releasing; }

  void update(float v, float conf, double t, const ConditionParams& p, bool onset, uint32_t& serialCounter) {
    pressed = released = cancelled = false;
    const bool wasActive = active;
    value = v;
    confidence = conf;
    if (!initialised) {
      initialised = true;
      state = onset ? CondState::Unarmed : CondState::Ready;
      prevValue = v;
      lastTime = t;
      quietTime = t;
    }
    const float dt = static_cast<float>(t - lastTime);
    if (dt < 0.f) return;  // non-monotonic time

    // ---- confidence gating: freeze, then cancel after grace ----------------------------------------
    if (conf < p.minConfidence) {
      if (!suspended) {
        suspended = true;
        suspendStart = t;
      }
      if (t - suspendStart > p.grace) {
        if (isActiveState()) cancelled = true;
        state = onset ? CondState::Unarmed : CondState::Ready;
        openStart = -1.0;
      } else if (state == CondState::Pending) {
        state = CondState::Ready;  // do not commit an activation we could not observe
      }
      active = isActiveState();
      if (wasActive && !active) inactiveSince = t;
      lastTime = t;
      rateFilter.reset();
      rate = 0.f;
      return;
    }
    if (suspended) {
      suspended = false;
      prevValue = v;  // do not compute a rate across the gap
    }

    if (dt > 0.f) rate = rateFilter.filter((v - prevValue) / dt, dt, 15.f);
    prevValue = v;
    lastTime = t;
    const float vEnter = v + rate * p.predict;

    // Up to a few transitions per frame so a decisive change commits in the frame it happens.
    for (int iter = 0; iter < 3; ++iter) {
      const CondState before = state;
      switch (state) {
        case CondState::Unarmed:
          if (v < p.exit) {
            if (openStart < 0.0) openStart = t;
            if (t - openStart >= p.rearmTime) {
              state = CondState::Ready;
              lastOpenTime = t;
            }
          } else {
            openStart = -1.0;
          }
          break;
        case CondState::Ready:
          if (v < p.exit) lastOpenTime = t;
          if (vEnter > p.enter) {
            state = CondState::Pending;
            pendingSince = t;
            peakRate = std::max(rate, 0.f);
          } else if (onset && t - lastOpenTime > p.onsetWindow) {
            state = CondState::Unarmed;  // hovering half-closed too long: require a fresh onset
            openStart = -1.0;
          }
          break;
        case CondState::Pending: {
          peakRate = std::max(peakRate, rate);
          if (vEnter <= p.enter) {
            state = CondState::Ready;
            break;
          }
          const float dwell = p.maxDwell * (1.f - clamp01(peakRate / std::max(p.rateRef, 1e-3f)));
          if (t - pendingSince >= dwell - 1e-6) {
            state = CondState::Active;
            activeSince = t;
            pressed = true;
            riseSerial = ++serialCounter;
          }
          break;
        }
        case CondState::Active:
          if (v < p.exit) {
            state = CondState::Releasing;
            releaseStart = t;
            peakFallRate = std::max(-rate, 0.f);
          }
          break;
        case CondState::Releasing: {
          peakFallRate = std::max(peakFallRate, -rate);
          // Symmetric with the press: a decisive opening releases at once, a drift must persist.
          const float dwell = p.releaseDwell * (1.f - clamp01(peakFallRate / std::max(p.rateRef, 1e-3f)));
          if (v >= p.exit) {
            state = CondState::Active;
          } else if (t - releaseStart >= dwell - 1e-6) {
            state = CondState::Refractory;
            refractoryStart = t;
            released = true;
          }
          break;
        }
        case CondState::Refractory:
          // The release itself is the "open" observation, so a quick re-close during the refractory
          // period fires as soon as it ends (fast double pinches) rather than being lost.
          if (t - refractoryStart >= p.refractory - 1e-6) {
            state = CondState::Ready;
            lastOpenTime = std::max(lastOpenTime, refractoryStart);
          }
          break;
      }
      if (state == before) break;
      if (pressed || released) break;  // one edge per frame
    }

    if ((state == CondState::Ready || state == CondState::Unarmed) && std::fabs(rate) < 1.5f) quietTime = t;
    if (state == CondState::Active && rate > -1.5f) activeQuietTime = t;
    active = isActiveState();
    if (wasActive && !active) inactiveSince = t;
  }
};

}  // namespace detail
}  // namespace gc
