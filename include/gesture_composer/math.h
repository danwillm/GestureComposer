#pragma once
// Minimal, dependency-free math used across GestureComposer.
// Conventions: right-handed, +Y up, -Z forward, metres (OpenXR convention).

#include <cmath>

namespace gc {

struct Vec3 {
  float x = 0.f, y = 0.f, z = 0.f;
};

struct Quat {
  float x = 0.f, y = 0.f, z = 0.f, w = 1.f;
};

struct Pose {
  Vec3 position;
  Quat orientation;
};

struct Ray {
  Vec3 origin;
  Vec3 direction{0.f, 0.f, -1.f};
};

constexpr float kPi = 3.14159265358979323846f;
constexpr float degToRad(float d) { return d * (kPi / 180.f); }
constexpr float radToDeg(float r) { return r * (180.f / kPi); }

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline Vec3 operator/(Vec3 a, float s) { return a * (1.f / s); }
inline Vec3& operator+=(Vec3& a, Vec3 b) { return a = a + b; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { return a = a - b; }

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float lengthSq(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline float distance(Vec3 a, Vec3 b) { return length(a - b); }
inline Vec3 normalize(Vec3 a, Vec3 fallback = {0.f, 0.f, -1.f}) {
  const float l = length(a);
  return l > 1e-8f ? a / l : fallback;
}
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

inline float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Hermite smoothstep. Works for edge0 > edge1 too (gives a falling ramp).
inline float smoothstep(float edge0, float edge1, float x) {
  if (edge0 == edge1) return x < edge0 ? 0.f : 1.f;
  const float t = clamp01((x - edge0) / (edge1 - edge0));
  return t * t * (3.f - 2.f * t);
}

// Unsigned angle between two vectors, radians.
inline float angleBetween(Vec3 a, Vec3 b) {
  const float d = dot(normalize(a), normalize(b));
  return std::acos(clampf(d, -1.f, 1.f));
}

// Signed angle from a to b about `axis`, after projecting both onto the plane orthogonal to axis.
inline float signedAngleAbout(Vec3 a, Vec3 b, Vec3 axis) {
  const Vec3 n = normalize(axis);
  const Vec3 pa = a - n * dot(a, n);
  const Vec3 pb = b - n * dot(b, n);
  return std::atan2(dot(cross(pa, pb), n), dot(pa, pb));
}

inline Quat operator*(Quat a, Quat b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Quat normalize(Quat q) {
  const float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (l < 1e-8f) return {};
  const float s = 1.f / l;
  return {q.x * s, q.y * s, q.z * s, q.w * s};
}
inline Vec3 rotate(Quat q, Vec3 v) {
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = cross(u, v) * 2.f;
  return v + t * q.w + cross(u, t);
}
inline Quat quatFromAxisAngle(Vec3 axis, float radians) {
  const Vec3 n = normalize(axis);
  const float s = std::sin(radians * 0.5f);
  return {n.x * s, n.y * s, n.z * s, std::cos(radians * 0.5f)};
}

// Rotation whose local axes map to the given orthonormal world axes.
inline Quat quatFromBasis(Vec3 x, Vec3 y, Vec3 z) {
  const float m00 = x.x, m11 = y.y, m22 = z.z;
  const float tr = m00 + m11 + m22;
  Quat q;
  if (tr > 0.f) {
    const float s = std::sqrt(tr + 1.f) * 2.f;
    q = {(y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s, 0.25f * s};
  } else if (m00 > m11 && m00 > m22) {
    const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
    q = {0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s, (y.z - z.y) / s};
  } else if (m11 > m22) {
    const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
    q = {(y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s, (z.x - x.z) / s};
  } else {
    const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
    q = {(z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s, (x.y - y.x) / s};
  }
  return normalize(q);
}

// Rotation mapping local -Z to `forward` and local +Y (as close as possible) to `up`.
inline Quat quatLookRotation(Vec3 forward, Vec3 up) {
  const Vec3 z = -normalize(forward);
  Vec3 x = cross(up, z);
  if (lengthSq(x) < 1e-10f) x = cross(Vec3{1.f, 0.f, 0.f}, z);
  x = normalize(x);
  const Vec3 y = cross(z, x);
  return quatFromBasis(x, y, z);
}

// Closest distance between segments [p0,p1] and [q0,q1].
inline float segmentSegmentDistance(Vec3 p0, Vec3 p1, Vec3 q0, Vec3 q1) {
  const Vec3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
  const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  float s = 0.f, t = 0.f;
  const float eps = 1e-12f;
  if (a <= eps && e <= eps) return length(r);
  if (a <= eps) {
    t = clamp01(f / e);
  } else {
    const float c = dot(d1, r);
    if (e <= eps) {
      s = clamp01(-c / a);
    } else {
      const float b = dot(d1, d2);
      const float denom = a * e - b * b;
      s = denom > eps ? clamp01((b * f - c * e) / denom) : 0.f;
      t = (b * s + f) / e;
      if (t < 0.f) {
        t = 0.f;
        s = clamp01(-c / a);
      } else if (t > 1.f) {
        t = 1.f;
        s = clamp01((b - c) / a);
      }
    }
  }
  return length((p0 + d1 * s) - (q0 + d2 * t));
}

}  // namespace gc
