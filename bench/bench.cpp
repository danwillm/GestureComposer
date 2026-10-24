// Measures Composer::update() cost with a realistic gesture set (both hands, head, 32 gestures).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "gesture_composer/gesture_composer.h"
#include "gesture_composer/testing/synthetic_hand.h"

using namespace gc;
using namespace gc::gestures;
using namespace std::chrono_literals;
using gc::testing::HandPose;

int main() {
  Composer c;
  const Finger fingers[] = {Finger::Index, Finger::Middle, Finger::Ring, Finger::Little};
  auto relaxed = !grasping() && !occupied();
  auto palmToHead = palmFacing(Target::Head) && inView() && still() && extended(Finger::Middle, Finger::Ring, Finger::Little);
  auto pointPose = extended(Finger::Index) && curled(Finger::Middle, Finger::Ring, Finger::Little);
  int n = 0;
  for (Finger f : fingers) {
    c.add(gesture("press").trigger(press(pinch(f))).gate(relaxed).exclusive("t")); ++n;
    c.add(gesture("click").trigger(click(pinch(f)))); ++n;
    c.add(gesture("hold").trigger(hold(pinch(f), 600ms))); ++n;
    c.add(gesture("drag").trigger(active(pinch(f))).aim(Aim::ShoulderRay)); ++n;
    c.add(gesture("dbl").trigger(repeat(click(pinch(f)), 2).within(400ms))); ++n;
    c.add(gesture("menu").trigger(press(pinch(f))).arm(palmToHead, 250ms).gate(relaxed)); ++n;
    c.add(gesture("both").trigger(both(active(pinch(f))))); ++n;
  }
  c.add(gesture("aim").trigger(active(pointPose)).aim(Aim::IndexRay)); ++n;
  c.add(gesture("go").trigger(release(pointPose)).arm(pointPose, 100ms).aim(Aim::IndexRay, Stabilize::RewindToOnset)); ++n;
  c.add(gesture("seq").trigger(seq(click(pinch(Finger::Middle)), click(pinch(Finger::Ring))).within(800ms))); ++n;
  c.add(gesture("chord").trigger(chord(press(pinch(Finger::Index)), press(pinch(Finger::Middle))))); ++n;
  const CompileResult& r = c.compile();
  if (!r.ok) {
    for (const auto& e : r.errors) std::printf("error: %s\n", e.c_str());
    return 1;
  }

  const int frames = 20000;
  std::vector<HandSample> left(frames), right(frames);
  std::vector<HeadSample> heads(frames);
  double t = 0.0;
  for (int i = 0; i < frames; ++i) {
    t += 1.0 / 90.0;
    const float a = 0.5f + 0.5f * std::sin(i * 0.05f);
    const Finger f = fingers[(i / 150) % 4];
    HandPose p;
    p.curl = {0.f, 0.08f, 0.1f, 0.1f, 0.1f};
    p.pinchFinger = f;
    p.pinchAmount = a;
    const Pose w = gc::testing::wristPose({0.1f, 1.3f, -0.35f}, {0, 0, -1}, {0, -1, 0});
    left[i] = gc::testing::makeHand(Hand::Left, w, p, t);
    right[i] = gc::testing::makeHand(Hand::Right, w, p, t);
    heads[i].time = t;
    heads[i].pose.position = {0.f, 1.6f, 0.f};
  }

  std::vector<double> us(frames);
  size_t events = 0;
  for (int i = 0; i < frames; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    c.update(&left[i], &right[i], &heads[i]);
    const auto t1 = std::chrono::steady_clock::now();
    us[i] = std::chrono::duration<double, std::micro>(t1 - t0).count();
    events += c.events().size();
  }
  std::vector<double> sorted(us.begin() + 100, us.end());
  std::sort(sorted.begin(), sorted.end());
  double sum = 0;
  for (double v : sorted) sum += v;
  std::printf("%d gestures, both hands + head, %d frames, %zu events\n", n, frames, events);
  std::printf("update(): mean %.2f us, median %.2f us, p99 %.2f us, max %.2f us\n", sum / sorted.size(),
              sorted[sorted.size() / 2], sorted[sorted.size() * 99 / 100], sorted.back());
  return 0;
}
