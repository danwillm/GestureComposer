// The gestures from the proposal, driven by a scripted synthetic hand.

#include <cstdio>

#include "gesture_composer/gesture_composer.h"
#include "gesture_composer/testing/synthetic_hand.h"

using namespace gc;
using namespace gc::gestures;
using namespace std::chrono_literals;
using gc::testing::HandPose;

int main() {
  Composer composer;

  // Reusable building blocks.
  auto indexPinch = pinch(Finger::Index);
  auto relaxed = !grasping() && !occupied();
  auto palmToHead = palmFacing(Target::Head) && inView(40.f) && still() &&
                    extended(Finger::Middle, Finger::Ring, Finger::Little);
  auto pointPose = extended(Finger::Index) && curled(Finger::Middle, Finger::Ring, Finger::Little);

  // 1. Index-pinch click.
  const GestureId select = composer.add(gesture("select").trigger(press(indexPinch)).gate(relaxed));

  // 2. Point to aim; curling the index finger confirms with the pre-confirm ray.
  const GestureId aim = composer.add(
      gesture("teleport.aim").trigger(active(pointPose).minDuration(120ms)).aim(Aim::IndexRay).updates(false));
  const GestureId teleport = composer.add(gesture("teleport.go")
                                              .trigger(release(pointPose))
                                              .arm(pointPose, 120ms)
                                              .aim(Aim::IndexRay, Stabilize::RewindToOnset));

  // 3. Left palm toward head, then index pinch: menu. Restricted to the left hand; on that hand it
  //    outranks select because it is more specific. The right hand doing the same thing just selects.
  const GestureId menu = composer.add(
      gesture("menu").hands(Hands::Left).trigger(press(indexPinch)).arm(palmToHead, 250ms).gate(relaxed));

  // 4. Thumb to the other fingers, mutually exclusive.
  for (Finger f : {Finger::Middle, Finger::Ring, Finger::Little})
    composer.add(gesture(std::string("tool.") + toString(f)).trigger(press(pinch(f))).exclusive("thumb_pinch"));

  // Composition.
  composer.add(gesture("undo").trigger(repeat(click(indexPinch), 2).within(350ms)));

  const CompileResult& r = composer.compile();
  for (const auto& e : r.errors) std::printf("error: %s\n", e.c_str());
  for (const auto& w : r.warnings) std::printf("warning: %s\n", w.c_str());
  if (!r.ok) return 1;

  // ---- scripted session --------------------------------------------------------------------------
  double t = 0.0;
  const double dt = 1.0 / 90.0;
  HeadSample head;
  head.pose.position = {0.f, 1.6f, 0.f};
  Hand scripted = Hand::Right;  // which hand the script is driving
  Vec3 wrist{0.1f, 1.4f, -0.3f};
  Vec3 fingerDir{0.f, 0.f, -1.f}, palmNormal{0.f, -1.f, 0.f};
  HandPose pose = HandPose::relaxed();

  auto frame = [&]() {
    t += dt;
    head.time = t;
    const HandSample hand = gc::testing::makeHand(scripted, gc::testing::wristPose(wrist, fingerDir, palmNormal), pose, t);
    if (scripted == Hand::Left)
      composer.update(&hand, nullptr, &head);
    else
      composer.update(nullptr, &hand, &head);
    for (const Event& e : composer.events()) {
      static const char* phases[] = {"Armed", "Disarmed", "Began", "Updated", "Ended", "Cancelled", "Triggered"};
      std::printf("%6.3fs  %-14s %-9s %s", e.time, composer.name(e.id), phases[static_cast<int>(e.phase)],
                  toString(e.hand));
      if (e.rayValid) std::printf("  ray dir (%.2f %.2f %.2f)", e.ray.direction.x, e.ray.direction.y, e.ray.direction.z);
      std::printf("\n");
    }
  };
  auto hold = [&](double s) {
    for (int i = 0; i < static_cast<int>(s / dt); ++i) frame();
  };
  auto blend = [&](HandPose target, double s) {
    const HandPose from = pose;
    const int n = static_cast<int>(s / dt);
    for (int i = 1; i <= n; ++i) {
      const float a = static_cast<float>(i) / n;
      pose = a < 1.f ? from : target;
      for (int k = 0; k < kFingerCount; ++k) pose.curl[k] = lerpf(from.curl[k], target.curl[k], a);
      pose.pinchFinger = target.pinchFinger != Finger::Thumb ? target.pinchFinger : from.pinchFinger;
      pose.pinchAmount = lerpf(from.pinchFinger == pose.pinchFinger ? from.pinchAmount : 0.f, target.pinchAmount, a);
      frame();
    }
  };

  std::printf("-- index pinch (select)\n");
  hold(0.5);
  blend(HandPose::pinch(Finger::Index), 0.08);
  hold(0.15);
  blend(HandPose::relaxed(), 0.08);
  hold(0.5);

  for (Hand h : {Hand::Right, Hand::Left}) {
    std::printf("-- %s palm to head, then pinch (%s)\n", toString(h),
                h == Hand::Left ? "menu" : "select: menu is left-hand only");
    scripted = h;
    wrist = {h == Hand::Left ? -0.1f : 0.1f, 1.4f, -0.3f};
    palmNormal = normalize(head.pose.position - wrist);
    fingerDir = normalize(Vec3{0.f, 1.f, 0.f} - palmNormal * palmNormal.y);
    hold(0.4);
    blend(HandPose::pinch(Finger::Index), 0.08);
    hold(0.15);
    blend(HandPose::relaxed(), 0.08);
    hold(0.3);
    palmNormal = {0.f, -1.f, 0.f};
    fingerDir = {0.f, 0.f, -1.f};
    hold(0.5);
  }
  scripted = Hand::Right;
  wrist = {0.1f, 1.4f, -0.3f};

  std::printf("-- thumb to middle, ring\n");
  for (Finger f : {Finger::Middle, Finger::Ring}) {
    blend(HandPose::pinch(f), 0.08);
    hold(0.15);
    blend(HandPose::relaxed(), 0.08);
    hold(0.3);
  }

  std::printf("-- point, then curl the index finger to teleport\n");
  blend(HandPose::point(), 0.15);
  hold(0.5);
  HandPose curledIndex = HandPose::point();
  curledIndex.curl[1] = 0.8f;
  blend(curledIndex, 0.15);
  hold(0.3);
  blend(HandPose::relaxed(), 0.15);
  hold(0.3);

  (void)select;
  (void)aim;
  (void)teleport;
  (void)menu;
  return 0;
}
