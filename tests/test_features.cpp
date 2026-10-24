#include "sim.h"
#include "test_framework.h"

using namespace gc;
using namespace gc::gestures;
using gc::testing::HandPose;
using gctest::Sim;

TEST_CASE("math: segment distance and rotations") {
  CHECK_NEAR(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {0.5f, 1, 0}, {0.5f, 2, 0}), 1.0, 1e-5);
  CHECK_NEAR(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}), 1.0, 1e-5);
  CHECK_NEAR(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {0.5f, -1, 1}, {0.5f, 1, 1}), 1.0, 1e-5);
  const Quat q = quatLookRotation({1, 0, 0}, {0, 1, 0});
  const Vec3 f = rotate(q, {0, 0, -1});
  CHECK_NEAR(f.x, 1.0, 1e-5);
  const Vec3 u = rotate(q, {0, 1, 0});
  CHECK_NEAR(u.y, 1.0, 1e-5);
  const Quat r = quatFromAxisAngle({0, 1, 0}, kPi / 2);
  const Vec3 v = rotate(r, {1, 0, 0});
  CHECK_NEAR(v.z, -1.0, 1e-5);
}

TEST_CASE("features: curl, palm frame and pinch for both hands") {
  Composer c;
  c.track(curl(Finger::Index));
  c.track(pinch(Finger::Index));
  c.track(palmFacing(Target::Down));
  c.track(spread(Finger::Index));
  Sim sim(c);
  for (int h = 0; h < 2; ++h) {
    sim.hand[h].pose = HandPose::open();
    sim.run(0.3);
    const HandFeatures& f = c.features(static_cast<Hand>(h));
    CHECK(f.tracked);
    CHECK(f.curl[1] < 0.08f);
    CHECK(f.palmNormal.y < -0.95f);  // palm down
    sim.hand[h].pose = HandPose::fist();
    sim.run(0.4);
    CHECK(c.features(static_cast<Hand>(h)).curl[1] > 0.8f);
    CHECK(c.features(static_cast<Hand>(h)).curl[3] > 0.8f);
    sim.hand[h].pose = HandPose::pinch(Finger::Index);
    sim.run(0.4);
    CHECK(c.features(static_cast<Hand>(h)).pinchDistance[1] < 0.004f);
    CHECK(c.features(static_cast<Hand>(h)).pinchDistance[2] > 0.02f);
  }
}

TEST_CASE("features: hand scale is measured and normalises distances") {
  Composer c;
  c.track(pinch(Finger::Index));
  Sim sim(c);
  sim.hand[1].pose = HandPose::open();
  sim.hand[1].pose.scale = 0.8f;
  sim.run(2.0);
  const HandFeatures& f = c.features(Hand::Right);
  CHECK(f.scaleLocked);
  CHECK_NEAR(f.handScale, 0.8, 0.03);
  CHECK_NEAR(c.features(Hand::Left).handScale, 1.0, 0.03);
}

TEST_CASE("features: palm facing head signal") {
  Composer c;
  Signal facing = palmFacing(Target::Head);
  c.track(facing);
  Sim sim(c);
  sim.palmToHead(1);
  sim.run(0.3);
  CHECK(c.signal(facing, Hand::Right).value > 0.95f);
  CHECK(c.signal(facing, Hand::Right).confidence > 0.9f);
  sim.palmDown(1);
  sim.run(0.3);
  CHECK(c.signal(facing, Hand::Right).value < 0.05f);
  sim.headPresent = false;
  sim.run(0.05);
  CHECK(c.signal(facing, Hand::Right).confidence > 0.9f);  // missing head data is held briefly...
  sim.run(0.15);
  CHECK(c.signal(facing, Hand::Right).confidence == 0.f);  // ...then head-relative signals lose confidence
}

TEST_CASE("features: grasp detector fires for a power grip, not for point or pinch") {
  Composer c;
  Signal g = grasping();
  c.track(g);
  Sim sim(c);
  sim.hand[1].pose = HandPose::grip();
  sim.run(0.4);
  CHECK(c.signal(g, Hand::Right).value > 0.9f);
  sim.hand[1].pose = HandPose::point();
  sim.run(0.8);  // past the hold-off
  CHECK(c.signal(g, Hand::Right).value < 0.3f);
  sim.hand[1].pose = HandPose::pinch(Finger::Index);
  sim.run(0.5);
  CHECK(c.signal(g, Hand::Right).value < 0.3f);
}

TEST_CASE("features: only the features registered gestures need are computed") {
  Composer c;
  GestureId p = c.add(gesture("p").trigger(press(pinch(Finger::Index))));
  GestureId g = c.add(gesture("g").trigger(press(pinch(Finger::Middle))).gate(!grasping()));
  c.compile();
  CHECK((c.featureMask() & Features::Grasp) != 0);
  c.setEnabled(g, false);
  CHECK((c.featureMask() & Features::Grasp) == 0);
  CHECK((c.featureMask() & Features::Pinch) != 0);
  CHECK((c.featureMask() & Features::Spread) == 0);
  c.setEnabled(p, false);
  CHECK(c.featureMask() == 0);
}
