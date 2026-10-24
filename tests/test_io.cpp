#include <cstring>

#include "gesture_composer/adapters/openxr.h"
#include "gesture_composer/adapters/visionos.h"
#include "sim.h"
#include "test_framework.h"

using namespace gc;
using namespace gc::gestures;
using gc::testing::HandPose;
using gctest::Sim;

TEST_CASE("recorder: replay reproduces identical events") {
  Recorder rec;
  std::vector<Event> live;
  {
    Composer c;
    c.add(gesture("select").trigger(press(pinch(Finger::Index))));
    c.add(gesture("drag").trigger(active(pinch(Finger::Middle))));
    Sim sim(c);
    sim.noise = 0.001f;
    for (int i = 0; i < 120; ++i) {
      const float a = 0.5f + 0.5f * std::sin(i * 0.15f);
      sim.hand[1].pose = gctest::mix(HandPose::relaxed(), HandPose::pinch(i < 60 ? Finger::Index : Finger::Middle), a);
      sim.t += sim.dt;
      sim.head.time = sim.t;
      HandSample l = sim.sample(0), r = sim.sample(1);
      rec.record(&l, &r, &sim.head);
      c.update(&l, &r, &sim.head);
      for (const Event& e : c.events()) live.push_back(e);
    }
  }
  CHECK(!live.empty());
  Replayer rp;
  CHECK(rp.load(rec.data().data(), rec.data().size()));
  Composer c2;
  c2.add(gesture("select").trigger(press(pinch(Finger::Index))));
  c2.add(gesture("drag").trigger(active(pinch(Finger::Middle))));
  std::vector<Event> replayed;
  rp.play(c2, [&](const RecordedFrame&) {
    for (const Event& e : c2.events()) replayed.push_back(e);
  });
  CHECK_EQ(replayed.size(), live.size());
  for (size_t i = 0; i < std::min(replayed.size(), live.size()); ++i) {
    CHECK(replayed[i].id == live[i].id);
    CHECK(replayed[i].phase == live[i].phase);
    CHECK(replayed[i].time == live[i].time);
  }
}

TEST_CASE("visionOS adapter: maps ARKit joints and synthesises the palm") {
  const HandSample ref =
      gc::testing::makeHand(Hand::Right, gc::testing::wristPose({0.1f, 1.2f, -0.3f}, {0, 0, -1}, {0, -1, 0}),
                            HandPose::pinch(Finger::Middle), 1.0);
  float matrices[visionos::kARKitJointCount * 16] = {};
  bool tracked[visionos::kARKitJointCount] = {};
  for (int i = 0; i < visionos::kARKitJointCount; ++i) {
    const Joint j = visionos::kJointMap[i];
    float* m = matrices + i * 16;
    const Pose& p = ref[j].pose;
    const Vec3 x = rotate(p.orientation, {1, 0, 0}), y = rotate(p.orientation, {0, 1, 0}), z = rotate(p.orientation, {0, 0, 1});
    const float cols[16] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, z.x, z.y, z.z, 0, p.position.x, p.position.y, p.position.z, 1};
    std::memcpy(m, cols, sizeof(cols));
    tracked[i] = j != Joint::ForearmWrist && j != Joint::ForearmArm;
  }
  HandSample s;
  visionos::convert(matrices, tracked, true, Hand::Right, 1.0, s);
  CHECK(s.tracked);
  for (size_t i = 0; i < kCoreJointCount; ++i) CHECK(s.valid(static_cast<Joint>(i)));
  CHECK(!s.valid(Joint::ForearmArm));
  CHECK_NEAR(distance(s[Joint::MiddleTip].pose.position, ref[Joint::MiddleTip].pose.position), 0.0, 1e-5);
  CHECK_NEAR(distance(s[Joint::Palm].pose.position, ref[Joint::Palm].pose.position), 0.0, 1e-5);
  CHECK_NEAR(std::fabs(s[Joint::IndexTip].pose.orientation.w), std::fabs(ref[Joint::IndexTip].pose.orientation.w), 1e-4);

  // Features from the converted sample match the original.
  Composer a, b;
  a.track(pinch(Finger::Middle));
  b.track(pinch(Finger::Middle));
  HandSample s2 = s;
  s2.time = 1.0;
  a.update(1.0, nullptr, &ref, nullptr);
  b.update(1.0, nullptr, &s2, nullptr);
  CHECK_NEAR(a.features(Hand::Right).pinchDistance[2], b.features(Hand::Right).pinchDistance[2], 1e-4);
  CHECK_NEAR(a.features(Hand::Right).curl[2], b.features(Hand::Right).curl[2], 1e-4);
}

TEST_CASE("OpenXR adapter: flags map to validity and confidence") {
  XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT] = {};
  for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i) {
    joints[i].locationFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                              XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    joints[i].pose.orientation.w = 1.f;
    joints[i].pose.position.x = static_cast<float>(i);
    joints[i].radius = 0.01f;
  }
  joints[XR_HAND_JOINT_RING_TIP_EXT].locationFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT;
  joints[XR_HAND_JOINT_LITTLE_TIP_EXT].locationFlags = 0;
  XrHandJointLocationsEXT locs{};
  locs.isActive = XR_TRUE;
  locs.jointCount = XR_HAND_JOINT_COUNT_EXT;
  locs.jointLocations = joints;
  HandSample s;
  openxr::convert(locs, Hand::Left, openxr::timeToSeconds(2500000000), s);
  CHECK(s.tracked);
  CHECK_NEAR(s.time, 2.5, 1e-9);
  CHECK(s.valid(Joint::IndexTip));
  CHECK(s.valid(Joint::RingTip));
  CHECK(!s.valid(Joint::LittleTip));
  CHECK_NEAR(s[Joint::RingTip].confidence, 0.5, 1e-6);
  CHECK_NEAR(s[Joint::IndexTip].pose.position.x, static_cast<float>(XR_HAND_JOINT_INDEX_TIP_EXT), 1e-6);
  CHECK(!s.valid(Joint::ForearmWrist));
}
