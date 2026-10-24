#pragma once
// OpenXR XR_EXT_hand_tracking -> gc::HandSample. Header-only; include after <openxr/openxr.h> is available.
//
//   XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT];
//   XrHandJointLocationsEXT locations{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
//   locations.jointCount = XR_HAND_JOINT_COUNT_EXT; locations.jointLocations = joints;
//   xrLocateHandJointsEXT(tracker, &locateInfo, &locations);
//   gc::HandSample s;
//   gc::openxr::convert(locations, gc::Hand::Right, xrTimeToSeconds(time), s);

#include <openxr/openxr.h>

#include "gesture_composer/types.h"

namespace gc {
namespace openxr {

// The canonical gc::Joint order matches XrHandJointEXT for the first 26 joints.
static_assert(static_cast<int>(Joint::Palm) == XR_HAND_JOINT_PALM_EXT, "joint order mismatch");
static_assert(static_cast<int>(Joint::LittleTip) == XR_HAND_JOINT_LITTLE_TIP_EXT, "joint order mismatch");

inline void convert(const XrHandJointLocationEXT* joints, uint32_t count, bool isActive, Hand hand, double timeSeconds,
                    HandSample& out) {
  out = HandSample{};
  out.hand = hand;
  out.time = timeSeconds;
  out.tracked = isActive != 0;
  const uint32_t n = count < kCoreJointCount ? count : static_cast<uint32_t>(kCoreJointCount);
  for (uint32_t i = 0; i < n; ++i) {
    const XrHandJointLocationEXT& j = joints[i];
    JointSample& js = out.joints[i];
    js.pose.position = {j.pose.position.x, j.pose.position.y, j.pose.position.z};
    js.pose.orientation = {j.pose.orientation.x, j.pose.orientation.y, j.pose.orientation.z, j.pose.orientation.w};
    js.radius = j.radius;
    const bool posValid = (j.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
    const bool posTracked = (j.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
    if (posValid) out.validMask |= 1u << i;
    // Valid-but-not-tracked joints are inferred by the runtime: usable, but trusted less.
    js.confidence = posTracked ? 1.f : (posValid ? 0.5f : 0.f);
  }
}

inline void convert(const XrHandJointLocationsEXT& locations, Hand hand, double timeSeconds, HandSample& out) {
  convert(locations.jointLocations, locations.jointCount, locations.isActive != 0, hand, timeSeconds, out);
}

inline double timeToSeconds(XrTime t) { return static_cast<double>(t) * 1e-9; }

}  // namespace openxr
}  // namespace gc
