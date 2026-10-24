#include "gesture_composer/types.h"

namespace gc {

const char* toString(Finger f) {
  switch (f) {
    case Finger::Thumb: return "thumb";
    case Finger::Index: return "index";
    case Finger::Middle: return "middle";
    case Finger::Ring: return "ring";
    case Finger::Little: return "little";
  }
  return "?";
}

const char* toString(Hand h) { return h == Hand::Left ? "left" : "right"; }

const char* toString(Joint j) {
  static const char* const kNames[] = {
      "Palm", "Wrist",
      "ThumbMetacarpal", "ThumbProximal", "ThumbDistal", "ThumbTip",
      "IndexMetacarpal", "IndexProximal", "IndexIntermediate", "IndexDistal", "IndexTip",
      "MiddleMetacarpal", "MiddleProximal", "MiddleIntermediate", "MiddleDistal", "MiddleTip",
      "RingMetacarpal", "RingProximal", "RingIntermediate", "RingDistal", "RingTip",
      "LittleMetacarpal", "LittleProximal", "LittleIntermediate", "LittleDistal", "LittleTip",
      "ForearmWrist", "ForearmArm"};
  const auto i = static_cast<size_t>(j);
  return i < kJointCount ? kNames[i] : "?";
}

}  // namespace gc
