#pragma once
// Minimal stand-in for <openxr/openxr.h> so the adapter can be compiled and tested without the SDK.
// Values match the OpenXR 1.0 specification (XR_EXT_hand_tracking).

#include <cstdint>

typedef uint32_t XrBool32;
typedef int64_t XrTime;
typedef uint64_t XrSpaceLocationFlags;
typedef int32_t XrStructureType;
#define XR_TRUE 1
#define XR_FALSE 0

static const XrSpaceLocationFlags XR_SPACE_LOCATION_ORIENTATION_VALID_BIT = 0x00000001;
static const XrSpaceLocationFlags XR_SPACE_LOCATION_POSITION_VALID_BIT = 0x00000002;
static const XrSpaceLocationFlags XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT = 0x00000004;
static const XrSpaceLocationFlags XR_SPACE_LOCATION_POSITION_TRACKED_BIT = 0x00000008;

typedef struct XrVector3f { float x, y, z; } XrVector3f;
typedef struct XrQuaternionf { float x, y, z, w; } XrQuaternionf;
typedef struct XrPosef { XrQuaternionf orientation; XrVector3f position; } XrPosef;

typedef enum XrHandJointEXT {
  XR_HAND_JOINT_PALM_EXT = 0,
  XR_HAND_JOINT_WRIST_EXT = 1,
  XR_HAND_JOINT_THUMB_METACARPAL_EXT = 2,
  XR_HAND_JOINT_THUMB_PROXIMAL_EXT = 3,
  XR_HAND_JOINT_THUMB_DISTAL_EXT = 4,
  XR_HAND_JOINT_THUMB_TIP_EXT = 5,
  XR_HAND_JOINT_INDEX_METACARPAL_EXT = 6,
  XR_HAND_JOINT_INDEX_PROXIMAL_EXT = 7,
  XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT = 8,
  XR_HAND_JOINT_INDEX_DISTAL_EXT = 9,
  XR_HAND_JOINT_INDEX_TIP_EXT = 10,
  XR_HAND_JOINT_MIDDLE_METACARPAL_EXT = 11,
  XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT = 12,
  XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT = 13,
  XR_HAND_JOINT_MIDDLE_DISTAL_EXT = 14,
  XR_HAND_JOINT_MIDDLE_TIP_EXT = 15,
  XR_HAND_JOINT_RING_METACARPAL_EXT = 16,
  XR_HAND_JOINT_RING_PROXIMAL_EXT = 17,
  XR_HAND_JOINT_RING_INTERMEDIATE_EXT = 18,
  XR_HAND_JOINT_RING_DISTAL_EXT = 19,
  XR_HAND_JOINT_RING_TIP_EXT = 20,
  XR_HAND_JOINT_LITTLE_METACARPAL_EXT = 21,
  XR_HAND_JOINT_LITTLE_PROXIMAL_EXT = 22,
  XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT = 23,
  XR_HAND_JOINT_LITTLE_DISTAL_EXT = 24,
  XR_HAND_JOINT_LITTLE_TIP_EXT = 25
} XrHandJointEXT;
#define XR_HAND_JOINT_COUNT_EXT 26

typedef struct XrHandJointLocationEXT {
  XrSpaceLocationFlags locationFlags;
  XrPosef pose;
  float radius;
} XrHandJointLocationEXT;

typedef struct XrHandJointLocationsEXT {
  XrStructureType type;
  void* next;
  XrBool32 isActive;
  uint32_t jointCount;
  XrHandJointLocationEXT* jointLocations;
} XrHandJointLocationsEXT;
