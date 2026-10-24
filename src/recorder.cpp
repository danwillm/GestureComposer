#include "gesture_composer/recorder.h"

#include <cstdio>
#include <cstring>

namespace gc {

namespace {

constexpr char kMagic[4] = {'G', 'C', 'R', '1'};
enum : uint8_t { kHasLeft = 1, kHasRight = 2, kHasHead = 4, kHasGaze = 8 };

template <class T>
void put(std::vector<uint8_t>& d, const T& v) {
  const auto* p = reinterpret_cast<const uint8_t*>(&v);
  d.insert(d.end(), p, p + sizeof(T));
}

void putPose(std::vector<uint8_t>& d, const Pose& p) {
  for (float f : {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z,
                  p.orientation.w})
    put(d, f);
}

void putHand(std::vector<uint8_t>& d, const HandSample& h) {
  put(d, h.time);
  put(d, static_cast<uint8_t>(h.hand));
  put(d, static_cast<uint8_t>(h.tracked ? 1 : 0));
  put(d, h.validMask);
  for (const JointSample& j : h.joints) {
    putPose(d, j.pose);
    put(d, j.radius);
    put(d, j.confidence);
  }
}

struct Reader {
  const std::vector<uint8_t>& d;
  size_t& pos;
  bool ok = true;
  template <class T>
  T get() {
    T v{};
    if (pos + sizeof(T) > d.size()) {
      ok = false;
      return v;
    }
    std::memcpy(&v, d.data() + pos, sizeof(T));
    pos += sizeof(T);
    return v;
  }
  Pose pose() {
    Pose p;
    p.position = {get<float>(), get<float>(), get<float>()};
    p.orientation.x = get<float>();
    p.orientation.y = get<float>();
    p.orientation.z = get<float>();
    p.orientation.w = get<float>();
    return p;
  }
  void hand(HandSample& h) {
    h.time = get<double>();
    h.hand = get<uint8_t>() ? Hand::Right : Hand::Left;
    h.tracked = get<uint8_t>() != 0;
    h.validMask = get<uint32_t>();
    for (JointSample& j : h.joints) {
      j.pose = pose();
      j.radius = get<float>();
      j.confidence = get<float>();
    }
  }
};

}  // namespace

Recorder::Recorder() { clear(); }

void Recorder::clear() {
  data_.assign(kMagic, kMagic + 4);
  put(data_, static_cast<uint32_t>(kJointCount));
  frames_ = 0;
}

void Recorder::record(const HandSample* left, const HandSample* right, const HeadSample* head, uint8_t occupiedMask) {
  uint8_t flags = 0;
  if (left) flags |= kHasLeft;
  if (right) flags |= kHasRight;
  if (head) flags |= kHasHead;
  if (head && head->gaze) flags |= kHasGaze;
  put(data_, flags);
  put(data_, occupiedMask);
  if (left) putHand(data_, *left);
  if (right) putHand(data_, *right);
  if (head) {
    put(data_, head->time);
    putPose(data_, head->pose);
    if (head->gaze) {
      for (float f : {head->gaze->origin.x, head->gaze->origin.y, head->gaze->origin.z, head->gaze->direction.x,
                      head->gaze->direction.y, head->gaze->direction.z})
        put(data_, f);
    }
  }
  ++frames_;
}

bool Recorder::save(const std::string& path) const {
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  const bool ok = std::fwrite(data_.data(), 1, data_.size(), f) == data_.size();
  return std::fclose(f) == 0 && ok;
}

bool Replayer::load(const std::string& path) {
  ok_ = false;
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  std::vector<uint8_t> buf;
  uint8_t chunk[65536];
  size_t n;
  while ((n = std::fread(chunk, 1, sizeof(chunk), f)) > 0) buf.insert(buf.end(), chunk, chunk + n);
  std::fclose(f);
  return load(buf.data(), buf.size());
}

bool Replayer::load(const uint8_t* data, size_t size) {
  data_.assign(data, data + size);
  pos_ = 0;
  ok_ = false;
  if (size < 8 || std::memcmp(data, kMagic, 4) != 0) return false;
  uint32_t joints;
  std::memcpy(&joints, data + 4, 4);
  if (joints != kJointCount) return false;
  pos_ = 8;
  ok_ = true;
  return true;
}

void Replayer::rewind() { pos_ = ok_ ? 8 : 0; }

bool Replayer::next(RecordedFrame& out) {
  if (!ok_ || pos_ >= data_.size()) return false;
  Reader r{data_, pos_};
  const auto flags = r.get<uint8_t>();
  out.occupiedMask = r.get<uint8_t>();
  out.hasLeft = (flags & kHasLeft) != 0;
  out.hasRight = (flags & kHasRight) != 0;
  out.hasHead = (flags & kHasHead) != 0;
  if (out.hasLeft) r.hand(out.left);
  if (out.hasRight) r.hand(out.right);
  if (out.hasHead) {
    out.head.time = r.get<double>();
    out.head.pose = r.pose();
    out.head.gaze.reset();
    if (flags & kHasGaze) {
      Ray g;
      g.origin = {r.get<float>(), r.get<float>(), r.get<float>()};
      g.direction = {r.get<float>(), r.get<float>(), r.get<float>()};
      out.head.gaze = g;
    }
  }
  if (!r.ok) ok_ = false;
  return r.ok;
}

}  // namespace gc
