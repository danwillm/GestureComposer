#pragma once
// Record / replay of input streams for deterministic regression tests and threshold tuning.
// Format: "GCR1" magic + little-endian frames (see recorder.cpp). Not intended as an interchange format.

#include <cstdint>
#include <string>
#include <vector>

#include "types.h"

namespace gc {

class Composer;

struct RecordedFrame {
  bool hasLeft = false, hasRight = false, hasHead = false;
  HandSample left, right;
  HeadSample head;
  uint8_t occupiedMask = 0;  // bit 0 = left, bit 1 = right
};

class Recorder {
 public:
  Recorder();
  void record(const HandSample* left, const HandSample* right, const HeadSample* head, uint8_t occupiedMask = 0);
  bool save(const std::string& path) const;
  const std::vector<uint8_t>& data() const { return data_; }
  size_t frameCount() const { return frames_; }
  void clear();

 private:
  std::vector<uint8_t> data_;
  size_t frames_ = 0;
};

class Replayer {
 public:
  bool load(const std::string& path);
  bool load(const uint8_t* data, size_t size);
  bool next(RecordedFrame& out);  // false at end or on corrupt data
  void rewind();
  bool ok() const { return ok_; }

  // Feeds the remaining frames (occupancy flags included) to the composer; calls onFrame after each update.
  template <class OnFrame>
  void play(Composer& composer, OnFrame&& onFrame);
  void play(Composer& composer);

 private:
  std::vector<uint8_t> data_;
  size_t pos_ = 0;
  bool ok_ = false;
};

}  // namespace gc

#include "composer.h"

namespace gc {
template <class OnFrame>
void Replayer::play(Composer& composer, OnFrame&& onFrame) {
  RecordedFrame f;
  while (next(f)) {
    composer.setHandOccupied(Hand::Left, (f.occupiedMask & 1u) != 0);
    composer.setHandOccupied(Hand::Right, (f.occupiedMask & 2u) != 0);
    composer.update(f.hasLeft ? &f.left : nullptr, f.hasRight ? &f.right : nullptr, f.hasHead ? &f.head : nullptr);
    onFrame(f);
  }
}
inline void Replayer::play(Composer& composer) {
  play(composer, [](const RecordedFrame&) {});
}
}  // namespace gc
