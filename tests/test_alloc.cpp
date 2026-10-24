// Verifies Composer::update() performs no heap allocation once compiled.
// Separate executable because it replaces the global allocator.

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <new>

#include "sim.h"

static std::atomic<long> g_allocs{0};
static std::atomic<bool> g_counting{false};

void* operator new(std::size_t n) {
  if (g_counting) ++g_allocs;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void* operator new[](std::size_t n) {
  if (g_counting) ++g_allocs;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

int main() {
  using namespace gc;
  using namespace gc::gestures;
  using namespace std::chrono_literals;
  using gc::testing::HandPose;

  Composer c;
  auto ip = pinch(Finger::Index);
  auto palmToHead = palmFacing(Target::Head) && inView() && still();
  c.add(gesture("select").trigger(press(ip)).gate(!grasping()));
  c.add(gesture("menu").trigger(press(ip)).arm(palmToHead, 250ms));
  c.add(gesture("drag").trigger(active(pinch(Finger::Middle))).aim(Aim::ShoulderRay));
  c.add(gesture("dbl").trigger(repeat(click(ip), 2).within(400ms)).ambiguity(Resolve::Wait));
  c.add(gesture("combo").trigger(seq(click(pinch(Finger::Ring)), click(pinch(Finger::Little)))));
  c.add(gesture("scale").trigger(both(active(ip))));
  c.add(gesture("custom").trigger(press(custom("pistol", [](const HandFeatures& f) { return f.curl[2]; }))));
  int events = 0;
  c.add(gesture("cb").trigger(press(pinch(Finger::Ring))).onEvent([&](const Event&) { ++events; }));
  c.compile();

  gctest::Sim sim(c);
  sim.noise = 0.001f;
  // Pre-generate samples so the measurement only covers update().
  std::vector<HandSample> left, right;
  for (int i = 0; i < 900; ++i) {
    const float a = 0.5f + 0.5f * std::sin(i * 0.07f);
    const Finger f = static_cast<Finger>(1 + (i / 120) % 4);
    sim.hand[0].pose = gctest::mix(HandPose::relaxed(), HandPose::pinch(f), a);
    sim.hand[1].pose = gctest::mix(HandPose::relaxed(), HandPose::pinch(f), 1.f - a);
    sim.t += sim.dt;
    left.push_back(sim.sample(0));
    right.push_back(sim.sample(1));
  }
  HeadSample head = sim.head;
  g_counting = true;
  for (size_t i = 0; i < left.size(); ++i) {
    head.time = left[i].time;
    c.update(&left[i], (i / 200) % 2 ? nullptr : &right[i], &head);
  }
  g_counting = false;
  std::printf("allocations during %zu updates: %ld (callback events: %d)\n", left.size(), g_allocs.load(), events);
  return g_allocs == 0 ? 0 : 1;
}
