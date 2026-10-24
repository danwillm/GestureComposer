# GestureComposer

Gesture detection over a world-space hand skeleton, for XR headsets. C++17, no dependencies, CMake `add_subdirectory`.
Works with any skeleton source; adapters for OpenXR (`XR_EXT_hand_tracking`) and visionOS (ARKit `HandSkeleton`) are included.
Design rationale and references: [docs/PROPOSAL.md](docs/PROPOSAL.md).

```cpp
#include <gesture_composer/gesture_composer.h>
using namespace gc::gestures;
using namespace std::chrono_literals;

gc::Composer composer;

auto indexPinch = pinch(Finger::Index);
auto relaxed    = !grasping() && !occupied();
auto palmToHead = palmFacing(Target::Head) && inView() && still()
                  && extended(Finger::Middle, Finger::Ring, Finger::Little);

auto select = composer.add(gesture("select").trigger(press(indexPinch)).gate(relaxed));
auto menu   = composer.add(gesture("menu").hands(Hands::Left)        // left hand only
                               .trigger(press(indexPinch)).arm(palmToHead, 250ms).gate(relaxed));
auto undo   = composer.add(gesture("undo").trigger(repeat(click(indexPinch), 2).within(350ms)));

// every frame
composer.update(&leftSample, &rightSample, &headSample);   // nullptr for an unavailable hand/head
for (const gc::Event& e : composer.events())
  if (e.id == menu && e.phase == gc::Phase::Triggered) ui.toggleMenu();
```

`menu` and `select` share one physical pinch. On the left hand, `menu` wins automatically when armed (it is more specific), and only one of them fires. On the right hand, the same motion is a plain `select`.

### Choosing hands

`.hands()` picks where a gesture is evaluated:

| | |
|---|---|
| `Hands::Either` (default) | each hand independently; `Event::hand` says which |
| `Hands::Left` / `Hands::Right` | that hand only; the other hand's motion is never seen by this gesture |
| `Hands::Both` | two-handed: the trigger must happen on both hands together (same as `both(trigger)`) |

For different gestures per hand, register one gesture per hand, for example `menu` on `Hands::Left` and `teleport` on `Hands::Right`. When each hand must do something *different* at the same time, use `chord(leftPattern, rightPattern)`.

## Building

```cmake
add_subdirectory(external/GestureComposer)
target_link_libraries(my_app PRIVATE gesture_composer::gesture_composer
                                     gesture_composer::openxr)   # or gesture_composer::visionos
```

| Option | Default | |
|---|---|---|
| `GC_BUILD_TESTS` / `GC_BUILD_EXAMPLES` / `GC_BUILD_BENCHMARKS` | ON only when top-level | |
| `GC_BUILD_OPENXR_ADAPTER` | ON | header-only; you provide `<openxr/openxr.h>` (links `OpenXR::headers` if that target exists) |
| `GC_BUILD_VISIONOS_ADAPTER` | ON | header-only, no Apple dependency |
| `GC_NO_EXCEPTIONS` | OFF | builds the library with exceptions and RTTI disabled |

Standalone: `cmake -S . -B build && cmake --build build --config Release && ctest --test-dir build -C Release`.

## Concepts

| Layer | What it is | Examples |
|---|---|---|
| **Signal** | Per-hand continuous score in [0,1] with a confidence. Combine with `&&` (min), `\|\|` (max), `!` (1−x). | `pinch(f)`, `curl(f)`, `extended(f, ...)`, `curled(f, ...)`, `spread(f)`, `palmFacing(Target::Head)`, `inView()`, `still()`, `grasping()`, `occupied()`, `tracked()`, `custom(name, fn, features)` |
| **Condition** | A signal debounced into a boolean. Signals convert implicitly. | `pinch(f).above(0.8f, 0.5f).dwell(80ms)`, `Condition(s).onset(false).predict(25ms)` |
| **Pattern** | Temporal expression over conditions | `press`, `release`, `click(c, maxDuration, maxTravel)`, `active(c).minDuration(t)`, `hold(c, t)`, `seq(a, b, ...).within(t).maxGap(t)`, `repeat(p, n).within(t)`, `both(p).sync(t)`, `chord(left, right)` |
| **Gesture** | A pattern plus context and arbitration | `.trigger()`, `.arm(cond, minHold)`, `.gate(cond)`, `.cancelIf(cond)`, `.hands()`, `.priority()`, `.exclusive(group)`, `.shared()`, `.aim(Aim, Stabilize)`, `.ambiguity(Resolve::Wait, window)`, `.onEvent(cb)` |

Events: `Armed`/`Disarmed` (show an affordance), `Began`/`Updated`/`Ended`/`Cancelled` (continuous patterns), `Triggered` (discrete).
Continuous state, including hold progress, armed state and aim rays, is available from `composer.state(id, hand)`. Signal values come from `composer.signal(sig, hand)`, and raw features from `composer.features(hand)`.

### What each condition does for you

- **Hysteresis.** Separate `enter` and `exit` thresholds; defaults are 0.7 and 0.4.
- **Velocity-adaptive dwell.** A decisive press or release commits in the same frame. A slow drift must persist for `maxDwell` (60 ms) or `releaseDwell` (40 ms).
- **Onset requirement.** On by default for press, release, click and hold. The signal must be seen *open* shortly before closing, so a hand that is already pinched or gripping never fires.
- **Refractory period.** 50 ms. A re-close during refractory still fires once it ends, so fast double pinches are not lost.
- **Tracking loss.** Low confidence freezes the condition. Active conditions are `Cancelled` (never `Ended`) after the 120 ms grace period.
- **Predictive onset.** Optional: `.predict(25ms)`.

### Arbitration

- **One action, one gesture.** Every activation of a condition is a token, and only one non-shared gesture may consume it. Ranking is `priority`, then *specificity*, then registration order. Specificity is the number of arm and gate conditions plus the number of steps in the pattern, so `repeat(click, 2)` beats `click` and palm+pinch beats pinch.
- **Arm before trigger.** Arm conditions must have held for `minHold` *before* the trigger's onset.
- **Exclusive groups.** At most one member per hand at a time. Sliding from one finger to the next needs a release first.
- **`Resolve::Wait`.** Delays a gesture until no longer pattern has consumed its action (tap vs double-tap). `Eager` is the default.

### Aim

`Aim::IndexRay` and `Aim::ShoulderRay` (falls back to the index ray without a head pose) are One-Euro filtered. With `Stabilize::RewindToOnset`, a discrete gesture reports the ray from the last moment it was stable before the confirming motion. This is detected from the ray's own angular speed and bounded by `Config::rewindMax`, which counters the Heisenberg effect.

## Feeding input

`nullptr` for a hand or the head means **"no new data in this call"**, not "untracked". The last state is held, so hands can arrive in separate calls:

```cpp
composer.update(t, &left, nullptr, &head);   // left hand's callback
composer.update(t, nullptr, &right);         // right hand's callback, same or slightly different t
```

- **When a hand counts as untracked:** when its sample has `tracked == false`, which the adapters set from the platform, or when no new sample has arrived for `Config::inputTimeout` (100 ms). After that, active gestures are cancelled once the 120 ms grace period runs out.
- **Repeated samples are ignored.** A sample whose `HandSample::time` was already processed is skipped, so hand tracking that runs slower than your update loop (e.g. 30 Hz hands in a 90 Hz loop) is handled correctly.
- **Time never goes backwards.** A call whose time is older than the last one is clamped to the last time instead of being dropped.

**Reading events with one `update()` per hand.** By default, `events()` holds only the events from the *latest* `update()` call. A call that carries only left-hand data can still produce right-hand events, such as a hold completing or a `Resolve::Wait` deadline passing. You have three options:

1. Read `events()` after **every** `update()` call.
2. Set `Config::autoClearEvents = false`. Events then accumulate across calls until you call `clearEvents()`:
   ```cpp
   gc::Config cfg;
   cfg.autoClearEvents = false;
   gc::Composer composer(cfg);
   // ...
   composer.update(t, &left, nullptr, &head);
   composer.update(t, nullptr, &right);
   for (const gc::Event& e : composer.events()) handle(e);   // everything since the last clear
   composer.clearEvents();
   ```
   The buffer is preallocated with room for about 8 updates. Raise `Config::eventCapacity` if you clear less often. Overflow is counted in `droppedEvents()`.
3. Use `.onEvent(callback)`. Callbacks fire inside the `update()` that produced the event, exactly once, in either mode.

## Platform adapters

```cpp
// OpenXR
gc::openxr::convert(locations, gc::Hand::Right, gc::openxr::timeToSeconds(time), sample);

// visionOS: 27 column-major world matrices in gc::visionos::ARKitJoint order + per-joint isTracked
gc::visionos::convert(matrices, tracked, anchor.isTracked, gc::Hand::Left, timestamp, sample);
```

Features are derived from joint **positions**, never the platform's per-joint axis conventions, so the same hand produces the same values on every platform. visionOS has no palm joint, so it is synthesised. Joint radii default to anatomical values scaled by the measured hand size.

## Testing and tuning

- `gc::testing::makeHand()` (`<gesture_composer/testing/synthetic_hand.h>`) is a forward-kinematics hand with poses `open`, `relaxed`, `point`, `fist`, `pinch(f)` and `grip`.
- `gc::Recorder` and `gc::Replayer` record and replay input streams. Replay is deterministic, so a recording reproduces the same events.
- The tests include the failure cases that motivated the design: threshold chatter under jitter, the steering-wheel grip, a palm arriving together with a pinch, thumb-to-finger cross-talk, tracking dropouts, and fast double pinches. The steering-wheel test also checks that a naive detector *does* misfire on the same motion.
- `gesture_composer_alloc_test` asserts that `update()` makes zero heap allocations.

## Performance

`gesture_composer_bench`, with 32 gestures, both hands and a head, on a desktop x64 CPU (MSVC, Release): **median 2.4 µs, p99 2.8 µs per `update()`**.
Headset numbers have not been measured yet; expect a few times slower on mobile cores.

The cost stays low for these reasons:

- **Compile step.** Identical sub-expressions are hash-consed, so `pinch(Index)` used by five gestures is evaluated once.
- **Flat evaluation.** Signals, conditions and patterns are evaluated as flat arrays with no virtual calls.
- **Masked features.** Only features required by *enabled* gestures are computed.
- **No per-frame allocation.**

`Composer` is single-threaded: call `update()` from one thread.

## Status against the proposal

Implemented:

- canonical input types, the OpenXR and visionOS adapters, and a generic mapping path (fill `HandSample` yourself)
- conditioning: dropout hold, One Euro filtering, hand-scale normalisation
- the full feature set: capsule pinch distances, curl, spread, the position-derived palm frame, kinematics, the grasp detector with hold-off, and index and shoulder rays
- the signal, condition and pattern API (`gc::gestures`) with compile-time diagnostics
- token arbitration, specificity, exclusive groups, `Resolve::Wait`, and ray rewind
- record and replay, the synthetic hand, tests, the benchmark, and an example

Changes made during implementation:

- **Arming is evaluated at the trigger onset.** The arm condition must have been active over `[onset − minHold, onset]`. It does not need to stay active until the gesture fires, which makes `release(pose).arm(pose)` usable.
- **Release dwell is velocity-adaptive**, like the press dwell.
- **Specificity includes pattern length.**
- **Ray rewind uses the ray's own stability** rather than the trigger signal's quiet time. Signals with dead zones, such as `extended()`, hid the start of the motion.
- **Sequences track one partial match.** It restarts on a new first step, rather than the up-to-4 parallel partial matches described.

Not yet implemented:

- `Stabilize::FreezeDuringPress`
- debug probes; use `signal()`, `features()` and `state()` instead
- the `gc_tune` CLI
- per-user pinch adaptation
- learned pose templates and `$P` trajectories
- the kinematic-plausibility tie-break for thumb-to-finger
- fusing runtime pinch hints (`XR_FB_hand_tracking_aim`)

All default thresholds come from the synthetic hand. They still need tuning against recordings from real devices.

## Layout

```
include/gesture_composer/   public API (composer.h, gestures.h, features.h, types.h, math.h, recorder.h, testing/)
src/                        implementation (condition state machine, hand tracker, compiler/runtime)
adapters/openxr|visionos/   header-only adapters
tests/ examples/ bench/     tests (incl. allocation test), example, benchmark
docs/PROPOSAL.md            design proposal
```
