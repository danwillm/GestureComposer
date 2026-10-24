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
Besides events, you can poll gesture state, live signal values and raw hand features every frame. See [Polling state and signals](#polling-state-and-signals).

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

## Polling state and signals

Events tell you when something *changes*. For anything you read every frame (held buttons, analog values, UI feedback), poll after `update()` instead. There are three levels:

| Call | Returns | Use it for |
|---|---|---|
| `composer.state(id, hand)` | `GestureState`: the gesture's **decision**, after hysteresis, onset, gates, arming and arbitration | game logic: "is the user grabbing?", "is the menu armed?" |
| `composer.signal(sig, hand)` | `SignalValue {value, confidence}`: the **raw score** 0..1 of one ingredient, recomputed every frame with no decisions applied | analog feedback: pinch rings, trigger axes, debug readouts |
| `composer.features(hand)` | `HandFeatures`: filtered **measurements** (palm pose, velocities, curls, pinch distances, rays) | attaching objects to the hand, custom logic |

### Held buttons: `state().active`

`active` is only true for continuous gestures (`active(...)`, or `both(active(...))`). A `press()`, `click()` or `hold()` gesture never reports `active`; read those from events. To get a held button and its "pressed" moment from one gesture, use `active()`. Its `Began` event fires at the same instant `press()` would.

```cpp
gc::Signal sIndexPinch = pinch(Finger::Index);   // keep it: needed for signal() below
gc::GestureId pinchHeld = composer.add(gesture("pinch_held")
    .trigger(active(Condition(sIndexPinch).onset(true)))   // onset: a hand appearing pinched doesn't count
    .updates(false));                                     // no per-frame Updated events: we poll

// every frame, after update():
const bool pinching = composer.state(pinchHeld, gc::Hand::Right).active;
```

### Analog values: `signal()`

```cpp
const gc::SignalValue p = composer.signal(sIndexPinch, gc::Hand::Right);
pinchRing.setFill(p.value);                      // 0 = open, 1 = touching
pinchRing.setVisible(p.confidence > 0.3f);       // hide when the fingers aren't visible to the tracker
```

`signal()` looks a signal up by the **object you built**. Keep the `gc::Signal` you passed to the gesture; a newly created `pinch(...)` won't be found and returns `{0, 0}`. A signal that no gesture uses must be registered with `composer.track(sig)` at setup. Calling it later triggers a recompile, which resets runtime state. Composite signals work too:

```cpp
gc::Signal palmToHead = palmFacing(Target::Head) && inView(40.f) && still();
composer.track(palmToHead);                      // only needed if no gesture uses this exact object
float facing = composer.signal(palmToHead, gc::Hand::Left).value;
```

### Affordances and progress: `state().armed` and `state().progress`

```cpp
// Show a "menu available" hint while the arm conditions hold, before the user pinches.
const gc::GestureState menuState = composer.state(menu, gc::Hand::Left);
menuHint.setVisible(menuState.armed);            // only meaningful for gestures with .arm(...)

// Fill a radial timer for a long-press: hold(pinch(Finger::Index), 600ms)
const gc::GestureState lp = composer.state(longPress, gc::Hand::Right);
radial.setFill(lp.progress);                     // 0..1 while held; 0 when released
```

`progress` also reports sequence and repeat completion (`seq` steps done / total, `repeat` count / n), click timing, and `active(...).minDuration()`.

### Aim rays: `state().ray`

```cpp
gc::GestureId aim = composer.add(gesture("aim")
    .trigger(active(extended(Finger::Index) && curled(Finger::Middle, Finger::Ring, Finger::Little)))
    .aim(gc::Aim::IndexRay)
    .updates(false));

const gc::GestureState a = composer.state(aim, gc::Hand::Right);
if (a.active && a.rayValid) teleportArc.update(a.ray.origin, a.ray.direction);
```

The ray is filtered and available whenever the gesture has `.aim(...)`, even while it isn't active (useful for a faint "preview" ray).

### Raw measurements: `features()`

```cpp
const gc::HandFeatures& f = composer.features(gc::Hand::Right);
if (f.tracked) {
  gc::Vec3 pinchPoint = gc::lerp(f.position(gc::Joint::ThumbTip), f.position(gc::Joint::IndexTip), 0.5f);
  heldObject.follow(pinchPoint, f.palmNormal, f.palmForward);   // filtered palm frame
  throwVelocity = f.palmVelocity;                               // m/s, filtered
}
float middleCurl = f.curl[static_cast<int>(gc::Finger::Middle)];  // 0 straight .. 1 curled
float gap        = f.pinchDistance[static_cast<int>(gc::Finger::Index)];  // surface distance, scaled metres
```

Only features that a registered (enabled) gesture or tracked signal needs are computed each frame. For example, `curl` stays at its last value if nothing uses a curl-based signal. Use `composer.featureMask()` to check what is being computed, or `track()` a signal that needs the feature.

### What each `GestureState` field means

| Field | Meaningful for |
|---|---|
| `active` | continuous gestures (`active`, `both(active)`, `chord` of actives) |
| `armed` | gestures with `.arm(...)` |
| `progress` | `hold`, `seq`, `repeat`, `click`, `active(...).minDuration()` |
| `strength` | single-step triggers (`press`, `click`, `active`, `hold`): current value of the trigger's signal; for `seq`/`repeat` it is the value at the last completed step |
| `ray`, `rayValid` | gestures with `.aim(...)` |
| `since` | all: time of the last state change |

All of these reflect the most recent `update()`. With hands fed in separate calls, `signal()` and `features()` for a hand change only on calls that carry new data for it. `state()` is re-evaluated on every call, so time-based fields such as hold `progress` keep advancing.

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
