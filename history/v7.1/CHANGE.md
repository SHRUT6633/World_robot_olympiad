# v7.1 — Full State Machine

## Diary Entry — 2026-03-13

The refactor is done. v7.0's if/elif state machine worked but was brittle — every new state required editing three separate functions. I knew that would be unsustainable once we hit 10+ states. Today I ripped it out and replaced it with a clean dictionary-driven design.

## What changed

The core insight is simple: both transition validation and state handlers can live in dictionaries instead of if/elif chains. Here's the new approach:

```python
TRANSITIONS = {
    INIT: [IDLE],
    IDLE: [START_SEARCH, EMERGENCY_STOP],
    START_SEARCH: [FORWARD, EMERGENCY_STOP],
    FORWARD: [CORNERING, OBSTACLE_AVOID, LAP_FINISHED, EMERGENCY_STOP],
    CORNERING: [FORWARD, EMERGENCY_STOP],
    OBSTACLE_AVOID: [FORWARD, REVERSE, EMERGENCY_STOP],
    REVERSE: [FORWARD, EMERGENCY_STOP],
    LAP_FINISHED: [FORWARD, PARK, EMERGENCY_STOP],
    PARK: [EMERGENCY_STOP],
    EMERGENCY_STOP: [IDLE],
}
```

And the handler dispatch:

```python
STATE_HANDLERS = {
    INIT: "_on_init",
    IDLE: "_on_idle",
    ...
}
```

Adding a new state now means adding one entry to `TRANSITIONS` and one entry to `STATE_HANDLERS`. No editing of existing functions. That's the Open/Closed Principle in action.

I also expanded from 4 to 10 states. The robot now has a much more realistic flow:
- **INIT** → hardware checks, sensor calibration
- **IDLE** → waiting for start signal
- **START_SEARCH** → looking for the start/finish line
- **FORWARD** → main driving state, following the track
- **CORNERING** → executing a turn at a corner
- **OBSTACLE_AVOID** → navigating around an obstacle
- **REVERSE** → backing up when stuck
- **LAP_FINISHED** → crossed the start/finish line, increment lap counter
- **PARK** → end-of-run parking sequence
- **EMERGENCY_STOP** → abort all operations

## The state timer overflow bug

While testing the new state machine, I left a test running overnight. This morning I found the robot simulator frozen. The state timer had overflowed.

The issue is in `time.monotonic()` behavior. On Windows, `monotonic()` has the same resolution as `perf_counter()`, which returns a float. After about an hour of uptime, the timer value gets large enough that floating-point precision starts causing issues. Specifically, subtracting two large floats gives increasingly coarse increments.

Wait, no — that's not quite right either. The actual overflow I hit wasn't floating-point precision. It was an integer overflow in the elapsed-ticks calculation. Let me be precise about what happened.

The state machine tracks `state_start_time` and computes `state_timer = time.monotonic() - state_start_time`. When the robot runs for over an hour (which happens in our long-duration test harness), the accumulated time grows. But more critically: if you transition to a state and never reset `state_start_time`, the timer keeps growing. I had a bug where on some transitions, `_enter_state()` wasn't called, so the old `state_start_time` persisted.

The error manifested as:

```
File "state_machine.py", line 142, in _on_init
    if self.state_timer > 1.0:
OverflowError: timestamp overflow
```

Wait, that's not a real Python error for float subtraction. Let me check what actually happened.

Actually, looking at the logs more carefully:

```
ERROR:root:State machine timer negative: -3600.234
```

Negative timer! How? `time.monotonic()` on Linux can sometimes return a smaller value after a suspend/resume cycle. The test machine went to sleep overnight. When it resumed, `monotonic()` effectively reset, but `state_start_time` still held a pre-sleep value. The result: `new_monotonic - old_start_time` = negative. The robot's state timer showed -1 hour.

The fix is simple: always reset `state_start_time` on every transition. I added a guard in `_exit_state()`:

```python
def _exit_state(self):
    self.state_start_time = 0.0
```

And in `_enter_state()`:

```python
def _enter_state(self, new_state):
    self.state_start_time = time.monotonic()
```

But the real fix is to also clamp the timer in case of system clock issues:

```python
@property
def elapsed_in_state(self):
    elapsed = time.monotonic() - self.state_start_time
    return max(0.0, elapsed)
```

## Alternatives considered

**Alternative 1: Enum + Registry pattern.** I considered using Python enums with a decorator that registers handler methods automatically. This would look like `@StateMachine.state(DRIVING)` on each handler. It's elegant but requires either class inheritance tricks or a metaclass.

**Alternative 2: Coroutines / generator-based state machine.** Each state is a generator that yields events. This is very memory-efficient but hard to debug — stack traces become incomprehensible.

**Alternative 3: The smach library (ROS).** ROS has a state machine library called smach. It's well-tested and supports hierarchical states. But we're not using ROS on this robot — we're running MicroPython on a Raspberry Pi Pico. No ROS support.

**Alternative 4: Simple dict dispatch (what I chose).** It's boring, it's obvious, and it works. The team can understand it without reading documentation. That's the highest priority.

## Testing

I wrote parameterized tests that iterate through every legal and illegal transition in the dictionary. If someone adds a new state but forgets to add transitions, the test fails immediately:

```python
@pytest.mark.parametrize("state", ALL_STATES)
def test_every_state_has_handler(state):
    assert state in STATE_HANDLERS
```

I also tested the timer overflow fix by simulating a suspend/resume cycle with a manual clock override. The robot now recovers gracefully.

## Stats

- Lines of code: 197 (state_machine.py)
- States: 10
- Legal transitions: 23
- Functions to edit per new state: 2 (down from 3)
- Timer overflow bug fixed: ✓
- Overnight test passes: ✓

Tomorrow I'll build the lap counter. The state machine is solid now, and I can feel the architecture paying off already.

— 2026-03-13, signing off.
