# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/state_machine.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Mission state machine
# =============================================================================

from enum import Enum, auto
from ..system.logger import log


class RobotState(Enum):
    # ── ALL ROBOT STATES ──────────────────────────────────────────────
    # These 15 states capture every phase of a WRO mission, from power-on
    # through racing to final parking and shutdown.
    #
    # WHY these states exist (race lifecycle):
    #   INIT → IDLE → START_SEARCH → FORWARD → (repeat laps) → LAP_FINISHED
    #   → PARK_APPROACH → PARK_ALIGN → PARK_BACK_IN → PARK_VERIFY → SHUTDOWN
    #
    # Error states (can be entered from almost anywhere):
    #   → OBSTACLE_AVOID, REVERSE, EMERGENCY_STOP
    #
    # The mission planner decides which transitions to register and under
    # what conditions – the state machine itself is a generic engine.

    INIT = auto()             # ── Boot / initialisation ──
                              # Robot just powered on.  Hardware checks
                              # (sensors, motors, IMU) run here.
                              # Transitions → IDLE when init is complete.

    IDLE = auto()             # ── Waiting for start signal ──
                              # Robot is ready but stationary.
                              # The start_detection module monitors the
                              # camera for the green-LED / referee signal.
                              # Transitions → START_SEARCH on signal.

    START_SEARCH = auto()     # ── Looking for start line ──
                              # After the start signal, the robot drives
                              # forward slowly, looking for the first
                              # checkpoint / start-finish line on the track.
                              # Transitions → FORWARD once found.

    FORWARD = auto()          # ── Normal forward driving ──
                              # Main racing state: straights and gentle
                              # curves.  The path tracker follows waypoints
                              # from the CheckpointManager.
                              # Transitions → CORNERING when turn angle
                              # exceeds a threshold; → OBSTACLE_AVOID when
                              # an obstacle is detected ahead;
                              # → LAP_FINISHED when all laps done.

    CORNERING = auto()        # ── Actively cornering ──
                              # Sharp turn at a track corner.  Speed is
                              # reduced and steering angle is increased.
                              # Separating CORNERING from FORWARD lets the
                              # controller apply different PID gains or
                              # feed-forward.
                              # Transitions → FORWARD when heading
                              # stabilises after the apex.

    OBSTACLE_AVOID = auto()   # ── Avoiding an obstacle ──
                              # An obstacle (cone, block, other robot) is
                              # detected in the path.  The robot steers
                              # around it using the ObstacleStrategy's
                              # decision (left / right).  This state overrides
                              # the normal path tracker.
                              # Transitions → FORWARD once the obstacle is
                              # cleared; → REVERSE if boxed in.

    REVERSE = auto()          # ── Backing up ──
                              # Robot is trapped (front + at least one side
                              # blocked) and must reverse to free itself.
                              # The ReverseLogic module manages the timed
                              # backup, after which the robot tries a
                              # different steering direction.
                              # Transitions → OBSTACLE_AVOID after minimum
                              # reverse time elapses.

    LAP_FINISHED = auto()     # ── All laps complete ──
                              # The robot has crossed the finish line the
                              # required number of times.  This is a brief
                              # intermediate state that signals the mission
                              # planner to switch from "racing" to "parking"
                              # mode.
                              # Transitions → PARK_APPROACH.

    PARK = auto()             # ── Transitional: entry to parking ──
                              # (Reserved for future use.)  A broad entry
                              # point before the sub-states below.  May
                              # be removed if PARK_APPROACH suffices.

    PARK_APPROACH = auto()    # ── Moving to the parking zone ──
                              # Robot navigates from the finish line to
                              # the vicinity of the parking bay.
                              # Uses the CheckpointManager to follow
                              # a parking-approach waypoint chain.
                              # Transitions → PARK_ALIGN when within range.

    PARK_ALIGN = auto()       # ── Aligning beside the bay ──
                              # The robot aligns itself parallel to the
                              # outer wall, at a distance of ~2 cm.
                              # Uses wall-following or visual markers
                              # (magenta tape) to establish the correct
                              # offset before reversing in.
                              # Transitions → PARK_BACK_IN once aligned.

    PARK_BACK_IN = auto()     # ── Reversing into the parking spot ──
                              # The robot reverses straight into the
                              # parking bay, guided by the magenta marker
                              # posts.  This is a low-speed, precision
                              # manoeuvre.
                              # Transitions → PARK_VERIFY once fully
                              # inside the bay (position within tolerance).

    PARK_VERIFY = auto()      # ── Stationary: judges verify ──
                              # Robot has stopped inside the parking bay.
                              # It must remain stationary for ≥30 seconds
                              # so judges can verify the position.  The
                              # elapsed_s property is checked externally
                              # to enforce the minimum wait.
                              # Transitions → SHUTDOWN after ≥30 s.

    EMERGENCY_STOP = auto()   # ── Fault / collision risk ──
                              # Immediate hard stop.  Triggered by:
                              #   - Bumper / collision sensor.
                              #   - IMU detects sudden deceleration.
                              #   - Watchdog timeout in mission planner.
                              # Robot does NOT recover from this state
                              # automatically; it requires a manual reset.

    SHUTDOWN = auto()         # ── Safe shutdown ──
                              # Final state: motors powered off, log files
                              # flushed, camera released.  The robot stays
                              # in this state until physically powered off.


class StateMachine:
    # ── GENERIC FINITE-STATE MACHINE ENGINE ──────────────────────────
    #
    # StateMachine is the robot's behavioural core.  It manages transitions
    # between RobotState values, fires entry / exit actions, and tracks how
    # long the robot has been in the current state.
    #
    # WHY a generic engine instead of hard-coded if/else?
    #   - The mission planner (or any other module) can register/unregister
    #     transitions dynamically.  This makes the robot's behaviour
    #     configurable at runtime: e.g.  "on lap 1 use cautious transitions;
    #     on lap 3 register racing transitions."
    #   - Entry/exit actions encapsulate side-effects  (e.g. "start camera
    #     recording" on entry, "stop motors" on exit).
    #   - The elapsed_s property lets timers drive behaviour without
    #     external counters.
    #
    # DESIGN PATTERN: "Transition-Table" pattern.
    #   Transitions are stored as (from_state, to_state) → condition.
    #   update() iterates only transitions whose from_state matches the
    #   current state.  The first true condition wins (first-match).
    #   If no condition is true, the robot stays in the current state.

    def __init__(self):
        # ── Internal state ──
        self.state = RobotState.INIT          # Current state the robot is in.
        self._prev_state = None                # Previous state — useful for
                                               # rollback or "return to where
                                               # you were" after an interrupt
                                               # (e.g. obstacle avoidance).
        self._transitions = {}                 # Transition table:
                                               #   key   = (from_state, to_state)
                                               #   value = callable → bool
                                               # WHY dict?  O(1) lookup for
                                               # add/remove; update() checks
                                               # all entries with matching src.
        self._entry_actions = {}               # Callbacks fired once on state entry.
                                               # Used for one-shot setup:
                                               #   e.g. "switch on backup LEDs"
                                               #        "reset obstacle counter"
        self._exit_actions = {}                # Callbacks fired on state exit.
                                               # Used for cleanup:
                                               #   e.g. "disable obstacle
                                               #         detection temporarily"
        self._state_time = 0.0                 # Seconds spent in the current
                                               # state.  Reset on each transition.
                                               # Checked by timer-driven transitions
                                               # (e.g. PARK_VERIFY ≥30 s).
        self._state_data = {}                  # Per-state key-value store.
                                               # Holds transient data like
                                               # "which checkpoint index was
                                               # the robot heading to?"

    # ── Transition registration ──────────────────────────────────────
    def add_transition(self, from_state, to_state, condition):
        # Register a transition rule.
        # When current state == from_state AND condition() == True,
        # the state machine will switch to to_state on the next update().
        #
        # WHY explicit condition callbacks?
        #   Keeps the SM logic testable: you can register mock conditions
        #   in unit tests.  Also lets multiple modules contribute rules
        #   without modifying this file.
        #
        # Transition lifecycle:
        #   The mission_planner registers all transitions once during setup(),
        #   then may deregister/re-register on lap changes.
        key = (from_state, to_state)
        self._transitions[key] = condition

    def set_entry(self, state, action):
        # Register a one-shot callback that runs when `state` is entered.
        # The callback receives no arguments — use closures or partials
        # to pass context.
        #
        # Example:
        #   sm.set_entry(RobotState.REVERSE, lambda: motor.set_speed(-0.3))
        self._entry_actions[state] = action

    def set_exit(self, state, action):
        # Register a callback that runs when `state` is exited.
        # Useful for cleanup: e.g. resetting integrators, stopping sounds.
        self._exit_actions[state] = action

    # ── Per-state data store ─────────────────────────────────────────
    def set_state_data(self, key, value):
        # Store an arbitrary value keyed to the current state.
        # WHY not just store in the mission planner?
        #   Some state-related data is transient (only meaningful while
        #   in that state).  This store is implicitly "namespaced" by
        #   the state — the data is discarded on transition (the caller
        #   must re-set it in the new state's entry action).
        self._state_data[key] = value

    def get_state_data(self, key, default=None):
        # Retrieve a value previously stored via set_state_data.
        return self._state_data.get(key, default)

    # ── Core tick (called every control loop iteration) ──────────────
    def update(self, dt):
        # Called every iteration by the mission planner with the timestep
        # dt (seconds, typically 0.01–0.05).
        #
        # WHAT IT DOES:
        #   1. Increment the state timer (elapsed_s).
        #   2. Scan every registered transition where src == current state.
        #   3. If a condition is True, transition and stop scanning
        #      (first-match priority).
        #
        # WHY first-match?  Conditions are evaluated in dict insertion
        # order (Python 3.7+).  The mission planner should register more
        # urgent transitions first — e.g. EMERGENCY_STOP before FORWARD.
        #
        # RACE CONDITION NOTE:
        #   If two conditions become true in the same tick, only the first
        #   registered transition fires.  The other condition will be
        #   re-evaluated next tick (after entry action runs).  This is
        #   acceptable because entry actions should be idempotent or
        #   fast.
        self._state_time += dt
        for (src, dst), cond in self._transitions.items():
            if src == self.state and cond():
                self.transition_to(dst)
                break

    # ── State transition (internal) ──────────────────────────────────
    def transition_to(self, new_state):
        # Execute a state switch with full lifecycle:
        #
        #   [EXIT current] → [store prev] → [set new] → [reset timer]
        #   → [log] → [ENTRY new]
        #
        # WHY this order?  The exit action may depend on knowing the
        # current state; the entry action may depend on timer=0 and
        # the previous state being set correctly.
        #
        # GUARD: If already in new_state, do nothing (prevents
        # re-entering the same state, which would fire entry again).
        if new_state == self.state:
            return
        # 1. Fire exit action for the current state (if any).
        if self.state in self._exit_actions:
            self._exit_actions[self.state]()
        # 2. Remember where we came from (useful for "back" transitions).
        self._prev_state = self.state
        # 3. Set the new state.
        self.state = new_state
        # 4. Reset elapsed timer — the "clock" for the new state starts now.
        self._state_time = 0.0
        # 5. Log every transition (critical for post-race debugging).
        log.info(f"State: {self._prev_state.name} -> {new_state.name}")
        # 6. Fire entry action for the new state (if any).
        if new_state in self._entry_actions:
            self._entry_actions[new_state]()

    # ── Queries ──────────────────────────────────────────────────────
    @property
    def elapsed_s(self):
        # Time spent in the current state.
        # Used by:
        #   - PARK_VERIFY: must stay ≥30 s.
        #   - REVERSE:     reverse for at least min_time.
        #   - IDLE:        timeout if no start signal received.
        return self._state_time

    def is_in(self, *states):
        # Returns True if the current state matches any of the given states.
        # Shorthand so the mission planner can write:
        #   if sm.is_in(RobotState.FORWARD, RobotState.CORNERING):
        #       ...apply path tracking...
        # instead of chaining == checks.
        return self.state in states
