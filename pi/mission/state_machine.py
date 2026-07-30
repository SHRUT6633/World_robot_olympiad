from enum import Enum, auto
from ..system.logger import log


class RobotState(Enum):
    # All possible states the robot can be in during a WRO mission lap.
    # The state machine transitions between these based on sensor inputs,
    # lap progress, and strategy decisions.
    INIT = auto()             # Startup / hardware initialisation.
    IDLE = auto()             # Waiting for the start signal.
    START_SEARCH = auto()     # Looking for the start line / first checkpoint.
    FORWARD = auto()          # Normal forward driving (straights and gentle curves).
    CORNERING = auto()        # Actively turning around a corner.
    OBSTACLE_AVOID = auto()   # Manoeuvring around a detected obstacle.
    REVERSE = auto()          # Backing up (e.g. when stuck or boxed in).
    LAP_FINISHED = auto()     # All laps completed; prepare to park.
    PARK = auto()             # Final parking manoeuvre (entry).
    PARK_APPROACH = auto()    # Moving towards the detected parking zone.
    PARK_ALIGN = auto()       # Aligning parallel to the outer wall (2 cm tolerance).
    PARK_BACK_IN = auto()     # Reversing into the parking spot between magenta markers.
    PARK_VERIFY = auto()      # Stopped — judges verify position (≥30 s stationary).
    EMERGENCY_STOP = auto()   # Immediate stop due to fault or collision risk.
    SHUTDOWN = auto()         # Safe shutdown sequence.


class StateMachine:
    # StateMachine is the robot's behavioural core.  It manages transitions
    # between RobotState values, fires entry / exit actions, and tracks how
    # long the robot has been in the current state.  Other modules register
    # transitions (from_state, to_state, condition) so that the update()
    # call, invoked each tick, evaluates every outgoing transition from the
    # current state and switches when a condition becomes true.

    def __init__(self):
        self.state = RobotState.INIT          # Current state.
        self._prev_state = None                # Previous state (for logging / rollback).
        self._transitions = {}                 # key=(from, to) -> callable condition().
        self._entry_actions = {}               # state -> callable run on entry.
        self._exit_actions = {}                # state -> callable run on exit.
        self._state_time = 0.0                 # Seconds spent in the current state.
        self._state_data = {}                  # Arbitrary key-value store for state-related data.

    def add_transition(self, from_state, to_state, condition):
        # Register a transition: when the robot is in from_state and condition()
        # returns True, the machine will move to to_state at the next update.
        key = (from_state, to_state)
        self._transitions[key] = condition

    def set_entry(self, state, action):
        # Register a callback that runs once when the state is entered.
        self._entry_actions[state] = action

    def set_exit(self, state, action):
        # Register a callback that runs once when the state is exited.
        self._exit_actions[state] = action

    def set_state_data(self, key, value):
        # Store arbitrary data associated with the current state
        # (e.g. a target waypoint index, a timer override).
        self._state_data[key] = value

    def get_state_data(self, key, default=None):
        # Retrieve a value previously stored via set_state_data.
        return self._state_data.get(key, default)

    def update(self, dt):
        # Called every iteration with the timestep dt (seconds).
        # Increments the state timer and checks all registered transitions
        # that originate from the current state.  The first true transition
        # triggers a switch.
        self._state_time += dt
        for (src, dst), cond in self._transitions.items():
            if src == self.state and cond():
                self.transition_to(dst)
                break

    def transition_to(self, new_state):
        # Perform the actual state switch:
        #   1. Fire the exit action of the current state.
        #   2. Update the previous-state pointer.
        #   3. Set the new state.
        #   4. Reset the state timer.
        #   5. Log the transition.
        #   6. Fire the entry action of the new state.
        if new_state == self.state:
            return
        if self.state in self._exit_actions:
            self._exit_actions[self.state]()
        self._prev_state = self.state
        self.state = new_state
        self._state_time = 0.0
        log.info(f"State: {self._prev_state.name} -> {new_state.name}")
        if new_state in self._entry_actions:
            self._entry_actions[new_state]()

    @property
    def elapsed_s(self):
        # Returns how many seconds the robot has been in the current state.
        return self._state_time

    def is_in(self, *states):
        # Convenience check: returns True if the current state matches any
        # of the given states.  Usage:  sm.is_in(RobotState.FORWARD, RobotState.CORNERING)
        return self.state in states
