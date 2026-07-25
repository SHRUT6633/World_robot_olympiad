from enum import Enum, auto
from ..system.logger import log


class RobotState(Enum):
    INIT = auto()
    IDLE = auto()
    START_SEARCH = auto()
    FORWARD = auto()
    CORNERING = auto()
    OBSTACLE_AVOID = auto()
    REVERSE = auto()
    LAP_FINISHED = auto()
    PARK = auto()
    EMERGENCY_STOP = auto()
    SHUTDOWN = auto()


class StateMachine:
    def __init__(self):
        self.state = RobotState.INIT
        self._prev_state = None
        self._transitions = {}
        self._entry_actions = {}
        self._exit_actions = {}
        self._state_time = 0.0
        self._state_data = {}

    def add_transition(self, from_state, to_state, condition):
        key = (from_state, to_state)
        self._transitions[key] = condition

    def set_entry(self, state, action):
        self._entry_actions[state] = action

    def set_exit(self, state, action):
        self._exit_actions[state] = action

    def set_state_data(self, key, value):
        self._state_data[key] = value

    def get_state_data(self, key, default=None):
        return self._state_data.get(key, default)

    def update(self, dt):
        self._state_time += dt
        for (src, dst), cond in self._transitions.items():
            if src == self.state and cond():
                self.transition_to(dst)
                break

    def transition_to(self, new_state):
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
        return self._state_time

    def is_in(self, *states):
        return self.state in states
