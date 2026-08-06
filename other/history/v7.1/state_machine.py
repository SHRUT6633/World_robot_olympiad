import time

INIT = "INIT"
IDLE = "IDLE"
START_SEARCH = "START_SEARCH"
FORWARD = "FORWARD"
CORNERING = "CORNERING"
OBSTACLE_AVOID = "OBSTACLE_AVOID"
REVERSE = "REVERSE"
LAP_FINISHED = "LAP_FINISHED"
PARK = "PARK"
EMERGENCY_STOP = "EMERGENCY_STOP"

ALL_STATES = [
    INIT, IDLE, START_SEARCH, FORWARD, CORNERING,
    OBSTACLE_AVOID, REVERSE, LAP_FINISHED, PARK, EMERGENCY_STOP,
]

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

STATE_HANDLERS = {
    INIT: "on_init",
    IDLE: "on_idle",
    START_SEARCH: "on_start_search",
    FORWARD: "on_forward",
    CORNERING: "on_cornering",
    OBSTACLE_AVOID: "on_obstacle_avoid",
    REVERSE: "on_reverse",
    LAP_FINISHED: "on_lap_finished",
    PARK: "on_park",
    EMERGENCY_STOP: "on_emergency_stop",
}

STATE_ENTERS = {
    INIT: "on_enter_init",
    IDLE: "on_enter_idle",
    START_SEARCH: "on_enter_start_search",
    FORWARD: "on_enter_forward",
    CORNERING: "on_enter_cornering",
    OBSTACLE_AVOID: "on_enter_obstacle_avoid",
    REVERSE: "on_enter_reverse",
    LAP_FINISHED: "on_enter_lap_finished",
    PARK: "on_enter_park",
    EMERGENCY_STOP: "on_enter_emergency_stop",
}


class StateMachine:
    def __init__(self):
        self.current_state = INIT
        self.state_start_time = time.monotonic()
        self.previous_state = None
        self.emergency_triggered = False

    def transition(self, new_state):
        if new_state == self.current_state:
            return
        if new_state not in TRANSITIONS.get(self.current_state, []):
            valid = TRANSITIONS.get(self.current_state, [])
            raise ValueError(
                f"Illegal transition: {self.current_state} -> {new_state}. "
                f"Allowed from {self.current_state}: {valid}"
            )
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.monotonic()

    @property
    def elapsed_in_state(self):
        elapsed = time.monotonic() - self.state_start_time
        return max(0.0, elapsed)

    def handle_state(self):
        handler_name = STATE_HANDLERS.get(self.current_state)
        if handler_name is None:
            raise KeyError(f"No handler registered for state: {self.current_state}")
        handler = getattr(self, handler_name, None)
        if handler is None:
            raise AttributeError(
                f"Handler method '{handler_name}' not found for state "
                f"{self.current_state}"
            )
        handler()

    def emergency_stop(self):
        self.transition(EMERGENCY_STOP)
        self.emergency_triggered = True

    def can_transition(self, target_state):
        return target_state in TRANSITIONS.get(self.current_state, [])

    def get_state(self):
        return self.current_state

    def get_previous_state(self):
        return self.previous_state

    def reset(self):
        self.current_state = INIT
        self.state_start_time = time.monotonic()
        self.previous_state = None
        self.emergency_triggered = False

    def on_init(self):
        if self.elapsed_in_state > 1.0:
            self.transition(IDLE)

    def on_idle(self):
        pass

    def on_start_search(self):
        if self.elapsed_in_state > 5.0:
            self.transition(FORWARD)

    def on_forward(self):
        pass

    def on_cornering(self):
        if self.elapsed_in_state > 2.0:
            self.transition(FORWARD)

    def on_obstacle_avoid(self):
        if self.elapsed_in_state > 3.0:
            self.transition(FORWARD)

    def on_reverse(self):
        if self.elapsed_in_state > 1.5:
            self.transition(FORWARD)

    def on_lap_finished(self):
        if self.elapsed_in_state > 0.5:
            self.transition(FORWARD)

    def on_park(self):
        pass

    def on_emergency_stop(self):
        if not self.emergency_triggered:
            self.emergency_triggered = True

    def on_enter_init(self):
        pass

    def on_enter_idle(self):
        pass

    def on_enter_start_search(self):
        pass

    def on_enter_forward(self):
        pass

    def on_enter_cornering(self):
        pass

    def on_enter_obstacle_avoid(self):
        pass

    def on_enter_reverse(self):
        pass

    def on_enter_lap_finished(self):
        pass

    def on_enter_park(self):
        pass

    def on_enter_emergency_stop(self):
        pass
