import pytest
import time
from state_machine import (
    StateMachine, INIT, IDLE, START_SEARCH, FORWARD,
    CORNERING, OBSTACLE_AVOID, REVERSE, LAP_FINISHED,
    PARK, EMERGENCY_STOP, ALL_STATES, TRANSITIONS, STATE_HANDLERS,
)


class TestStateMachine:
    def test_initial_state(self):
        sm = StateMachine()
        assert sm.get_state() == INIT

    def test_transition_to_same_state(self):
        sm = StateMachine()
        sm.transition(INIT)
        assert sm.get_state() == INIT

    @pytest.mark.parametrize("state_from, state_to", [
        (INIT, IDLE),
        (IDLE, START_SEARCH),
        (START_SEARCH, FORWARD),
        (FORWARD, CORNERING),
        (FORWARD, OBSTACLE_AVOID),
        (FORWARD, LAP_FINISHED),
        (FORWARD, EMERGENCY_STOP),
        (CORNERING, FORWARD),
        (OBSTACLE_AVOID, FORWARD),
        (OBSTACLE_AVOID, REVERSE),
        (REVERSE, FORWARD),
        (LAP_FINISHED, FORWARD),
        (LAP_FINISHED, PARK),
        (EMERGENCY_STOP, IDLE),
    ])
    def test_legal_transitions(self, state_from, state_to):
        sm = StateMachine()
        sm.current_state = state_from
        sm.transition(state_to)
        assert sm.get_state() == state_to

    @pytest.mark.parametrize("state_from, state_to", [
        (INIT, START_SEARCH),
        (INIT, FORWARD),
        (INIT, PARK),
        (IDLE, FORWARD),
        (IDLE, PARK),
        (START_SEARCH, PARK),
        (CORNERING, OBSTACLE_AVOID),
        (REVERSE, CORNERING),
        (PARK, FORWARD),
        (PARK, IDLE),
    ])
    def test_illegal_transitions(self, state_from, state_to):
        sm = StateMachine()
        sm.current_state = state_from
        with pytest.raises(ValueError):
            sm.transition(state_to)

    def test_emergency_stop_from_any_state(self):
        for state in ALL_STATES:
            sm = StateMachine()
            sm.current_state = state
            if EMERGENCY_STOP in TRANSITIONS.get(state, []):
                sm.transition(EMERGENCY_STOP)
                assert sm.get_state() == EMERGENCY_STOP

    def test_timer_overflow_protection(self):
        sm = StateMachine()
        sm.state_start_time = time.monotonic() + 3600.0
        elapsed = sm.elapsed_in_state
        assert elapsed >= 0.0
        assert elapsed < 1.0

    def test_init_auto_transition(self):
        sm = StateMachine()
        sm.state_start_time = time.monotonic() - 2.0
        sm.handle_state()
        assert sm.get_state() == IDLE

    def test_previous_state_tracked(self):
        sm = StateMachine()
        sm.transition(IDLE)
        assert sm.get_previous_state() == INIT

    def test_reset(self):
        sm = StateMachine()
        sm.transition(IDLE)
        sm.transition(START_SEARCH)
        sm.reset()
        assert sm.get_state() == INIT
        assert sm.get_previous_state() is None
        assert sm.emergency_triggered is False

    @pytest.mark.parametrize("state", ALL_STATES)
    def test_every_state_has_handler(self, state):
        assert state in STATE_HANDLERS

    @pytest.mark.parametrize("state", ALL_STATES)
    def test_every_state_has_transitions_defined(self, state):
        assert state in TRANSITIONS
