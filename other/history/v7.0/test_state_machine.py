import pytest
import time
from state_machine import StateMachine


class TestStateMachine:
    def test_initial_state(self):
        sm = StateMachine()
        assert sm.get_state() == StateMachine.INIT

    def test_init_to_idle(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        assert sm.get_state() == StateMachine.IDLE

    def test_illegal_init_to_driving(self):
        sm = StateMachine()
        with pytest.raises(ValueError):
            sm.transition(StateMachine.DRIVING)

    def test_illegal_init_to_stop(self):
        sm = StateMachine()
        with pytest.raises(ValueError):
            sm.transition(StateMachine.STOP)

    def test_idle_to_driving(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        sm.transition(StateMachine.DRIVING)
        assert sm.get_state() == StateMachine.DRIVING

    def test_idle_to_stop(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        sm.transition(StateMachine.STOP)
        assert sm.get_state() == StateMachine.STOP

    def test_driving_to_stop(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        sm.transition(StateMachine.DRIVING)
        sm.transition(StateMachine.STOP)
        assert sm.get_state() == StateMachine.STOP

    def test_stop_to_init(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        sm.transition(StateMachine.STOP)
        sm.transition(StateMachine.INIT)
        assert sm.get_state() == StateMachine.INIT

    def test_self_transition_does_nothing(self):
        sm = StateMachine()
        sm.transition(StateMachine.IDLE)
        sm.transition(StateMachine.IDLE)
        assert sm.get_state() == StateMachine.IDLE

    def test_init_auto_transition_after_timeout(self):
        sm = StateMachine()
        sm.state_start_time = time.monotonic() - 2.0
        sm.update()
        assert sm.get_state() == StateMachine.IDLE

    def test_timer_tracks_state_duration(self):
        sm = StateMachine()
        time.sleep(0.1)
        sm.update()
        assert sm.get_state_time() > 0.0
