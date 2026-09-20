import types
import time
import threading
import builtins
import pytest

import central_hub.central_hub as ch_mod
from test_central_hub_charac import ros_context  # reuse rclpy init/teardown fixture
from fixtures import _central_hub_params


class _DummyTimer:
    def __init__(self):
        self.reset_called = False
        self.cancel_called = False

    def reset(self):
        self.reset_called = True

    def cancel(self):
        self.cancel_called = True


def test_on_activate_resets_existing_timer(monkeypatch, ros_context):
    """If a heartbeat timer already exists, on_activate should call reset() on it."""
    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    # ensure active false initially
    hub.active = False

    dummy = _DummyTimer()
    hub._heartbeat_timer = dummy

    # Call on_activate and assert reset was used
    res = hub.on_activate(None)
    assert res is not None
    assert hub.active is True
    assert dummy.reset_called, "Existing heartbeat timer should be reset on activate"


def test_on_shutdown_cancels_timer(monkeypatch, ros_context):
    """on_shutdown should cancel an existing heartbeat timer and set _finalized."""
    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    dummy = _DummyTimer()
    hub._heartbeat_timer = dummy

    # Ensure finalized False
    hub._finalized = False

    res = hub.on_shutdown(None)
    assert res is not None
    assert hub._finalized is True
    assert dummy.cancel_called, "Heartbeat timer should be cancelled on shutdown"
    assert hub._heartbeat_timer is None


def test_main_loop_with_mocks(monkeypatch):
    """Validate main()'s wiring with rclpy/threading/CentralHub mocked out.
    main() only constructs the hub and calls hub.run(); the run-loop branches are tested
    against CentralHub.run() directly below."""

    calls = {
        'init': 0,
        'shutdown': 0,
        'destroy_node': 0,
        'start_auto_management': 0,
        'run': 0,
    }

    def fake_init(args=None):
        calls['init'] += 1

    def fake_shutdown():
        calls['shutdown'] += 1

    def fake_spin(node):
        return

    monkeypatch.setattr(ch_mod.rclpy, 'init', fake_init)
    monkeypatch.setattr(ch_mod.rclpy, 'shutdown', fake_shutdown)
    monkeypatch.setattr(ch_mod.rclpy, 'spin', fake_spin)

    # Replace threading.Thread so no real thread is spawned
    class DummyThread:
        def __init__(self, target=None, args=(), daemon=None):
            return

        def start(self):
            return

        def join(self, timeout=None):
            return

    monkeypatch.setattr(ch_mod.threading, 'Thread', DummyThread)

    class FakeLifecycleManager:
        def start_auto_management(self):
            calls['start_auto_management'] += 1

    class FakeHub:
        def __init__(self, node_name=None, parameters=None):
            self.lifecycle_manager = FakeLifecycleManager()

        def run(self):
            calls['run'] += 1

        def destroy_node(self):
            calls['destroy_node'] += 1

    # Patch CentralHub constructor to return our FakeHub instance
    monkeypatch.setattr(ch_mod, 'CentralHub', FakeHub)

    # Now call main(); it should return cleanly
    ch_mod.main(args=None)

    assert calls['init'] == 1
    assert calls['start_auto_management'] == 1
    assert calls['run'] == 1
    assert calls['shutdown'] == 1
    assert calls['destroy_node'] == 1


def test_on_configure_exception_branch(ros_context):
    from rclpy.lifecycle import TransitionCallbackReturn

    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    def bad_setup():
        raise RuntimeError("setup failed")

    hub.publisher_manager.setup_publishers = bad_setup
    result = hub.on_configure(None)
    hub.destroy_node()

    assert result == TransitionCallbackReturn.ERROR


def test_on_activate_exception_branch(ros_context):
    from rclpy.lifecycle import TransitionCallbackReturn

    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())
    hub._heartbeat_timer = None

    def bad_event(*_a, **_k):
        raise RuntimeError("event failed")

    hub.publisher_manager.publish_event = bad_event
    result = hub.on_activate(None)
    hub.destroy_node()

    assert result == TransitionCallbackReturn.ERROR


def test_on_deactivate_exception_branch(ros_context):
    from rclpy.lifecycle import TransitionCallbackReturn

    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    def bad_event(*_a, **_k):
        raise RuntimeError("event failed")

    hub.publisher_manager.publish_event = bad_event
    result = hub.on_deactivate(None)
    hub.destroy_node()

    assert result == TransitionCallbackReturn.ERROR


def test_on_cleanup_exception_branch(ros_context):
    from rclpy.lifecycle import TransitionCallbackReturn

    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    def bad_cleanup():
        raise RuntimeError("cleanup failed")

    hub.sensor_handler.cleanup_subscriptions = bad_cleanup
    result = hub.on_cleanup(None)
    hub.destroy_node()

    assert result == TransitionCallbackReturn.ERROR


def test_on_shutdown_failure_branch(ros_context):
    from rclpy.lifecycle import TransitionCallbackReturn

    hub = ch_mod.CentralHub(node_name="central_hub_node", parameters=_central_hub_params())

    class BadTimer:
        def cancel(self):
            raise RuntimeError("cancel failed")

    hub._heartbeat_timer = BadTimer()
    result = hub.on_shutdown(None)
    hub.destroy_node()

    assert result == TransitionCallbackReturn.FAILURE


# The tests below call CentralHub.run() directly against a lightweight stub (same technique as
# test_central_hub_source_coverage.py), since the run loop moved out of main() into hub.run().


class _RunStub:
    """Minimal stand-in satisfying everything CentralHub.run() touches."""

    def __init__(self, finalized=False, active=True, recharging=False, sleep_effect=None):
        self._finalized = finalized
        self.config = types.SimpleNamespace(frequency=10.0)
        self.battery_manager = types.SimpleNamespace(
            is_recharging=recharging,
            recharge=lambda: None,
        )
        self._active = active
        self._sleep_effect = sleep_effect
        self.detect_calls = 0
        self.sleep_calls = 0

    def create_rate(self, _frequency):
        stub = self

        class Rate:
            def sleep(self_inner):
                stub.sleep_calls += 1
                if stub._sleep_effect is not None:
                    raise stub._sleep_effect

        return Rate()

    def is_active(self):
        return self._active

    def detect(self):
        self.detect_calls += 1

    def get_logger(self):
        class L:
            def debug(self, *_a, **_k):
                return

        return L()


def test_main_finalized_break_branch(monkeypatch):
    """run() should break out of its loop immediately when _finalized is set."""
    monkeypatch.setattr(ch_mod.rclpy, 'ok', lambda: True)
    stub = _RunStub(finalized=True)

    ch_mod.CentralHub.run(stub)

    assert stub.detect_calls == 0
    assert stub.sleep_calls == 0


def test_main_active_detect_branch(monkeypatch):
    """run() should call detect() once per iteration while active and not recharging."""
    ok_values = [True, False]
    monkeypatch.setattr(ch_mod.rclpy, 'ok', lambda: ok_values.pop(0) if ok_values else False)
    stub = _RunStub(active=True, recharging=False)

    ch_mod.CentralHub.run(stub)

    assert stub.detect_calls == 1
    assert stub.sleep_calls == 1


def test_main_keyboardinterrupt_branch(monkeypatch):
    """run() should swallow a KeyboardInterrupt raised from rate.sleep() and return."""
    monkeypatch.setattr(ch_mod.rclpy, 'ok', lambda: True)
    stub = _RunStub(active=True, sleep_effect=KeyboardInterrupt())

    # Should not raise.
    ch_mod.CentralHub.run(stub)

    assert stub.detect_calls == 1
