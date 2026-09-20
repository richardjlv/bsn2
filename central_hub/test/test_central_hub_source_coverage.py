import importlib.util
import types
from pathlib import Path


def _load_source_module():
    # Resolve relative to this test file instead of a hardcoded absolute
    # workspace path, which broke as soon as this package moved to
    # src/bsn_ros2/central_hub (the path used to be src/central_hub).
    src = Path(__file__).resolve().parent.parent / 'central_hub' / 'central_hub.py'
    spec = importlib.util.spec_from_file_location('central_hub_source_cov', src)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class _Logger:
    def info(self, *a, **k):
        return

    def error(self, *a, **k):
        return

    def debug(self, *a, **k):
        return


class _PM:
    def setup_publishers(self):
        return

    def publish_status(self, *a, **k):
        return

    def publish_event(self, *a, **k):
        return

    def publish_heartbeat(self, *a, **k):
        return


class _SH:
    def setup_subscriptions(self):
        return

    def cleanup_subscriptions(self):
        return

    latest_data = {}
    latest_risk = {}
    sensor_battery_levels = {}


class _Timer:
    def __init__(self):
        self.canceled = False
        self.reset_called = False

    def cancel(self):
        self.canceled = True

    def reset(self):
        self.reset_called = True


class _HubStub:
    def __init__(self):
        self.active = False
        self._finalized = False
        self._heartbeat_timer = None
        self.publisher_manager = _PM()
        self.sensor_handler = _SH()
        self.battery_manager = types.SimpleNamespace(is_recharging=False)

    def get_logger(self):
        return _Logger()

    def create_timer(self, *_a, **_k):
        return _Timer()

    def is_active(self):
        return self.active


def test_source_on_configure_exception_branch():
    mod = _load_source_module()
    hub = _HubStub()

    def bad_setup():
        raise RuntimeError('x')

    hub.publisher_manager.setup_publishers = bad_setup

    res = mod.CentralHub.on_configure(hub, None)
    assert res == mod.TransitionCallbackReturn.ERROR


def test_source_on_activate_exception_branch():
    mod = _load_source_module()
    hub = _HubStub()

    def bad_publish_event(*a, **k):
        raise RuntimeError('x')

    hub.publisher_manager.publish_event = bad_publish_event

    res = mod.CentralHub.on_activate(hub, None)
    assert res == mod.TransitionCallbackReturn.ERROR


def test_source_on_deactivate_exception_branch():
    mod = _load_source_module()
    hub = _HubStub()

    def bad_publish_event(*a, **k):
        raise RuntimeError('x')

    hub.publisher_manager.publish_event = bad_publish_event

    res = mod.CentralHub.on_deactivate(hub, None)
    assert res == mod.TransitionCallbackReturn.ERROR


def test_source_on_cleanup_exception_branch():
    mod = _load_source_module()
    hub = _HubStub()

    def bad_cleanup():
        raise RuntimeError('x')

    hub.sensor_handler.cleanup_subscriptions = bad_cleanup

    res = mod.CentralHub.on_cleanup(hub, None)
    assert res == mod.TransitionCallbackReturn.ERROR


def test_source_on_shutdown_failure_branch():
    mod = _load_source_module()
    hub = _HubStub()

    class BadTimer:
        def cancel(self):
            raise RuntimeError('x')

    hub._heartbeat_timer = BadTimer()

    res = mod.CentralHub.on_shutdown(hub, None)
    assert res == mod.TransitionCallbackReturn.FAILURE


def test_source_main_keyboardinterrupt_branch(monkeypatch):
    """CentralHub.run() should swallow a KeyboardInterrupt raised from rate.sleep().

    This used to drive the branch through a mocked main(), back when main()
    contained the run loop inline. main() now only calls hub.run() (the loop
    branches live in CentralHub.run()), so the branch is exercised directly
    against the unbound run() method instead - same technique as the
    on_configure/on_activate/... tests above.
    """
    mod = _load_source_module()

    hub = _HubStub()
    hub.active = True
    hub.config = types.SimpleNamespace(frequency=1.0)
    hub.detect = lambda: None

    def _raise_keyboard_interrupt(*_a, **_k):
        raise KeyboardInterrupt()

    hub.create_rate = lambda _f: types.SimpleNamespace(sleep=_raise_keyboard_interrupt)
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True)

    # Should not raise.
    mod.CentralHub.run(hub)
