import importlib.util
import types
from pathlib import Path


def _load_source_module():
    src = Path('/ros_ws/src/central_hub/central_hub/central_hub.py')
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
    mod = _load_source_module()

    calls = {'shutdown': 0, 'destroy': 0}

    class FakeLifecycle:
        def start_auto_management(self):
            return

    class FakeHub:
        def __init__(self):
            self.lifecycle_manager = FakeLifecycle()
            self._finalized = False
            self.config = types.SimpleNamespace(frequency=1.0)
            self.battery_manager = types.SimpleNamespace(
                is_recharging=False,
                battery=types.SimpleNamespace(current_level=42.0),
            )

        def is_active(self):
            return True

        def detect(self):
            return

        def create_rate(self, _f):
            class _Rate:
                def sleep(self_inner):
                    raise KeyboardInterrupt()

            return _Rate()

        def get_logger(self):
            return _Logger()

        def destroy_node(self):
            calls['destroy'] += 1

    class DummyThread:
        def __init__(self, *a, **k):
            return

        def start(self):
            return

        def join(self, timeout=None):
            return

    monkeypatch.setattr(mod, 'CentralHub', FakeHub)
    monkeypatch.setattr(mod.threading, 'Thread', DummyThread)
    monkeypatch.setattr(mod.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(mod.rclpy, 'spin', lambda _hub: None)
    monkeypatch.setattr(mod.rclpy, 'shutdown', lambda: calls.__setitem__('shutdown', calls['shutdown'] + 1))

    mod.main(args=None)

    assert calls['destroy'] == 1
    assert calls['shutdown'] == 1
