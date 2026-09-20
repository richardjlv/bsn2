import time
from types import SimpleNamespace

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter

from sensor.sensor import Sensor
from shared_components.test_components.node_setups import (
    setup_effector_register_service,
)


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def sensor_node(ros_context):
    import os
    import yaml
    from bsn_interfaces.srv import PatientData
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    import threading
    from contextlib import suppress
    from rclpy.executors import ExternalShutdownException

    params_path = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    with open(params_path, "r", encoding="utf-8") as stream:
        full_params = yaml.safe_load(stream)

    ros_params = full_params["thermometer_node"]["ros__parameters"]
    params = [Parameter(name=name, value=value) for name, value in ros_params.items()]

    suffix = str(int(time.time() * 1000))
    mock_service_node = Node(f"core_cov_mock_service_{suffix}")

    def mock_patient_service(request, response):
        response.datapoint = 37.0
        return response

    mock_service_node.create_service(PatientData, "/get_sensor_reading", mock_patient_service)

    # thermometer.yaml has enable_adaptation: true, which makes
    # RosComponent.__init__ block forever in
    # AdaptationHandler.register_with_effector() (wait_for_service on
    # 'EffectorRegister') unless that service is mocked *before* the Sensor
    # is constructed - reusing the same helper sensor/test/fixtures.py
    # already relies on for this, instead of leaving it out here.
    mock_effector_register = setup_effector_register_service()

    node = Sensor(f"core_cov_sensor_{suffix}", parameters=params)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_effector_register)
    executor.add_node(mock_service_node)
    executor.add_node(node)

    def spin_executor():
        with suppress(ExternalShutdownException, RuntimeError):
            executor.spin()

    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()

    yield node

    try:
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=1.0)
        mock_effector_register.destroy_node()
        mock_service_node.destroy_node()
        node.destroy_node()
    except Exception:
        pass


@pytest.mark.parametrize(
    "active_state, expected",
    [
        (False, False),
        (True, True),
    ],
)
def test_is_active_reflects_internal_state(sensor_node, active_state, expected):
    sensor_node.active = active_state
    assert sensor_node.is_active() is expected


def test_on_configure_error_path(sensor_node, monkeypatch):
    monkeypatch.setattr(sensor_node.publisher_manager, "setup_publishers", lambda: (_ for _ in ()).throw(RuntimeError("boom")))

    result = sensor_node.on_configure(SimpleNamespace())

    assert result.name == "ERROR"



@pytest.mark.parametrize(
    "field_name, expected, check_mode",
    [
        ("result_name", "SUCCESS", "equals"),
        ("active", True, "equals"),
        ("_heartbeat_timer", None, "not_none"),
    ],
)
def test_on_activate_state_and_result(sensor_node, field_name, expected, check_mode):
    sensor_node.trigger_configure()
    result = sensor_node.on_activate(SimpleNamespace())

    if field_name == "result_name":
        actual = result.name
    else:
        actual = getattr(sensor_node, field_name)

    if check_mode == "equals":
        assert actual == expected
    else:
        assert actual is not None


def test_on_activate_second_call_resets_timer(sensor_node):
    sensor_node.trigger_configure()
    sensor_node.on_activate(SimpleNamespace())

    timer = sensor_node._heartbeat_timer
    timer_info = {"reset_called": False}
    original_reset = timer.reset

    def wrapped_reset():
        timer_info["reset_called"] = True
        return original_reset()

    timer.reset = wrapped_reset
    try:
        sensor_node.on_activate(SimpleNamespace())
        assert timer_info["reset_called"] is True
    finally:
        timer.reset = original_reset


def test_on_cleanup_without_reset_method_still_succeeds(sensor_node):
    sensor_node.trigger_configure()
    sensor_node.trigger_activate()

    original_processor = sensor_node.processor
    sensor_node.processor = SimpleNamespace()
    try:
        result = sensor_node.on_cleanup(SimpleNamespace())
        assert result.name == "SUCCESS"
    finally:
        sensor_node.processor = original_processor


def test_on_shutdown_failure_path(sensor_node, monkeypatch):
    monkeypatch.setattr(sensor_node.lifecycle_manager, "stop_auto_management", lambda: (_ for _ in ()).throw(RuntimeError("shutdown_fail")))

    result = sensor_node.on_shutdown(SimpleNamespace())

    assert result.name == "FAILURE"



@pytest.mark.parametrize(
    "field_name, expected",
    [
        ("result_name", "SUCCESS"),
        ("active", False),
        ("_finalized", True),
        ("_heartbeat_timer", None),
    ],
)
def test_on_shutdown_state_and_result(sensor_node, field_name, expected):
    sensor_node.trigger_configure()
    sensor_node.trigger_activate()
    result = sensor_node.on_shutdown(SimpleNamespace())

    if field_name == "result_name":
        actual = result.name
    else:
        actual = getattr(sensor_node, field_name)

    assert actual == expected



def test_spin_sensor_starts_auto_management(sensor_node, monkeypatch):
    calls = []

    class FakeRate:
        def sleep(self):
            sensor_node._finalized = True

    monkeypatch.setattr(sensor_node, "create_rate", lambda *_: FakeRate())
    monkeypatch.setattr(sensor_node.lifecycle_manager, "start_auto_management", lambda: calls.append("start_auto_management"))
    sensor_node.active = True
    sensor_node._finalized = False
    sensor_node.battery_manager.is_recharging = False
    monkeypatch.setattr(sensor_node.processor, "collect", lambda: 37.0)
    monkeypatch.setattr(sensor_node.processor, "process", lambda value: value + 1.0)
    monkeypatch.setattr(sensor_node.processor, "transfer", lambda value: calls.append(("transfer", value)))

    monkeypatch.setattr(rclpy, "ok", lambda: True)

    sensor_node.run()

    assert "start_auto_management" in calls


def test_spin_sensor_performs_transfer_with_processed_value(sensor_node, monkeypatch):
    calls = []

    class FakeRate:
        def sleep(self):
            sensor_node._finalized = True

    monkeypatch.setattr(sensor_node, "create_rate", lambda *_: FakeRate())
    monkeypatch.setattr(sensor_node.lifecycle_manager, "start_auto_management", lambda: calls.append("start_auto_management"))
    sensor_node.active = True
    sensor_node._finalized = False
    sensor_node.battery_manager.is_recharging = False
    monkeypatch.setattr(sensor_node.processor, "collect", lambda: 37.0)
    monkeypatch.setattr(sensor_node.processor, "process", lambda value: value + 1.0)
    monkeypatch.setattr(sensor_node.processor, "transfer", lambda value: calls.append(("transfer", value)))

    monkeypatch.setattr(rclpy, "ok", lambda: True)

    sensor_node.run()

    assert ("transfer", 38.0) in calls


def test_spin_sensor_recharges_when_inactive(sensor_node, monkeypatch):
    calls = []

    class FakeRate:
        def sleep(self):
            sensor_node._finalized = True

    monkeypatch.setattr(sensor_node, "create_rate", lambda *_: FakeRate())
    monkeypatch.setattr(sensor_node.lifecycle_manager, "start_auto_management", lambda: calls.append("start_auto_management"))
    sensor_node.active = False
    sensor_node._finalized = False
    sensor_node.battery_manager.is_recharging = False
    monkeypatch.setattr(sensor_node.battery_manager, "recharge", lambda: calls.append("recharge"))
    monkeypatch.setattr(rclpy, "ok", lambda: True)

    sensor_node.run()

    assert "recharge" in calls


def test_spin_sensor_logs_processing_exception_and_continues_to_sleep(sensor_node, monkeypatch):
    errors = []

    class FakeRate:
        def sleep(self):
            sensor_node._finalized = True

    monkeypatch.setattr(sensor_node, "create_rate", lambda *_: FakeRate())
    monkeypatch.setattr(sensor_node.lifecycle_manager, "start_auto_management", lambda: None)
    monkeypatch.setattr(sensor_node.get_logger(), "error", lambda message: errors.append(message))
    sensor_node.active = True
    sensor_node._finalized = False
    sensor_node.battery_manager.is_recharging = False
    monkeypatch.setattr(sensor_node.processor, "collect", lambda: (_ for _ in ()).throw(RuntimeError("cycle_fail")))
    monkeypatch.setattr(rclpy, "ok", lambda: True)

    sensor_node.run()

    assert any("Sensor operation failed" in message for message in errors)


def test_spin_sensor_exits_immediately_when_finalized(sensor_node, monkeypatch):
    info_messages = []

    class FakeRate:
        def sleep(self):
            raise AssertionError("sleep should not be called")

    monkeypatch.setattr(sensor_node, "create_rate", lambda *_: FakeRate())
    monkeypatch.setattr(sensor_node.lifecycle_manager, "start_auto_management", lambda: None)
    monkeypatch.setattr(sensor_node.get_logger(), "info", lambda message: info_messages.append(message))
    sensor_node._finalized = True
    monkeypatch.setattr(rclpy, "ok", lambda: True)

    sensor_node.run()

    assert any("exiting spin loop" in message for message in info_messages)


def _run_main_and_collect_events(monkeypatch):
    import sensor.sensor as sensor_module

    events = []

    class FakeSensor:
        def __init__(self, node_name):
            events.append(("init_sensor", node_name))

        def run(self):
            # Sensor's real loop method is `run()` (required by
            # RosComponent); kept the "spin_sensor" event label below to
            # avoid rippling the change into test_main_flow_events'
            # parametrize list, which only cares about the event sequence.
            events.append("spin_sensor")

        def destroy_node(self):
            events.append("destroy_node")

    class FakeThread:
        def __init__(self, target=None, args=None, daemon=None):
            events.append(("thread_created", daemon, args))
            self.target = target
            self.args = args or ()

        def start(self):
            events.append("thread_started")

    monkeypatch.setattr(sensor_module, "Sensor", FakeSensor)
    monkeypatch.setattr(sensor_module.rclpy, "init", lambda args=None: events.append(("rclpy.init", args)))
    monkeypatch.setattr(sensor_module.rclpy, "spin", lambda node: events.append(("rclpy.spin", node)))
    monkeypatch.setattr(sensor_module.rclpy, "shutdown", lambda: events.append("rclpy.shutdown"))
    monkeypatch.setattr(sensor_module.threading, "Thread", FakeThread)

    sensor_module.main(args=["--ros-args"])
    return events


@pytest.mark.parametrize(
    "expected_event",
    [
        ("rclpy.init", ["--ros-args"]),
        ("init_sensor", "sensor_node"),
        "thread_started",
        "spin_sensor",
        "destroy_node",
        "rclpy.shutdown",
    ],
)
def test_main_flow_events(monkeypatch, expected_event):
    events = _run_main_and_collect_events(monkeypatch)

    assert expected_event in events
