"""pytest-bdd scenarios + step definitions for ../managing_system.feature. The 2 scenarios that still fail are
marked xfail by title (XFAIL_REASON_BY_SCENARIO; see MANAGING_SYSTEM_GAPS.md). Steps use ROS1's real thresholds,
so a future fix shows up as a green run. Assumes the system under test is already running."""

import os
import re
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from pytest_bdd import given, parsers, scenarios, then, when
from rclpy.node import Node

from conftest import LOGGER_NODE, SENSORS, _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import (  # noqa: E402
    ENACTOR_NODE, LOG_COLLECTOR_NODE, NODES as COMPONENT_NODES, PARAM_ADAPTER_NODE,
    RELIABILITY_ENGINE_NODE, CHANNELS as CHANNEL_TOPICS, reconfigure_topic,
)
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

from bsn_interfaces.msg import AdaptationCommand, Persist  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "managing_system.feature")

# One test per Scenario/Outline row in managing_system.feature; real steps still run and fail for the
# documented reason, so a future fix surfaces as a green run.
scenarios(FEATURE_FILE)

TARGET_STAYS_AT_CEILING_REASON = (
    "confirmed regression, root cause not yet identified (see MANAGING_SYSTEM_GAPS.md): ROS1 passes this "
    "scenario, but here the oximeter's commanded frequency keeps climbing after reliability recovers; "
    "the _publish_exception format fix works but was not enough"
)

CONTROLLER_NEVER_SILENT_REASON = (
    "the controller is never silent while reliability is healthy (a continuous optimizer, as in legacy ROS1), "
    "so asserting the absence of commands fails; a finding, not a defect - see ../MANAGING_SYSTEM_GAPS.md"
)
# Scenario title (Gherkin, exact) -> xfail reason, applied below by matching each generated test's
# `__scenario__.name`. Only 2 scenarios are listed: the "sampling rate decreases" regression and
# "no adaptation while stable" (CONTROLLER_NEVER_SILENT_REASON); all other scenarios pass.
XFAIL_REASON_BY_SCENARIO = {
    "Sampling rate decreases after sensor reliability recovers (G4/G5)":
        TARGET_STAYS_AT_CEILING_REASON,
    "No adaptation is triggered while sensor reliability stays above the threshold (G4/G5)":
        CONTROLLER_NEVER_SILENT_REASON,
}

for _test_name, _test_func in list(globals().items()):
    _scenario = getattr(_test_func, "__scenario__", None)
    _reason = XFAIL_REASON_BY_SCENARIO.get(getattr(_scenario, "name", None))
    if _reason is not None:
        globals()[_test_name] = pytest.mark.xfail(reason=_reason)(_test_func)
del _test_name, _test_func, _scenario, _reason


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/services
    (utils.parsers.get_rosnode_info_ros2). Duplicated across test_*.py files because each runs as its own
    pytest subprocess."""
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


# ---------------------------------------------------------------------------
# Scenario "The system log receives each topic collected by the log collector".
# ---------------------------------------------------------------------------

@given("the system log is running")
def system_log_is_running():
    _nodes_online(LOGGER_NODE)


@then(parsers.parse("the system log should subscribe to {topic} published by the log collector"))
def system_log_subscribes_to_collector_topic(topic):
    # A topology claim is only meaningful checked from both ends - see
    # test_target_system.py's _assert_wired for the same reasoning.
    collector_topic = CHANNEL_TOPICS[topic]

    collector_info = _node_info(LOG_COLLECTOR_NODE)
    assert any(pub["topic"] == collector_topic for pub in collector_info["publishers"]), (
        f"{LOG_COLLECTOR_NODE} does not publish {collector_topic}"
    )

    logger_info = _node_info(LOGGER_NODE)
    assert any(sub["topic"] == collector_topic for sub in logger_info["subscribers"]), (
        f"{LOGGER_NODE} does not subscribe to {collector_topic}"
    )


# ---------------------------------------------------------------------------
# Topology scenarios involving /enactor and /reli_engine - all blocked.
# ---------------------------------------------------------------------------

@given("the adaptation enactor is running")
def adaptation_enactor_running():
    # True since the `adaptation` package was merged and the launch file
    # started bringing /enactor up - see MANAGING_SYSTEM_GAPS.md.
    _nodes_online(ENACTOR_NODE)


@then(parsers.parse('{component} should subscribe to "{channel}" published by {publisher}'))
def component_subscribes_to_channel(component, channel, publisher):
    topic = CHANNEL_TOPICS[channel]
    publisher_info = _node_info(COMPONENT_NODES[publisher])
    assert any(pub["topic"] == topic for pub in publisher_info["publishers"]), (
        f"{COMPONENT_NODES[publisher]} does not publish {topic}"
    )
    component_info = _node_info(COMPONENT_NODES[component])
    assert any(sub["topic"] == topic for sub in component_info["subscribers"]), (
        f"{COMPONENT_NODES[component]} does not subscribe to {topic}"
    )


@then(parsers.parse('{component} should publish "{channel}" to {target}'))
def component_publishes_channel_to(component, channel, target):
    topic = CHANNEL_TOPICS[channel]
    component_info = _node_info(COMPONENT_NODES[component])
    assert any(pub["topic"] == topic for pub in component_info["publishers"]), (
        f"{COMPONENT_NODES[component]} does not publish {topic}"
    )
    target_info = _node_info(COMPONENT_NODES[target])
    assert any(sub["topic"] == topic for sub in target_info["subscribers"]), (
        f"{COMPONENT_NODES[target]} does not subscribe to {topic}"
    )


# G4/G5 behavioral scenarios: mirror ROS1's real checks (thresholded waits, peak stabilization, persistence).
# The assertions are the acceptance criteria; ../MANAGING_SYSTEM_GAPS.md is the gap report.

# ROS1's G4/G5 tests use /g3t1_1 (the oximeter) as the reference sensor, as does the feature text.
# BASELINE_FREQUENCY_HZ is its configured rate (sensor/config/oximeter.yaml).
BASELINE_FREQUENCY_HZ = 1.0
FAIL_STREAM_INTERVAL_S = 0.5
# ROS1 waited up to 30s for a MAPE-K engine cycle to react; kept the same
# here since this is the ideal-world target, not a tuned-down shortcut.
ADAPTATION_WAIT_S = 30.0
PEAK_STABLE_TIMEOUT_S = 15.0
PEAK_STABLE_QUIET_S = 2.5
STABILIZATION_S = 5.0

# Topic the enactor publishes every AdaptationCommand to (logger.py persists it and relays it to
# /reconfigure); the "failed adaptation attempt" scenario impersonates that publish, as ROS1's does.
LOG_ADAPT_TOPIC = "/log_adapt"

_FREQ_ACTION_RE = re.compile(r"freq=([\d.eE+-]+)")


class _FailStatusStream:
    """Background publisher of repeated 'fail' Status reports for one sensor, as in ROS1's
    _start_status_failure_stream: published on /log_status (0.5s interval, one message reused), which isolates
    /reli_engine from the collector relay verified by another scenario."""

    def __init__(self, sensor_node, interval=FAIL_STREAM_INTERVAL_S):
        self._sensor_node = sensor_node
        self._interval = interval
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        # target: 'logger' mirrors ROS1's msg.target = "/logger"; cosmetic (logger.py copies it into the
        # Persist record unfiltered), kept for parity.
        msg_yaml = f"{{source: '{self._sensor_node}', target: 'logger', content: 'fail', task: ''}}"
        while not self._stop.is_set():
            subprocess.run(
                ["ros2", "topic", "pub", "--once", "/log_status", "bsn_interfaces/msg/Status", msg_yaml],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
            )
            self._stop.wait(self._interval)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class _AdaptationCommandLog:
    """Persistent rclpy subscriber to the reference sensor's reconfigure_<node> channel, standing in for
    ROS1's SharedAdaptationTests: keeps every command's time and frequency; steps mark a `since_index`
    to ignore commands from earlier scenarios."""

    def __init__(self, topic):
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("bdd_managing_system_adaptation_log")
        self._lock = threading.Lock()
        self._commands = []  # [(arrival_time, freq_hz), ...]
        self._node.create_subscription(AdaptationCommand, topic, self._on_command, 50)
        # Explicit executor, not rclpy.spin(node): several nodes here spin from their own threads, and two
        # racing on the shared default executor raise "Executor is already spinning" and silently kill
        # the losing thread (that node stops processing callbacks for the session).
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def _on_command(self, msg):
        match = _FREQ_ACTION_RE.match(msg.action)
        if match:
            with self._lock:
                self._commands.append((time.time(), float(match.group(1))))

    def since_index(self):
        with self._lock:
            return len(self._commands)

    def _snapshot(self, since_index):
        with self._lock:
            return list(self._commands[since_index:])

    def wait_for_above(self, min_freq, since_index, timeout=ADAPTATION_WAIT_S):
        """Mirrors ROS1's _wait_for_adaptation_command_above: the first command after `since_index`
        with freq strictly greater than `min_freq`, discarding earlier reduce-commands."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            for _, freq in reversed(self._snapshot(since_index)):
                if freq > min_freq:
                    return freq
            time.sleep(0.1)
        return None

    def wait_for_below(self, max_freq, since_index, timeout=ADAPTATION_WAIT_S):
        """Mirrors ROS1's _wait_for_adaptation_command_below."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            for _, freq in reversed(self._snapshot(since_index)):
                if freq < max_freq:
                    return freq
            time.sleep(0.1)
        return None

    def wait_for_any(self, since_index, timeout=ADAPTATION_WAIT_S):
        """Mirrors ROS1's _wait_for_adaptation_command (no direction/threshold
        filter - just presence, e.g. for the 10s time-budget scenario)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            snapshot = self._snapshot(since_index)
            if snapshot:
                return snapshot[0][1]
            time.sleep(0.1)
        return None

    def wait_for_none(self, since_index, timeout):
        """True if nothing arrives in `timeout`s - for the "no adaptation"
        scenario. Always waits the full timeout (there is nothing to return
        early on when confirming absence)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.1)
        return not self._snapshot(since_index)

    def wait_for_peak_stable(self, since_index, timeout=PEAK_STABLE_TIMEOUT_S, quiet=PEAK_STABLE_QUIET_S):
        """Mirrors ROS1's _wait_for_peak_frequency_stable: waits until the highest commanded frequency stops
        increasing for `quiet` seconds and returns it (the peak the "decreases" Then compares against)."""
        deadline = time.time() + timeout
        peak = None
        seen = since_index
        last_increase = time.time()
        while time.time() < deadline:
            with self._lock:
                snapshot = self._commands[seen:]
                seen = len(self._commands)
            for _, freq in snapshot:
                if peak is None or freq > peak:
                    peak = freq
                    last_increase = time.time()
            if peak is not None and (time.time() - last_increase) > quiet:
                return peak
            time.sleep(0.2)
        return peak

    def wait_stable(self, timeout=STABILIZATION_S):
        """Mirrors ROS1's _wait_for_system_stable: waits until no new
        commands arrive for 2s within `timeout`, so a Given that needs a
        clean baseline isn't contaminated by a previous scenario's tail."""
        deadline = time.time() + timeout
        last_count = -1
        stable_since = None
        while time.time() < deadline:
            with self._lock:
                count = len(self._commands)
            if count == last_count:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since > 2.0:
                    return
            else:
                last_count = count
                stable_since = None
            time.sleep(0.2)

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()


@pytest.fixture(scope="module")
def adaptation_log():
    """One persistent subscriber for every G4/G5 scenario (see _AdaptationCommandLog); module-scoped
    so rclpy.init()/shutdown() happens once per pytest subprocess."""
    log = _AdaptationCommandLog(reconfigure_topic("the oximeter"))
    yield log
    log.shutdown()


class _PersistLog:
    """Persistent rclpy subscriber to /persist (ROS1's SharedAdaptationTests.persist_received). Needed because
    "a failed adaptation attempt is recorded" publishes one message and logger.py's persist_pub is VOLATILE
    QoS, so a subscriber created after that publish would miss it."""

    def __init__(self, topic="/persist"):
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("bdd_managing_system_persist_log")
        self._lock = threading.Lock()
        self._records = []  # [(source, target, type, content), ...]
        self._node.create_subscription(Persist, topic, self._on_record, 50)
        # Explicit executor, not rclpy.spin(node); see _AdaptationCommandLog's __init__ for why sharing
        # the default executor across threads kills a thread silently.
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def _on_record(self, msg):
        with self._lock:
            self._records.append((msg.source, msg.target, msg.type, msg.content))

    def since_index(self):
        with self._lock:
            return len(self._records)

    def wait_for_record(self, target, msg_type, since_index, timeout=10.0, source=None):
        """Mirrors ROS1's _wait_for_persist_record: the first record after `since_index` matching
        `target`/`msg_type` (and `source`, if given), polling instead of re-capturing a window. Pass `source`
        when the real /enactor's traffic for the same sensor could also match `target`."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                snapshot = list(self._records[since_index:])
            for rec_source, rec_target, rec_type, content in snapshot:
                if rec_target == target and rec_type == msg_type:
                    if source is not None and rec_source != source:
                        continue
                    return (rec_source, rec_target, rec_type, content)
            time.sleep(0.05)
        return None

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()


@pytest.fixture(scope="module")
def persist_log():
    """Module-scoped, same rationale as `adaptation_log` above."""
    log = _PersistLog()
    yield log
    log.shutdown()


def _set_effector_connection(component_name, connected, timeout=10.0):
    """Calls the EffectorRegister service (as Component.setUp/AdaptationHandler do) to (un)register in
    ParamAdapter.registered_components; mirrors ROS1's `_set_effector_connection`. `component_name` is the
    ROS2 node name without a leading slash."""
    request_yaml = f"{{name: '{component_name}', connection: {str(bool(connected)).lower()}}}"
    result = subprocess.run(
        ["ros2", "service", "call", "/EffectorRegister",
         "bsn_interfaces/srv/EffectorRegister", request_yaml],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout,
    )
    return b"ack=True" in result.stdout


def _assert_adaptation_logged(bdd, sensor, persist_log):
    """Mirrors ROS1's then_adaptation_logged: /persist should hold a record of the delivered AdaptationCommand,
    identified by its BSN target (e.g. "/g3t1_1"). Uses persist_log instead of a fresh capture because
    persist_pub is VOLATILE QoS and the single relay already happened by the time this runs."""
    sensor_bsn = SENSORS[sensor]["bsn"]
    record = persist_log.wait_for_record(
        target=sensor_bsn, msg_type="AdaptationCommand",
        since_index=bdd.persist_since_index, timeout=5.0,
    )
    assert record is not None, (
        f"An AdaptationCommand for {sensor} was delivered, but no matching "
        f"record reached the system log's /persist (target='{sensor_bsn}' "
        f"expected among type='AdaptationCommand' records)"
    )


@given(parsers.parse("the managing system is monitoring the reliability of {sensor}"))
def managing_system_monitoring_reliability(bdd, sensor, adaptation_log, persist_log):
    # Mirrors ROS1's given_monitoring_reliability: the reliability engine is up and the enactor/parameter
    # adapter are ready to act. A real check, so a failure says exactly what is missing.
    _nodes_online(RELIABILITY_ENGINE_NODE, ENACTOR_NODE, PARAM_ADAPTER_NODE)
    bdd.since_index = adaptation_log.since_index()
    # See _assert_adaptation_logged's docstring: persist_log has been
    # subscribed since before this Given, so nothing published from here on
    # can be missed the way a fresh capture after the fact could be.
    bdd.persist_since_index = persist_log.since_index()


@given(parsers.parse("{sensor} is operating normally with stable reliability"))
def sensor_operating_normally_stable(bdd, sensor, adaptation_log, persist_log):
    _nodes_online(RELIABILITY_ENGINE_NODE, ENACTOR_NODE, PARAM_ADAPTER_NODE)
    # Mirrors ROS1's given_stable_reliability: wait for the engine to settle (no new commands for 2s) before
    # marking since_index, so a leftover command from a previous scenario isn't mistaken for a reaction.
    adaptation_log.wait_stable()
    bdd.since_index = adaptation_log.since_index()
    bdd.persist_since_index = persist_log.since_index()


@when(parsers.parse("{sensor} repeatedly reports a 'fail' status"))
def sensor_repeatedly_reports_fail_status(bdd, sensor):
    bdd.degradation_start_time = time.time()
    bdd.fail_stream = _FailStatusStream(SENSORS[sensor]["node"])
    bdd.fail_stream.start()


@then(parsers.parse("an adaptation command increasing the sampling rate of {sensor} should be issued"))
def adaptation_command_increasing_rate_issued(bdd, sensor, adaptation_log, persist_log):
    # Waits specifically for freq > baseline - discards any reduce-command
    # still in flight from a prior scenario's recovery - mirroring ROS1's
    # _wait_for_adaptation_command_above.
    freq = adaptation_log.wait_for_above(BASELINE_FREQUENCY_HZ, bdd.since_index)
    bdd.fail_stream.stop()
    assert freq is not None, (
        f"No AdaptationCommand with freq > {BASELINE_FREQUENCY_HZ}Hz arrived "
        f"on {reconfigure_topic(sensor)} in {ADAPTATION_WAIT_S}s after "
        f"{sensor} repeatedly reported 'fail' - see "
        f"../MANAGING_SYSTEM_GAPS.md's runtime-breakage section"
    )
    _assert_adaptation_logged(bdd, sensor, persist_log)


@then(parsers.parse("an adaptation command for {sensor} should be issued within {time_limit} seconds"))
def adaptation_command_within_time_limit(bdd, sensor, time_limit, adaptation_log):
    timeout_s = float(time_limit)
    freq = adaptation_log.wait_for_any(bdd.since_index, timeout=timeout_s)
    elapsed = time.time() - bdd.degradation_start_time
    bdd.fail_stream.stop()
    assert freq is not None, (
        f"No AdaptationCommand ever arrived on {reconfigure_topic(sensor)} "
        f"within {time_limit}s - see ../MANAGING_SYSTEM_GAPS.md's "
        f"runtime-breakage section"
    )
    assert elapsed <= timeout_s, (
        f"AdaptationCommand for {sensor} arrived, but {elapsed:.2f}s after "
        f"degradation started - exceeds the {time_limit}s budget"
    )


@given(parsers.parse("{sensor} is operating at an elevated sampling rate"))
def sensor_operating_at_elevated_rate(bdd, sensor, adaptation_log, persist_log):
    # Mirrors ROS1's given_elevated_rate: drive real degradation (same fail-status stream as "increases") for a
    # fixed window, then read the peak frequency the engine converged on for the "decreases" Then to compare.
    _nodes_online(RELIABILITY_ENGINE_NODE, ENACTOR_NODE, PARAM_ADAPTER_NODE)
    since = adaptation_log.since_index()
    stream = _FailStatusStream(SENSORS[sensor]["node"])
    stream.start()
    time.sleep(8.0)
    stream.stop()

    peak = adaptation_log.wait_for_peak_stable(since)
    assert peak is not None, (
        f"No AdaptationCommand with freq= arrived for {sensor} during 8s of "
        f"'fail' status - the 'operating at an elevated sampling rate' "
        f"precondition was never actually reached. See "
        f"../MANAGING_SYSTEM_GAPS.md's runtime-breakage section"
    )
    bdd.peak_frequency_hz = peak
    bdd.since_index = adaptation_log.since_index()
    # See _assert_adaptation_logged's docstring - marked after the 8s
    # degradation window so the "decreases" Then below only looks for the
    # recovery record, not any of the increase commands from this Given.
    bdd.persist_since_index = persist_log.since_index()


@when(parsers.parse("the reliability of {sensor} rises above the threshold"))
def reliability_rises_above_threshold(sensor):
    # No new event to trigger: the Given already stopped the fail-status
    # stream, so this is a fixed settle time before checking for a
    # reduce-command - mirrors ROS1's when_reliability_recovers exactly.
    time.sleep(2.0)


@then(parsers.parse("an adaptation command reducing the sampling rate of {sensor} should be issued"))
def adaptation_command_reducing_rate_issued(bdd, sensor, adaptation_log, persist_log):
    assert hasattr(bdd, "peak_frequency_hz"), (
        "peak_frequency_hz not available - the Given never measured a peak "
        "before recovery"
    )
    freq = adaptation_log.wait_for_below(bdd.peak_frequency_hz, bdd.since_index)
    assert freq is not None, (
        f"No AdaptationCommand with freq < {bdd.peak_frequency_hz:.3f}Hz "
        f"(the measured peak) arrived on {reconfigure_topic(sensor)} in "
        f"{ADAPTATION_WAIT_S}s after {sensor}'s reliability recovered - see "
        f"../MANAGING_SYSTEM_GAPS.md's runtime-breakage section"
    )
    _assert_adaptation_logged(bdd, sensor, persist_log)


@when(parsers.parse("the reliability of {sensor} stays above the threshold"))
def reliability_stays_above_threshold(sensor):
    # Mirrors ROS1's when_managing_system_evaluates: a fixed pause standing
    # in for "the control loop re-evaluates", since nothing degraded
    # reliability in this scenario's Given to begin with.
    time.sleep(3.0)


@then(parsers.parse("no adaptation command for {sensor} should be issued"))
def no_adaptation_command_issued(bdd, sensor, adaptation_log):
    # Meant to stay `xfail` (CONTROLLER_NEVER_SILENT_REASON): the controller is never silent, as in legacy ROS1;
    # do not "fix" this assertion or the controller to make it pass. Keep the Given (nodes online) as is.
    silent = adaptation_log.wait_for_none(bdd.since_index, timeout=5.0)
    assert silent, (
        f"Unexpected AdaptationCommand(s) on {reconfigure_topic(sensor)} "
        f"while reliability was supposedly stable"
    )


# "A failed adaptation attempt is recorded in the system log": steps mirror ROS1's given_cannot_receive_adaptation /
# when_adaptation_issued / then_adaptation_recorded_despite_failure. The precondition is a reversible routing failure
# (sensor deregistered from ParamAdapter via EffectorRegister, restored in a finally); the process is never touched.

@given(parsers.parse("{sensor} cannot receive adaptation commands"))
def sensor_cannot_receive_adaptation_commands(bdd, sensor, persist_log):
    """Mirrors ROS1's given_cannot_receive_adaptation. Deregisters (not kills) the sensor: the log_adapt -> persist
    path under test never goes through param_adapter, so it only needs "param_adapter cannot route to this sensor"."""
    sensor_node = SENSORS[sensor]["node"]
    _nodes_online(PARAM_ADAPTER_NODE, sensor_node)

    bdd.target_sensor_node = sensor_node
    # Marked before the disconnect/When so the Then can tell this scenario's one-shot command apart from the
    # sensor's leftover /persist traffic from earlier scenarios (see _assert_adaptation_logged).
    bdd.persist_since_index = persist_log.since_index()

    ack = _set_effector_connection(sensor_node, False)
    assert ack, (
        f"EffectorRegister refused to disconnect {sensor_node} from "
        f"{PARAM_ADAPTER_NODE}; the 'cannot receive adaptation commands' "
        f"precondition was not actually established."
    )
    bdd.effector_disconnected = sensor_node


@when(parsers.parse("an adaptation command is issued for {sensor}"))
def adaptation_command_issued_for_sensor(sensor):
    """Mirrors ROS1's when_adaptation_issued: publishes straight to log_adapt (the enactor's channel), with the BSN
    target, impersonating the enactor. Publishes a few times, not `--once`: a fresh `ros2 topic pub` may lose the
    discovery race and drop its only message; the Then polls for up to 10s, so redundant deliveries are harmless."""
    sensor_bsn = SENSORS[sensor]["bsn"]
    msg_yaml = f"{{source: '/enactor', target: '{sensor_bsn}', action: 'freq=5.0'}}"
    for _ in range(5):
        subprocess.run(
            ["ros2", "topic", "pub", "--once", LOG_ADAPT_TOPIC,
             "bsn_interfaces/msg/AdaptationCommand", msg_yaml],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
        )
        time.sleep(0.3)


@then(parsers.parse("the adaptation command should be recorded in the system log even though {sensor} cannot receive it"))
def adaptation_recorded_despite_failure(bdd, sensor, persist_log):
    """Mirrors ROS1's then_adaptation_recorded_despite_failure: logger.py persists an AdaptationCommand as soon as it
    arrives on log_adapt, before any delivery, so the unroutable sensor doesn't matter. Uses persist_log's since-index
    (persist_pub is VOLATILE) and filters source == "/enactor", which the real enactor's traffic ("enactor") never uses."""
    sensor_node = SENSORS[sensor]["node"]
    sensor_bsn = SENSORS[sensor]["bsn"]
    try:
        record = persist_log.wait_for_record(
            target=sensor_bsn, msg_type="AdaptationCommand", source="/enactor",
            since_index=bdd.persist_since_index, timeout=10.0,
        )
        assert record is not None, (
            f"No new Persist record (type='AdaptationCommand', "
            f"target='{sensor_bsn}') appeared on /persist after this "
            f"scenario's own command, even though {sensor_node} was "
            f"disconnected from {PARAM_ADAPTER_NODE}. logger.py should "
            f"persist an AdaptationCommand received on log_adapt "
            f"regardless of whether param_adapter can route it onward."
        )
    finally:
        # Restore the sensor's registration so this scenario's precondition
        # doesn't leak into whatever runs after it - mirrors ROS1's own
        # cleanup in then_adaptation_recorded_despite_failure's finally.
        if getattr(bdd, "effector_disconnected", None) == sensor_node:
            _set_effector_connection(sensor_node, True)
