"""pytest-bdd scenarios + step definitions for ../managing_system.feature.

All 9 scenarios are collected via a single `scenarios()` call, same as every
other test_*.py file in this package; 8 come back marked `xfail` because
`/enactor`, `/reli_engine` and `/data_access` are still missing from this
ROS2 port - see ../MANAGING_SYSTEM_GAPS.md for the full breakdown and which
are tagged `@regression` vs. left untagged. Each of those 8 needs its own
specific reason (they are blocked for different underlying causes - see
MANAGING_SYSTEM_GAPS.md's scenario table), so this file marks them itself
right after `scenarios()` runs, by scenario title, instead of pytest-bdd's
usual per-scenario `@scenario(...)` decorator (which would need one stub
function per scenario just to hang a marker off of) - see
XFAIL_REASON_BY_SCENARIO, below.

The G4/G5 behavioral steps (below the topology section) are written against
an *ideal world* where the adaptation subsystem is fully merged and wired -
the same real thresholds, timeouts and persistence checks
bsn2/.../test_bdd/test_managing_system.py used against the working ROS1
system, not a shortcut that only proves a node is missing. This is
deliberate: these steps are the acceptance test the migration needs to keep
satisfying once `/reli_engine`/`/enactor`/`/data_access` are merged in (see
MANAGING_SYSTEM_GAPS.md's "Most of this may already be written" section) -
once that merge happens, deleting this file's XFAIL_REASON_BY_SCENARIO
mechanism entirely should be all that is needed, with no step-definition
changes, if the merge is faithful.

Same style as test_injector.py/test_knowledge_repo.py: real steps still run
and fail for the actual documented reason, rather than being hidden from
collection - so a future port that closes any of these gaps surfaces a
green run instead of a silently-passing one.

Assumes the system under test is already running - see conftest.py's module
docstring.
"""

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
from utils.parsers import capture_topic_data_with_timeout, get_rosnode_info_ros2  # noqa: E402

from bsn_interfaces.msg import AdaptationCommand  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "managing_system.feature")

# Generates one test per Scenario/Outline row in managing_system.feature -
# same as every other test_*.py file here. Real steps still run and fail for
# the actual documented reason, rather than being hidden from collection -
# so a future port that closes any of these gaps surfaces a green run
# instead of a silently-passing one.
scenarios(FEATURE_FILE)

MISSING_ENACTOR_REASON = (
    "no /enactor node exists in this ROS2 port - see ../MANAGING_SYSTEM_GAPS.md"
)
MISSING_RELIABILITY_ENGINE_REASON = (
    "no /reli_engine node exists in this ROS2 port - see ../MANAGING_SYSTEM_GAPS.md"
)
NO_RECONFIGURATION_PUBLISHER_REASON = (
    "route changed by design, and no originator exists yet: reconfiguration "
    "reaches targets here through per-component EffectorRegister/"
    "reconfigure_<node> channels (already covered, passing, by "
    "target_system.feature) rather than logger -> param_adapter, and nothing "
    "issues requests at all without /enactor - see ../MANAGING_SYSTEM_GAPS.md"
)
NO_AUTONOMOUS_ADAPTATION_REASON = (
    "no /reli_engine or /enactor exists to compute reliability or issue "
    "adaptation commands autonomously - see ../MANAGING_SYSTEM_GAPS.md"
)
NO_FAILURE_DETECTION_REASON = (
    "blocked twice over: no /enactor issues or logs adaptation commands at "
    "all (the regression), and - even once that is fixed - *no version* has "
    "ever produced a failure-specific record. ROS1's Controller published "
    "every command unconditionally, with no ack or delivery check, and its "
    "Logger persisted them all under one fixed type - see "
    "../MANAGING_SYSTEM_GAPS.md's row #7"
)

# Scenario title (Gherkin, exact) -> xfail reason. Applied below by walking
# every test function scenarios() just bound into this module's globals()
# and matching on its `__scenario__.name` (the attribute scenarios() itself
# uses internally to skip already-bound scenarios - see pytest_bdd/
# scenario.py) - not by guessing the auto-generated test_* name's slug,
# which is an internal, unstable detail of make_python_name().
#
# "The system log receives each topic collected by the log collector" and
# "No adaptation is triggered while sensor reliability stays above the
# threshold (G4/G5)" are deliberately absent: the first actually passes
# today: the second already carries its own `@xfail` tag in the Gherkin,
# which pytest-bdd applies as a plain `pytest.mark.xfail` automatically (see
# managing_system.feature's own comment on that scenario for why its reason
# lives there instead of here).
XFAIL_REASON_BY_SCENARIO = {
    "The adaptation enactor exchanges adaptation strategies with the reliability engine":
        MISSING_RELIABILITY_ENGINE_REASON,
    "The adaptation enactor exchanges adaptation records with the system log":
        MISSING_ENACTOR_REASON,
    "The system log sends reconfiguration requests to the parameter adapter":
        NO_RECONFIGURATION_PUBLISHER_REASON,
    "Sampling rate increases when sensor reliability falls below the threshold (G4/G5)":
        NO_AUTONOMOUS_ADAPTATION_REASON,
    "Sampling rate decreases after sensor reliability recovers (G4/G5)":
        NO_AUTONOMOUS_ADAPTATION_REASON,
    "A failed adaptation attempt is recorded in the system log (G4/G5)":
        NO_FAILURE_DETECTION_REASON,
    "An adaptation command is issued within 10 seconds after reliability degradation (G4/G5)":
        NO_AUTONOMOUS_ADAPTATION_REASON,
}

for _test_name, _test_func in list(globals().items()):
    _scenario = getattr(_test_func, "__scenario__", None)
    _reason = XFAIL_REASON_BY_SCENARIO.get(getattr(_scenario, "name", None))
    if _reason is not None:
        globals()[_test_name] = pytest.mark.xfail(reason=_reason)(_test_func)
del _test_name, _test_func, _scenario, _reason


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/
    services - see utils.parsers.get_rosnode_info_ros2. Duplicated (not
    imported) from the other test_*.py files here: each runs as its own
    pytest subprocess (see conftest.py's module docstring), so there's no
    shared module all of them would import from without introducing one
    anyway.
    """
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


# ---------------------------------------------------------------------------
# Scenario "The system log receives each topic collected by the log
# collector" - the one scenario that actually passes today.
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
    # Never true today - see MISSING_ENACTOR_REASON / MANAGING_SYSTEM_GAPS.md.
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


# ---------------------------------------------------------------------------
# G4/G5 behavioral scenarios.
#
# Written to the ideal-world target, mirroring bsn2/.../test_bdd/
# test_managing_system.py's actual verification logic exactly (thresholded
# waits for a command *above*/*below* a reference frequency, peak-frequency
# stabilization, persistence checks) rather than "any command at all". All
# five currently fail - no /reli_engine or /enactor exists - but the
# assertions themselves are the acceptance criteria, not the gap report; see
# ../MANAGING_SYSTEM_GAPS.md for the gap report itself.
# ---------------------------------------------------------------------------

# ROS1's own tests hardcoded /g3t1_1 (the oximeter) as the reference sensor
# for every G4/G5 scenario; managing_system.feature does the same via its
# literal "the oximeter" text. BASELINE_FREQUENCY_HZ is that sensor's
# configured rate (sensor/config/oximeter.yaml).
BASELINE_FREQUENCY_HZ = 1.0
FAIL_STREAM_INTERVAL_S = 0.5
# ROS1 waited up to 30s for a MAPE-K engine cycle to react; kept the same
# here since this is the ideal-world target, not a tuned-down shortcut.
ADAPTATION_WAIT_S = 30.0
PEAK_STABLE_TIMEOUT_S = 15.0
PEAK_STABLE_QUIET_S = 2.5
STABILIZATION_S = 5.0

_FREQ_ACTION_RE = re.compile(r"freq=([\d.eE+-]+)")


class _FailStatusStream:
    """Background publisher of repeated 'fail' Status reports for one sensor,
    matching ROS1's _start_status_failure_stream exactly - same injection
    point (published directly on /log_status, not the sensor's own
    /component_status), same 0.5s interval, same one-message-reused-forever
    shape. ROS1 injects at /log_status deliberately: that is precisely the
    topic `logger`/a future `data_access` consumes, so this exercises
    /reli_engine's reaction in isolation from the collector relay
    (component_status -> node_monitor -> log_status), which scenario #3
    ("the system log receives each topic collected by the log collector")
    already verifies separately. See ../MANAGING_SYSTEM_GAPS.md's
    architecture note for the full chain.
    """

    def __init__(self, sensor_node, interval=FAIL_STREAM_INTERVAL_S):
        self._sensor_node = sensor_node
        self._interval = interval
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        # target: 'logger' mirrors ROS1's msg.target = "/logger" - cosmetic
        # (logger.py's receive_status copies it into the Persist record
        # unfiltered; data_access, once merged, buckets status by `source`
        # only - see MANAGING_SYSTEM_GAPS.md), kept for parity with ROS1.
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
    """Persistent rclpy subscriber to the reference sensor's real
    reconfigure_<node> channel - the same end point target_system.feature's
    topology scenario already validates, and the one a real /enactor would
    ultimately have to deliver to (see MANAGING_SYSTEM_GAPS.md's naming-gap
    finding on why it doesn't, yet). Standing in for ROS1's
    SharedAdaptationTests: every command's arrival time and frequency is kept
    here, and steps mark a `since_index` (mirroring the ROS1 suite's own
    since_index) to look only at commands from after their own precondition,
    not ones left over from an earlier scenario in this session.
    """

    def __init__(self, topic):
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("bdd_managing_system_adaptation_log")
        self._lock = threading.Lock()
        self._commands = []  # [(arrival_time, freq_hz), ...]
        self._node.create_subscription(AdaptationCommand, topic, self._on_command, 50)
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
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
        """Mirrors ROS1's _wait_for_adaptation_command_above: the first
        command after `since_index` whose freq is strictly greater than
        `min_freq`, discarding earlier reduce-commands from before the
        engine converged on the degraded state."""
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
        """Mirrors ROS1's _wait_for_peak_frequency_stable: waits until the
        highest frequency commanded so far stops increasing for `quiet`
        seconds, then returns it. Used by the "elevated sampling rate" Given
        so the "decreases after recovery" Then has a real peak to compare
        against, instead of an arbitrary one."""
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
        self._node.destroy_node()


@pytest.fixture(scope="module")
def adaptation_log():
    """One persistent subscriber for every G4/G5 scenario in this file - see
    _AdaptationCommandLog. Module-scoped, matching test_bsn_p03.py's
    `ros_node` fixture: rclpy.init()/shutdown() should happen once per
    pytest subprocess, not per scenario.
    """
    log = _AdaptationCommandLog(reconfigure_topic("the oximeter"))
    yield log
    log.shutdown()


def _assert_adaptation_logged(bdd, sensor):
    """Mirrors ROS1's then_adaptation_logged: the system log should have
    persisted a record of the delivered AdaptationCommand, identified by its
    target (see failure_record_identifying_sensor_available's comment on
    which Persist field that is)."""
    sensor_node = SENSORS[sensor]["node"]
    capture_topic_data_with_timeout(bdd, ["/persist"], timeout=5.0)
    persist_data = bdd.topic_data.get("/persist", {})
    matched = [
        target
        for target, record_type in zip(persist_data.get("target", []), persist_data.get("type", []))
        if record_type == "Adaptation" and target == sensor_node
    ]
    assert matched, (
        f"An AdaptationCommand for {sensor} was delivered, but no matching "
        f"record reached the system log's /persist (source='{sensor_node}' "
        f"expected among type='Adaptation' records): {persist_data}"
    )


@given(parsers.parse("the managing system is monitoring the reliability of {sensor}"))
def managing_system_monitoring_reliability(bdd, sensor, adaptation_log):
    # Mirrors ROS1's given_monitoring_reliability: the reliability engine is
    # up and the enactor/parameter adapter are ready to act on what it
    # concludes. Real check (not a hardcoded failure) so the assertion
    # message says exactly what is missing if someone re-runs this after
    # merging the adaptation subsystem.
    _nodes_online(RELIABILITY_ENGINE_NODE, ENACTOR_NODE, PARAM_ADAPTER_NODE)
    bdd.since_index = adaptation_log.since_index()


@given(parsers.parse("{sensor} is operating normally with stable reliability"))
def sensor_operating_normally_stable(bdd, sensor, adaptation_log):
    _nodes_online(RELIABILITY_ENGINE_NODE, ENACTOR_NODE, PARAM_ADAPTER_NODE)
    # Mirrors ROS1's given_stable_reliability: wait for the engine to settle
    # (no new commands for 2s) before marking since_index, so a command left
    # over from a previous scenario's degradation isn't mistaken for a
    # reaction to this one.
    adaptation_log.wait_stable()
    bdd.since_index = adaptation_log.since_index()


@when(parsers.parse("{sensor} repeatedly reports a 'fail' status"))
def sensor_repeatedly_reports_fail_status(bdd, sensor):
    bdd.degradation_start_time = time.time()
    bdd.fail_stream = _FailStatusStream(SENSORS[sensor]["node"])
    bdd.fail_stream.start()


@then(parsers.parse("an adaptation command increasing the sampling rate of {sensor} should be issued"))
def adaptation_command_increasing_rate_issued(bdd, sensor, adaptation_log):
    # Waits specifically for freq > baseline - discards any reduce-command
    # still in flight from a prior scenario's recovery - mirroring ROS1's
    # _wait_for_adaptation_command_above.
    freq = adaptation_log.wait_for_above(BASELINE_FREQUENCY_HZ, bdd.since_index)
    bdd.fail_stream.stop()
    assert freq is not None, (
        f"No AdaptationCommand with freq > {BASELINE_FREQUENCY_HZ}Hz arrived "
        f"on {reconfigure_topic(sensor)} in {ADAPTATION_WAIT_S}s after "
        f"{sensor} repeatedly reported 'fail' - expected: "
        f"{NO_AUTONOMOUS_ADAPTATION_REASON}"
    )
    _assert_adaptation_logged(bdd, sensor)


@then(parsers.parse("an adaptation command for {sensor} should be issued within {time_limit} seconds"))
def adaptation_command_within_time_limit(bdd, sensor, time_limit, adaptation_log):
    timeout_s = float(time_limit)
    freq = adaptation_log.wait_for_any(bdd.since_index, timeout=timeout_s)
    elapsed = time.time() - bdd.degradation_start_time
    bdd.fail_stream.stop()
    assert freq is not None, (
        f"No AdaptationCommand ever arrived on {reconfigure_topic(sensor)} "
        f"within {time_limit}s - expected: {NO_AUTONOMOUS_ADAPTATION_REASON}"
    )
    assert elapsed <= timeout_s, (
        f"AdaptationCommand for {sensor} arrived, but {elapsed:.2f}s after "
        f"degradation started - exceeds the {time_limit}s budget"
    )


@given(parsers.parse("{sensor} is operating at an elevated sampling rate"))
def sensor_operating_at_elevated_rate(bdd, sensor, adaptation_log):
    # Mirrors ROS1's given_elevated_rate: drive real degradation (the same
    # fail-status stream as the "increases" scenario, not a synthetic
    # command) for a fixed window, then read back the peak frequency the
    # engine actually converged on - so the "decreases" Then, below, has a
    # real number to compare against instead of an arbitrary one.
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
        f"precondition was never actually reached. Expected: "
        f"{NO_AUTONOMOUS_ADAPTATION_REASON}"
    )
    bdd.peak_frequency_hz = peak
    bdd.since_index = adaptation_log.since_index()


@when(parsers.parse("the reliability of {sensor} rises above the threshold"))
def reliability_rises_above_threshold(sensor):
    # No new event to trigger: the Given already stopped the fail-status
    # stream, so this is a fixed settle time before checking for a
    # reduce-command - mirrors ROS1's when_reliability_recovers exactly.
    time.sleep(2.0)


@then(parsers.parse("an adaptation command reducing the sampling rate of {sensor} should be issued"))
def adaptation_command_reducing_rate_issued(bdd, sensor, adaptation_log):
    assert hasattr(bdd, "peak_frequency_hz"), (
        "peak_frequency_hz not available - the Given never measured a peak "
        "before recovery"
    )
    freq = adaptation_log.wait_for_below(bdd.peak_frequency_hz, bdd.since_index)
    assert freq is not None, (
        f"No AdaptationCommand with freq < {bdd.peak_frequency_hz:.3f}Hz "
        f"(the measured peak) arrived on {reconfigure_topic(sensor)} in "
        f"{ADAPTATION_WAIT_S}s after {sensor}'s reliability recovered - "
        f"expected: {NO_AUTONOMOUS_ADAPTATION_REASON}"
    )
    _assert_adaptation_logged(bdd, sensor)


@when(parsers.parse("the reliability of {sensor} stays above the threshold"))
def reliability_stays_above_threshold(sensor):
    # Mirrors ROS1's when_managing_system_evaluates: a fixed pause standing
    # in for "the control loop re-evaluates", since nothing degraded
    # reliability in this scenario's Given to begin with.
    time.sleep(3.0)


@then(parsers.parse("no adaptation command for {sensor} should be issued"))
def no_adaptation_command_issued(bdd, sensor, adaptation_log):
    # Careful with this one: asserting *absence* is vacuously satisfied today,
    # because nothing in this port issues adaptation commands at all. The only
    # thing keeping this scenario honest is its Given, which requires
    # /reli_engine and /enactor to actually be online first. Weakening that
    # Given would turn this green - and, since the Gherkin marks it @xfail,
    # into a silent XPASS that does not even fail the run - while proving
    # nothing about the controller staying quiet. See MANAGING_SYSTEM_GAPS.md's
    # note on scenario #9.
    silent = adaptation_log.wait_for_none(bdd.since_index, timeout=5.0)
    assert silent, (
        f"Unexpected AdaptationCommand(s) on {reconfigure_topic(sensor)} "
        f"while reliability was supposedly stable"
    )


# ---------------------------------------------------------------------------
# "A failed adaptation attempt is recorded in the system log" - its
# precondition can't be safely induced even in the ideal world (see Given
# below); everything else here is the real, full check.
# ---------------------------------------------------------------------------

@given(parsers.parse("{sensor} cannot receive adaptation commands"))
def sensor_cannot_receive_adaptation_commands(sensor):
    # No way to induce this today without killing the sensor node (would
    # break the other G4/G5 scenarios sharing this session) or modifying
    # application source. Every sensor already has a real, registered
    # reconfigure_<node> subscriber under the shared launch, so "cannot
    # receive" isn't even true by default - it would have to be actively
    # broken. This is a test-harness limitation, not something a merged
    # adaptation subsystem would change - failing here, honestly, beats
    # faking non-delivery. See MANAGING_SYSTEM_GAPS.md scenario #7.
    pytest.fail(
        f"Cannot simulate '{sensor} cannot receive adaptation commands' "
        f"without killing the node (would break other scenarios sharing "
        f"this session) or modifying application source. "
        f"{NO_FAILURE_DETECTION_REASON}"
    )


@when(parsers.parse("an adaptation command is issued for {sensor}"))
def adaptation_command_issued_for_sensor(sensor):
    sensor_node = SENSORS[sensor]["node"]
    reconfigure = reconfigure_topic(sensor)
    msg_yaml = f"{{source: 'test', target: '{sensor_node}', action: 'freq=5'}}"
    subprocess.run(
        ["ros2", "topic", "pub", "--once", reconfigure, "bsn_interfaces/msg/AdaptationCommand", msg_yaml],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )


@then(parsers.parse("a failure record identifying {sensor} should be available in the system log"))
def failure_record_identifying_sensor_available(bdd, sensor):
    # Matched on `target`, not `source`: logger.py's receive_adapt copies the
    # AdaptationCommand's own source (the enactor, or this test) into
    # Persist.source and its target (the sensor) into Persist.target - so the
    # sensor is identified by `target`. Same field the ROS1 test filtered on
    # (test_managing_system.py's then_adaptation_logged: `x.target ==
    # context["target_sensor"]`).
    sensor_node = SENSORS[sensor]["node"]
    capture_topic_data_with_timeout(bdd, ["/persist"], timeout=5.0)
    persist_data = bdd.topic_data.get("/persist", {})
    failure_records = [
        target
        for target, record_type in zip(persist_data.get("target", []), persist_data.get("type", []))
        if record_type == "Adaptation" and target == sensor_node
    ]
    assert failure_records, (
        f"No failure record identifying {sensor} was found on /persist - "
        f"{NO_FAILURE_DETECTION_REASON}"
    )
