"""pytest-bdd scenarios + step definitions for ../BSN-P03.feature.

../test/test_bsn_p03_bdd.py provides the launch_testing harness
for this file (`colcon test`); see its module docstring. For a direct
`python3 -m pytest steps/test_bsn_p03.py` run, launch it
yourself first:

    ros2 launch central_hub emergency_detection_adaptation_launch.py

(not emergency_detection_launch.py - see that launch file's docstring for
why this feature needs the adaptation variant). The `bsn_system` fixture
below only verifies that stack is up; see it for why this file needs its own
instead of reusing conftest.py's.

Both scenarios need to observe latency well under a second, so unlike every
other step in this package they talk to the ROS graph directly through rclpy
instead of shelling out to `ros2 topic echo`/`ros2 topic pub`: a fresh CLI
process has to start up and complete DDS discovery before it can send or see
anything, and that alone can take longer than the 250 ms budget being
measured. Talking through one already-discovered, long-lived Node sidesteps
that entirely - see the module-scoped `ros_node` fixture and
`_wait_until_matched` below.
"""

import os
import random
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from pytest_bdd import scenarios, given, when, then
from rclpy.node import Node

from conftest import CENTRAL_HUB_NODE, SENSORS, TARGET_SYSTEM_TOPIC, _nodes_online, _sensor_topic

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from bsn_interfaces.msg import SensorData, TargetSystemData  # noqa: E402

# Generates one test per Scenario in BSN-P03.feature.
scenarios(os.path.join(FEATURES_DIR, "BSN-P03.feature"))

THERMOMETER = "the thermometer"
THERMOMETER_TOPIC = _sensor_topic(THERMOMETER)
THERMOMETER_NODE = SENSORS[THERMOMETER]["node"]

# sensor/config/thermometer.yaml's HighRisk1 band is [41.0, 50.0]; 45.0 is a
# real high fever, comfortably inside it, and would land the sensor's own
# risk_manager in the "high" (66-100) percentage band (see
# sensor/utils/risk_evaluator.py) if the real sensor evaluated it.
HIGH_RISK_TEMPERATURE = 45.0
HIGH_RISK_VALUE = 90.0

DISCOVERY_TIMEOUT_S = 5.0
# >= a few central-hub publish cycles: at 6 Hz, detect() (and therefore
# /target_system_data) runs every ~167 ms, so this is ~6 cycles.
REACTION_WINDOW_S = 1.0
# See thermometer_reports_high_risk_reading for why an injection can be
# swallowed by the real sensor and has to be retried, and why the backoff is
# randomized rather than fixed.
INJECTION_ATTEMPTS = 5
INJECTION_BACKOFF_RANGE_S = (0.05, 0.35)
EMERGENCY_BUDGET_S = 0.25
# Set by both launch files as `parameters=[{"frequency": 6.0}]`, restoring the
# legacy hub cadence (ROS1's g4t1.launch used the same 6 Hz) - see
# ../BSN-P03.feature's first scenario for the full history. Not the node's own
# default, which is still 2.0 (central_hub/components/config_manager.py:9).
HUB_FREQUENCY_HZ = 6.0

# "Maximum sampling rate" stand-in - same value the ROS1 test used, inside
# the legacy controller's [0.1, 40] Hz bounds. The concrete "maximum" is
# still an open question (migration review's D-23).
MAX_SAMPLING_FREQUENCY_HZ = 20
BASELINE_FREQUENCY_HZ = 1.0  # sensor/config/thermometer.yaml
OVERLOAD_OBSERVATION_WINDOW_S = 2.0
OVERLOAD_MIN_SAMPLES = 20  # well above baseline, well below MAX * window
# thermometer.yaml's LowRisk band (36.0-37.99): must be genuinely low-risk,
# or the "then it goes high-risk" transition wouldn't be one.
LOW_RISK_TEMPERATURE = 37.0
LOW_RISK_VALUE = 10.0

# central_hub/launch/emergency_detection_adaptation_launch.py only starts
# these two as LifecycleNodes; patient_node and param_adapter are plain
# rclpy.node.Node and never leave the (non-lifecycle) "unconfigured" state.
LIFECYCLE_NODES = (THERMOMETER_NODE, CENTRAL_HUB_NODE)
PLAIN_NODES = ("patient_node", "param_adapter")
STARTUP_TIMEOUT_S = 30.0


def _wait_for_active(node_names, timeout_s=STARTUP_TIMEOUT_S):
    # Same polling conftest.py's bsn_system does; duplicated (rather than
    # imported) because it's the one piece that can't be shared as-is - the
    # set of node names differs per launch file.
    deadline = time.time() + timeout_s
    pending = set(node_names)
    while pending and time.time() < deadline:
        for name in list(pending):
            try:
                result = subprocess.run(
                    ["ros2", "lifecycle", "get", name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5,
                )
            except subprocess.TimeoutExpired:
                continue
            if result.stdout.decode().strip().startswith("active"):
                pending.discard(name)
        if pending:
            time.sleep(1)
    return pending


@pytest.fixture(scope="module", autouse=True)
def bsn_system():
    """Overrides conftest.py's session-scoped `bsn_system` by name: this file
    needs the adaptation launch variant (param_adapter + enable_adaptation),
    so it checks for a different set of nodes. Like conftest.py's version,
    this does not launch or tear anything down - see the module docstring for
    how to bring the right stack up first. Never combine this file with
    test_check_bsn.py/test_health_status.py in the same `pytest` invocation:
    only one of the two launch files can be running against
    central_hub_node/thermometer_node at a time.
    """
    pending = _wait_for_active(LIFECYCLE_NODES)
    if pending:
        pytest.exit(
            f"Nodes never reached 'active': {pending}. Is `ros2 launch "
            f"central_hub emergency_detection_adaptation_launch.py` running?",
            returncode=1,
        )

    _nodes_online(*PLAIN_NODES)
    yield


@pytest.fixture(scope="module")
def ros_node():
    if not rclpy.ok():
        rclpy.init()
    node = Node("bdd_bsn_p03")
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _wait_until_matched(predicate, node, timeout_s=DISCOVERY_TIMEOUT_S):
    """Spin `node` until `predicate()` is true or `timeout_s` elapses.

    Used to let DDS discovery (a publisher finding its subscribers, or vice
    versa) settle *before* the timed part of a step, instead of during it -
    see the module docstring.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


class _SensorLoad:
    """Background publisher standing in for the thermometer at its maximum
    sampling rate, and the source of this scenario's high-risk reading.

    Why the test produces the load itself instead of commanding the real
    sensor to: doing that via AdaptationCommand doesn't change the sensor's
    rate in this port (a real regression, covered separately and xfail'd by
    managing_system.feature's "Sampling rate decreases after sensor
    reliability recovers" - see ../MANAGING_SYSTEM_GAPS.md #6). This scenario
    only needs the load as a precondition for the hub-side property it
    actually asserts, so it produces the stream directly - the hub doesn't
    care who publishes on sensor_data/<key>. Honest limit: this loads the
    hub, not the sensor's own process; see ../BSN-P03.feature for the launch
    change that would load the real sensor instead.

    The stream switches from low-risk to high-risk in place, rather than a
    one-off injection, because a lone injection is overwritten by the stream
    itself within ~50ms - long before the hub's ~167ms cycle could see it.
    """

    def __init__(self, node, topic):
        self._node = node
        self._publisher = node.create_publisher(SensorData, topic, 10)
        self._stop = threading.Event()
        self._high_risk = False
        self._thread = None
        self.first_high_risk_time = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        period = 1.0 / MAX_SAMPLING_FREQUENCY_HZ
        while not self._stop.is_set():
            msg = SensorData()
            msg.sensor_type = "thermometer"
            msg.battery_level = 100.0
            if self._high_risk:
                msg.sensor_datapoint = HIGH_RISK_TEMPERATURE
                msg.risk = HIGH_RISK_VALUE
                msg.risk_level = "high"
                if self.first_high_risk_time is None:
                    self.first_high_risk_time = time.time()
            else:
                msg.sensor_datapoint = LOW_RISK_TEMPERATURE
                msg.risk = LOW_RISK_VALUE
                msg.risk_level = "low"
            self._publisher.publish(msg)
            self._stop.wait(period)

    def switch_to_high_risk(self):
        self._high_risk = True

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._node.destroy_publisher(self._publisher)


@given("the thermometer is reporting low-risk readings at its maximum sampling rate")
def thermometer_at_max_sampling_rate(bdd, ros_node):
    _nodes_online(THERMOMETER_NODE, CENTRAL_HUB_NODE)

    load = _SensorLoad(ros_node, THERMOMETER_TOPIC)
    bdd.sensor_load = load  # read by the When step; stopped there too
    load.start()

    # Observed, not assumed: count what actually reaches a subscriber on the
    # sensor's real topic. This confirms the stream is live and being
    # delivered through DDS (the hub is a subscriber of the same topic) at a
    # rate the sensor's own 1 Hz baseline could never account for.
    sample_times = []
    subscriber = ros_node.create_subscription(
        SensorData, THERMOMETER_TOPIC, lambda _msg: sample_times.append(time.time()), 50
    )
    try:
        deadline = time.time() + OVERLOAD_OBSERVATION_WINDOW_S
        while time.time() < deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.02)
    finally:
        ros_node.destroy_subscription(subscriber)

    assert len(sample_times) >= OVERLOAD_MIN_SAMPLES, (
        f"Expected {THERMOMETER_TOPIC} to be carrying a saturated stream "
        f"(>= {OVERLOAD_MIN_SAMPLES} readings in "
        f"{OVERLOAD_OBSERVATION_WINDOW_S}s at {MAX_SAMPLING_FREQUENCY_HZ} Hz) "
        f"but only {len(sample_times)} arrived - the load publisher is not "
        f"reaching subscribers, so this scenario would measure the hub under "
        f"no load at all. Check that {THERMOMETER_NODE} and "
        f"{CENTRAL_HUB_NODE} are up and that nothing is throttling the topic."
    )


@when("the thermometer reports a vital sign reading in the high-risk range")
def thermometer_reports_high_risk_reading(bdd, ros_node):
    # Shared by both scenarios. The sad path set up a saturated stream in its
    # Given (see _SensorLoad); there the high-risk reading is that stream
    # turning high-risk, not a one-off injection into an idle topic.
    if getattr(bdd, "sensor_load", None) is not None:
        _high_risk_from_saturated_stream(bdd, ros_node)
    else:
        _high_risk_from_single_injection(bdd, ros_node)


def _high_risk_from_saturated_stream(bdd, ros_node):
    load = bdd.sensor_load
    received = []
    subscriber = ros_node.create_subscription(
        TargetSystemData, TARGET_SYSTEM_TOPIC, lambda msg: received.append(msg), 10
    )
    try:
        matched = _wait_until_matched(lambda: subscriber.get_publisher_count() > 0, ros_node)
        assert matched, (
            f"{CENTRAL_HUB_NODE} never matched as a publisher of "
            f"{TARGET_SYSTEM_TOPIC}; is it running and active?"
        )

        # Drain whatever the hub already queued while the load was low-risk,
        # so nothing published *before* the switch can be mistaken for a
        # reaction to it (that would read as a negative latency).
        drain_deadline = time.time() + 0.3
        while time.time() < drain_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.01)
        received.clear()

        load.switch_to_high_risk()
        deadline = time.time() + REACTION_WINDOW_S
        while time.time() < deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.01)
    finally:
        ros_node.destroy_subscription(subscriber)
        load.stop()
        bdd.sensor_load = None

    assert load.first_high_risk_time is not None, (
        "The load publisher never sent a high-risk reading after being "
        "switched - its thread died; see _SensorLoad."
    )
    bdd.injection_time = load.first_high_risk_time
    bdd.injection_attempts = 1
    bdd.injected_datapoint = HIGH_RISK_TEMPERATURE
    # Only samples stamped at or after the switch: an older sample still in
    # flight can carry a high-risk value from before the drain.
    bdd.target_messages = [
        {
            "trm_data": msg.trm_data,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        for msg in received
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 >= load.first_high_risk_time
    ]


def _high_risk_from_single_injection(bdd, ros_node):
    received = []
    subscriber = ros_node.create_subscription(
        TargetSystemData, TARGET_SYSTEM_TOPIC, lambda msg: received.append(msg), 10
    )
    publisher = ros_node.create_publisher(SensorData, THERMOMETER_TOPIC, 10)
    try:
        matched = _wait_until_matched(
            lambda: publisher.get_subscription_count() > 0 and subscriber.get_publisher_count() > 0,
            ros_node,
        )
        assert matched, (
            f"central_hub_node never matched as a subscriber of "
            f"{THERMOMETER_TOPIC} and a publisher of {TARGET_SYSTEM_TOPIC}; "
            f"is it running and active?"
        )

        # The patient simulator can take several minutes to drift into a
        # high-risk temperature on its own (patient_params.yaml's transition
        # matrix), so this publishes the high-risk reading directly onto the
        # thermometer's own topic instead - the hub doesn't care who
        # publishes there, so this exercises the same reaction latency.
        #
        # Retried with randomized backoff: the real thermometer_node is also
        # publishing on this topic at ~1 Hz, and the hub keeps only the
        # latest value per sensor, so a genuine reading can overwrite this
        # injection before the hub's next cycle reads it (harness artifact,
        # not a system defect - ROS1's hub was equally last-value-wins).
        # Measured on a live stack: ~20% of single injections are swallowed
        # this way. The backoff must be randomized, not fixed - a fixed
        # retry period near the sensor's own ~1 Hz phase-locks against it
        # (measured: 5 consecutive misses with a fixed 1s retry, vs. 15/15
        # first-attempt hits with jitter).
        send_time = None
        for attempt in range(INJECTION_ATTEMPTS):
            received.clear()

            msg = SensorData()
            msg.sensor_type = "thermometer"
            msg.sensor_datapoint = HIGH_RISK_TEMPERATURE
            msg.risk = HIGH_RISK_VALUE
            msg.risk_level = "high"
            msg.battery_level = 100.0

            send_time = time.time()
            publisher.publish(msg)

            deadline = time.time() + REACTION_WINDOW_S
            while time.time() < deadline:
                rclpy.spin_once(ros_node, timeout_sec=0.02)

            if any(m.trm_data == HIGH_RISK_TEMPERATURE for m in received):
                break
            time.sleep(random.uniform(*INJECTION_BACKOFF_RANGE_S))
    finally:
        ros_node.destroy_publisher(publisher)
        ros_node.destroy_subscription(subscriber)

    # Not bdd.topic_data (the dict-of-columns shape every capture_topic_data-
    # based step uses): this needs each /target_system_data sample's value
    # paired with its own timestamp, not parallel per-field lists. Only the
    # last (successful, if any) attempt's window is kept, paired with that
    # attempt's own send_time - mixing windows would measure latency against
    # the wrong injection.
    bdd.injection_time = send_time
    bdd.injection_attempts = attempt + 1
    bdd.injected_datapoint = HIGH_RISK_TEMPERATURE
    bdd.target_messages = [
        {
            "trm_data": msg.trm_data,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        for msg in received
    ]


@then("the central hub should report an emergency within 250 ms of that reading")
def hub_reports_emergency_in_time(bdd):
    matches = [m for m in bdd.target_messages if m["trm_data"] == bdd.injected_datapoint]
    assert matches, (
        f"The central hub never echoed the injected high-risk reading "
        f"(trm_data={bdd.injected_datapoint}) on {TARGET_SYSTEM_TOPIC} within "
        f"{REACTION_WINDOW_S}s, across all {bdd.injection_attempts} injection "
        f"attempts; observed trm_data values on the last attempt were: "
        f"{[m['trm_data'] for m in bdd.target_messages]}. Every attempt being "
        f"swallowed (rather than the usual ~20% one-off) points at the hub not "
        f"consuming {THERMOMETER_TOPIC} at all, not at the real sensor "
        f"overwriting the injection - check that central_hub_node is active "
        f"and subscribed."
    )

    latency = min(m["stamp"] for m in matches) - bdd.injection_time
    assert -0.01 <= latency <= EMERGENCY_BUDGET_S, (
        f"Central hub took {latency * 1000:.1f} ms to reflect the high-risk "
        f"thermometer reading on {TARGET_SYSTEM_TOPIC} (budget: "
        f"{EMERGENCY_BUDGET_S * 1000:.0f} ms). central_hub_node only runs "
        f"detect() (which publishes {TARGET_SYSTEM_TOPIC}) at "
        f"{HUB_FREQUENCY_HZ} Hz, i.e. every "
        f"{1000.0 / HUB_FREQUENCY_HZ:.0f} ms, so this budget can only be met "
        f"when the reading happens to land in roughly the back half of a "
        f"poll cycle."
    )
