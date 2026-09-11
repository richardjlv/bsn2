"""Session setup for the pytest-bdd scenarios in this directory - what
behave's before_all/after_all did before the port, except this fixture does
NOT launch or tear down the system under test itself; it only verifies it is
already up before any scenario runs. Launch it yourself first for a direct
`python3 -m pytest steps/...` run:

    ros2 launch central_hub emergency_detection_launch.py

This package's colcon-test path does this for you: each
../test/test_*_bdd.py brings up its own launch file (via launch_testing)
before running this directory's tests as a `pytest` subprocess - see
../test/harness.py. Note that pytest.ini's `testpaths = test` keeps
colcon's own collection out of this directory, so these files run only
through those harnesses, never bare.

One stack, one session: running more than one test_*.py file in the same
pytest invocation (e.g. `pytest steps/`) makes them share
whatever single system is already running. A scenario that permanently kills
a node (see test_health_status.py's "the central hub is unable to process
incoming readings") leaves it dead for whatever runs afterward in that same
session - captures no longer hang forever waiting on it (utils/parsers.py's
capture_topic_data has a timeout safety net), but they do run needlessly
slow and their assertions will see the wrong (empty) data. Each
launch_testing harness avoids this by running one file per `pytest`
subprocess (a fresh launch each time); running files together here manually
does not.
"""

import os
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
from pytest_bdd import given, parsers, when

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import (  # noqa: E402
    CENTRAL_HUB_NODE, LOGGER_NODE, PERSIST_TOPIC, SENSORS, TARGET_SYSTEM_TOPIC, sensor_topic as _sensor_topic,
)
from utils.parsers import capture_topic_data_with_timeout  # noqa: E402

# Every lifecycle node the full launch file brings up.
LIFECYCLE_NODES = (
    "thermometer_node",
    "oximeter_node",
    "ecg_node",
    "abps_node",
    "abpd_node",
    "glucosemeter_node",
    "central_hub_node",
)
STARTUP_TIMEOUT_S = 30.0


def _nodes_online(*node_names):
    result = subprocess.run(
        ["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    node_list = result.stdout.decode("utf-8").splitlines()
    for name in node_names:
        assert f"/{name}" in node_list, f"Node /{name} is not online"


def _kill_central_hub():
    """Shared by every "hub is down" Given step (check_bsn.feature's "the
    central hub is unavailable", health_status.feature's "...unable to
    process incoming readings"): killing (not lifecycle-deactivating) is the
    only reliable way to keep it down. LifecycleManager's ~5s auto-recovery
    timer (shared_components/lifecycle_manager.py) isn't reset by a manual
    deactivate and isn't exposed as a settable parameter, so a deactivated
    hub can silently reactivate mid-scenario.
    """
    # SIGKILL, not SIGTERM: emergency_detection doesn't die on SIGTERM at
    # all - rclpy only wires shutdown to SIGINT.
    subprocess.run(
        ["pkill", "-9", "-f", "emergency_detection"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )

    # `ros2 node list` won't do here: DDS discovery doesn't notice a killed
    # participant leaving and keeps reporting it online well after the
    # process is actually gone.
    deadline = time.time() + 5
    while time.time() < deadline:
        result = subprocess.run(
            ["pgrep", "-f", "emergency_detection"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode != 0:
            return
        time.sleep(0.2)
    pytest.fail(
        f"Failed to kill {CENTRAL_HUB_NODE} (emergency_detection); "
        f"this scenario's precondition never took effect"
    )


@pytest.fixture
def bdd():
    """Per-scenario state, standing in for behave's `context`."""
    return SimpleNamespace(topic_data={})


@given(parsers.parse("the patient is being monitored by {sensor}"))
def patient_monitored_by_sensor(sensor):
    # Shared across check_bsn.feature and health_status.feature - see
    # sensor_reports_reading below for why this lives here instead of
    # being redefined per test_*.py file.
    assert sensor in SENSORS, f"Unknown sensor '{sensor}'; see SENSORS map"
    _nodes_online(SENSORS[sensor]["node"], CENTRAL_HUB_NODE)


# One wall-clock window for every topic this step captures. ~12 readings at
# the sensors' 1 Hz baseline (sensor/config/*.yaml), and ~72 central-hub
# samples at its 6 Hz - plenty of overlap for a value published by the sensor
# to also appear in the hub's stream.
SENSOR_READING_CAPTURE_S = 12.0


@when(parsers.parse("{sensor} reports a new vital sign reading"))
def sensor_reports_reading(bdd, sensor):
    # Shared across every *.feature file that uses this exact phrasing -
    # including /persist so any scenario needing persistence (see
    # test_data_persistance.py) gets it for free. Also reused by sad-path
    # scenarios where the hub never publishes back.
    #
    # Wall-clock window, not a per-topic line count: the sensor (~1 Hz) and
    # the hub (6 Hz) publish at very different rates, so a line count closes
    # each topic's window at a different moment - the hub's could close
    # before the sensor even produced the value it would echo. This is what
    # broke check_bsn.feature's first Examples row before this fix.
    #
    # /persist is filtered to this sensor's own records (it's fed by every
    # sensor plus every node's Status/Event chatter, so unfiltered it drowns
    # this sensor's rows in noise). The filter matches the ROS2 *node* name -
    # what logger.py copies into Persist.source - not the config key, which
    # never appears there.
    capture_topic_data_with_timeout(
        bdd,
        [_sensor_topic(sensor), TARGET_SYSTEM_TOPIC, PERSIST_TOPIC],
        timeout=SENSOR_READING_CAPTURE_S,
        filters={PERSIST_TOPIC: f"m.source == '{SENSORS[sensor]['node']}'"},
    )


def pytest_configure(config):
    # Registers check_bsn.feature's and health_status.feature's tags as
    # pytest markers to silence "Unknown pytest.mark.X". Done here rather
    # than pytest.ini/setup.cfg so it applies regardless of which directory
    # pytest is invoked from.
    for marker in (
        "behavior", "bsn-p08", "bsn-p09", "bsn-p10", "bsn-p03", "sad-path", "persistence", "timing",
        "topology", "patient-simulator", "contract", "g4-g5", "logging",
        # "regression": confirmed against the ROS1 suite to have actually
        # worked there. "gap": ROS1 never really verified this either - see
        # e.g. INJECTOR_GAPS.md / DATA_PERSISTANCE_GAPS.md.
        "regression", "gap",
    ):
        config.addinivalue_line("markers", f"{marker}: Gherkin tag from a features/*.feature file")


@pytest.fixture(scope="session", autouse=True)
def bsn_system():
    """Verifies the system under test is already running and active -
    launching and tearing it down is someone else's job now (see this
    module's docstring), so there is nothing to start or kill here.
    """
    # Poll for 'active' instead of a blind sleep - usually faster, and gives
    # a precise, actionable failure if nothing was launched at all.
    deadline = time.time() + STARTUP_TIMEOUT_S
    pending = set(LIFECYCLE_NODES)
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
                # Slow service call, not a dead node - retry.
                continue
            if result.stdout.decode().strip().startswith("active"):
                pending.discard(name)
        if pending:
            time.sleep(1)
    if pending:
        pytest.exit(
            f"Nodes never reached 'active': {pending}. Is `ros2 launch "
            f"central_hub emergency_detection_launch.py` running?",
            returncode=1,
        )

    yield
