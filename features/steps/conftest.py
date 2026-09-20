"""Session setup for the pytest-bdd scenarios here: verifies the system under test is already up (it never
launches it; for a direct run use `ros2 launch central_hub emergency_detection_launch.py`). Each
../test/test_*_bdd.py launches its own stack (see ../test/harness.py); run one file per pytest session."""

import os
import re
import subprocess
import sys
import time
from contextlib import contextmanager
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
# 60s, not 30s: bsn_system polls with one `ros2 lifecycle get` subprocess per node, and running all
# test_*_bdd.py back-to-back can outrun 30s under contention (same reasoning as test/harness.py).
STARTUP_TIMEOUT_S = 60.0


def _nodes_online(*node_names):
    result = subprocess.run(
        ["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    node_list = result.stdout.decode("utf-8").splitlines()
    for name in node_names:
        assert f"/{name}" in node_list, f"Node /{name} is not online"


def _kill_central_hub():
    """Shared by every "hub is down" Given step: killing (not lifecycle-deactivating) is the only reliable
    way to keep it down, since LifecycleManager's ~5s auto-recovery timer can silently reactivate a
    deactivated hub."""
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


# Patient simulator node name, and the Markov state index of a vital sign's HighRisk1 range
# (patient.py labels states 0 HighRisk0 ... 4 HighRisk1; 4 is "out of range").
PATIENT_NODE = "patient_node"
PATIENT_HIGH_RISK_STATE = 4
PATIENT_NORMAL_STATE = 2
_PARAM_ARRAY_RE = re.compile(r"\[(.*)\]")


def _patient_transition_period(vital_sign):
    """Seconds between the patient's transition attempts for `vital_sign`:
    patient.py derives them from `<vital_sign>_Change` (gen_data only re-rolls
    once 1/Change has accumulated past the initial Offset)."""
    result = subprocess.run(
        ["ros2", "param", "get", f"/{PATIENT_NODE}", f"{vital_sign}_Change"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10,
    )
    match = re.search(r"([0-9]*\.?[0-9]+)\s*$", result.stdout.decode().strip())
    assert match, (
        f"Could not read {vital_sign}_Change from /{PATIENT_NODE}: "
        f"{result.stdout.decode()!r} {result.stderr.decode()!r}"
    )
    return 1.0 / float(match.group(1))


def _get_patient_transition_row(vital_sign, state):
    """Current `<vital_sign>_State<state>` row, as a list of floats."""
    result = subprocess.run(
        ["ros2", "param", "get", f"/{PATIENT_NODE}", f"{vital_sign}_State{state}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10,
    )
    match = _PARAM_ARRAY_RE.search(result.stdout.decode())
    assert match, (
        f"Could not read {vital_sign}_State{state} from /{PATIENT_NODE}: "
        f"{result.stdout.decode()!r} {result.stderr.decode()!r}"
    )
    return [float(v) for v in match.group(1).split(",")]


def _set_patient_transition_row(vital_sign, state, probabilities):
    """Overwrite one row of the patient simulator's Markov transition matrix.
    patient.py re-reads `<vital_sign>_State<n>` on every transition attempt, so it takes effect
    on the next attempt (every 1/`<vital_sign>_Change` s after the initial Offset)."""
    value = "[" + ", ".join(str(float(p)) for p in probabilities) + "]"
    result = subprocess.run(
        ["ros2", "param", "set", f"/{PATIENT_NODE}", f"{vital_sign}_State{state}", value],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10,
    )
    assert b"Set parameter successful" in result.stdout, (
        f"Failed to set {vital_sign}_State{state} on /{PATIENT_NODE} to {value}: "
        f"{result.stdout.decode()!r} {result.stderr.decode()!r}"
    )


@contextmanager
def patient_vital_pinned_to_high_risk(vital_sign):
    """Pin the patient simulator in `vital_sign`'s HighRisk1 range for the block, then restore it.
    Turns "reading outside its normal range" into a precondition (left alone it is ~0.4%/s);
    only the patient's state is forced - sensor, risk score and hub fusion stay real."""
    saved = {
        state: _get_patient_transition_row(vital_sign, state)
        for state in range(5)
    }
    # Rows the patient itself ignores (all zeros -> None in
    # _gen_sensor_state_matrix) stay untouched throughout: writing one would
    # enable a state whose risk range is [-1, -1] for this vital sign.
    live_states = [state for state, row in saved.items() if any(row)]

    def pin_to(target_state):
        row = [0.0] * 5
        row[target_state] = 1.0
        for state in live_states:
            _set_patient_transition_row(vital_sign, state, row)

    try:
        pin_to(PATIENT_HIGH_RISK_STATE)
        yield
    finally:
        # Restoring the matrix alone would leave the simulator in the high-risk state (0.05/0.35 chance
        # of leaving per attempt); walk it back to normal first (one attempt, p=1), then restore.
        pin_to(PATIENT_NORMAL_STATE)
        time.sleep(_patient_transition_period(vital_sign) + 1.5)
        for state in live_states:
            _set_patient_transition_row(vital_sign, state, saved[state])


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


# One wall-clock window for every topic this step captures: ~12 sensor readings (1 Hz) and ~72 hub
# samples (6 Hz), enough overlap for a published value to appear in the hub's stream.
SENSOR_READING_CAPTURE_S = 12.0


@when(parsers.parse("{sensor} reports a new vital sign reading"))
def sensor_reports_reading(bdd, sensor):
    # Shared by every *.feature with this phrasing (also captures /persist; reused by sad-path scenarios).
    # Wall-clock window, not a per-topic line count: sensor (1 Hz) and hub (6 Hz) rates differ too much.
    # /persist is filtered to this sensor by ROS2 *node* name (what logger.py copies into Persist.source).
    capture_topic_data_with_timeout(
        bdd,
        [_sensor_topic(sensor), TARGET_SYSTEM_TOPIC, PERSIST_TOPIC],
        timeout=SENSOR_READING_CAPTURE_S,
        filters={PERSIST_TOPIC: f"m.source == '{SENSORS[sensor]['node']}'"},
    )


def pytest_configure(config):
    # Registers check_bsn.feature's and health_status.feature's tags as pytest markers (silences
    # "Unknown pytest.mark.X"); done here, not in pytest.ini, so it applies from any invocation directory.
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
    """Verifies the system under test is already running and active; launching and tearing it down
    is the harness's job (see this module's docstring)."""
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
