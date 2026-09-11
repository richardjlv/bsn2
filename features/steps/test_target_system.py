"""pytest-bdd scenarios + step definitions for ../target_system.feature.

Same structure as the other test_*.py files. Assumes the system under test
is already running - see conftest.py's module docstring.
"""

import os
import re
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from pytest_bdd import given, parsers, scenarios, then, when

from conftest import _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import (  # noqa: E402
    PARAM_ADAPTER_NODE, PATIENT_NODE, reconfigure_topic, resolve_node, sensor_topic,
)
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

# Gherkin phrase -> patient.py's internal vital-sign key (patient/config/
# patient_params.yaml's vitalSigns list). No "body temperature" entry on
# purpose - the feature's Examples table doesn't cover it either.
VITAL_SIGNS = {
    "blood oxygenation": "oxigenation",
    "heart rate": "heart_rate",
    "systolic blood pressure": "abps",
    "diastolic blood pressure": "abpd",
    "blood glucose": "glucose",
}

# Generates one test per Scenario/Outline row in target_system.feature.
scenarios(os.path.join(FEATURES_DIR, "target_system.feature"))


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/
    services - see utils.parsers.get_rosnode_info_ros2."""
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


def _assert_wired(publisher_node, subscriber_node, topic):
    """A topology claim is only meaningful checked from both ends: a
    publisher with no subscriber (or vice versa) is a dangling channel, not
    a wired one."""
    publisher_info = _node_info(publisher_node)
    assert any(pub["topic"] == topic for pub in publisher_info["publishers"]), (
        f"{publisher_node} does not publish {topic}"
    )
    subscriber_info = _node_info(subscriber_node)
    assert any(sub["topic"] == topic for sub in subscriber_info["subscribers"]), (
        f"{subscriber_node} does not subscribe to {topic}"
    )


@given(parsers.parse("{component} is running"))
def component_is_running(component):
    _nodes_online(resolve_node(component))


@then(parsers.parse('{sensor} should publish "vital sign data" to the central hub'))
def sensor_publishes_vital_sign_data(sensor):
    _assert_wired(resolve_node(sensor), resolve_node("the central hub"), sensor_topic(sensor))


@then(parsers.parse('the parameter adapter should publish "reconfiguration command" to {target}'))
def param_adapter_publishes_reconfiguration_command(target):
    _assert_wired(PARAM_ADAPTER_NODE, resolve_node(target), reconfigure_topic(target))


@given("the patient simulator is generating vital signs for the monitored patient")
def patient_simulator_generating_vital_signs():
    _nodes_online(PATIENT_NODE)


@when(parsers.parse("the current value of {vital_sign} is requested"))
def current_value_requested(bdd, vital_sign):
    vital_key = VITAL_SIGNS[vital_sign]
    result = subprocess.run(
        [
            "ros2", "service", "call", "/get_sensor_reading",
            "bsn_interfaces/srv/PatientData", f"{{vital_sign: '{vital_key}'}}",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10, text=True,
    )
    assert result.returncode == 0, f"get_sensor_reading call failed: {result.stderr}"
    match = re.search(r"datapoint=([-\d.eE]+)", result.stdout)
    assert match, f"Could not parse a datapoint from the service response: {result.stdout}"
    bdd.reported_value = float(match.group(1))


def _valid_range(vital_key):
    """The union of every non-placeholder risk range configured for this
    vital sign in patient/config/patient_params.yaml - patient.py's
    _calculate_datapoint only ever draws from one of these bands, so their
    union is exactly the simulator's valid output range for it.
    """
    config_path = os.path.join(
        get_package_share_directory("patient"), "config", "patient_params.yaml"
    )
    with open(config_path) as f:
        params = yaml.safe_load(f)["patient_node"]["ros__parameters"]

    bounds = []
    for label in ("HighRisk0", "MidRisk0", "LowRisk", "MidRisk1", "HighRisk1"):
        range_min, range_max = params[f"{vital_key}_{label}"]
        if range_min != -1.0 or range_max != -1.0:  # -1,-1 marks an unused band
            bounds += [range_min, range_max]
    assert bounds, f"No valid risk range configured for '{vital_key}' in patient_params.yaml"
    return min(bounds), max(bounds)


@then(parsers.parse("a value within the valid range of {vital_sign} should be returned"))
def value_within_valid_range(bdd, vital_sign):
    range_min, range_max = _valid_range(VITAL_SIGNS[vital_sign])
    # 0.0 is patient.py's un-generated init value for every vital
    # (Patient.vital_datapoints), not a real draw - random.uniform() landing
    # exactly on it is statistically ~impossible, and it happens to fall
    # inside the valid range for 4 of 5 vitals here. Without this, a
    # simulator that never ran gen_data() would still pass this check.
    assert bdd.reported_value != 0.0, (
        f"{vital_sign} reading is exactly 0.0 - looks like the simulator's "
        f"init value, not a generated reading"
    )
    assert range_min <= bdd.reported_value <= range_max, (
        f"{vital_sign} reading {bdd.reported_value} is outside the configured "
        f"valid range [{range_min}, {range_max}]"
    )
