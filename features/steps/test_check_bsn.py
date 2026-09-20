"""pytest-bdd scenarios + step definitions for ../check_bsn.feature (ported from ../steps/check_bsn_steps.py).
Assumes the system under test is already running, launched manually or by ../test/test_check_bsn_bdd.py's
launch_testing harness."""

import os
import sys

from pytest_bdd import given, parsers, scenarios, then, when

from conftest import (  # noqa: E402
    CENTRAL_HUB_NODE, SENSORS, TARGET_SYSTEM_TOPIC, _kill_central_hub, _nodes_online,
    _sensor_topic, patient_vital_pinned_to_high_risk,
)

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.asserts import any_high_risk_percentage, count_matching_elements, matching_elements  # noqa: E402
from utils.parsers import capture_topic_data, capture_topic_data_with_timeout  # noqa: E402

# Generates one test per Scenario/Outline row in check_bsn.feature.
scenarios(os.path.join(FEATURES_DIR, "check_bsn.feature"))

# patient_params.yaml's name for blood oxygenation (sic - it is spelled that
# way throughout both stacks, including ROS1's own patient.launch).
OXIMETER_VITAL_SIGN = "oxigenation"

# Longer than the shared reading step's window: the pinned state only re-rolls every 5s
# (oxigenation_Change 0.2), then the oximeter (1 Hz) collects and the hub echoes it.
HIGH_RISK_CAPTURE_S = 25.0

@given("the central hub is unavailable")
def central_hub_unavailable():
    # Killing is the only reliable option (see conftest.py's _kill_central_hub); the hub is not restarted,
    # so this only holds if no other file's scenarios run afterward in the same session.
    _kill_central_hub()

@then(parsers.parse(
    "the central hub should receive that reading with the value reported by {sensor}"
))
def hub_receives_reading(bdd, sensor):
    info = SENSORS[sensor]
    hub_field = f"{info['hub_field']}_data"
    sensor_data = bdd.topic_data[_sensor_topic(sensor)]
    target_data = bdd.topic_data[TARGET_SYSTEM_TOPIC]
    assert count_matching_elements(sensor_data["sensor_datapoint"], target_data[hub_field]), (
        f"Mismatch: no matching values between {sensor} readings "
        f"({sensor_data['sensor_datapoint']}) and the central hub's "
        f"{hub_field} ({target_data[hub_field]})"
    )


@when("the oximeter reports a blood oxygenation reading outside its normal range")
def oximeter_reports_out_of_range_reading(bdd):
    # "Outside its normal range" is forced (conftest.patient_vital_pinned_to_high_risk), not awaited.
    # The capture stays open longer because the patient re-rolls its state only every 5s.
    with patient_vital_pinned_to_high_risk(OXIMETER_VITAL_SIGN):
        capture_topic_data_with_timeout(
            bdd,
            [_sensor_topic("the oximeter"), TARGET_SYSTEM_TOPIC],
            timeout=HIGH_RISK_CAPTURE_S,
        )


@then("the central hub should classify the patient risk for blood oxygenation as high")
def hub_classifies_high_risk(bdd):
    info = SENSORS["the oximeter"]
    hub_risk_field = f"{info['hub_field']}_risk"
    sensor_data = bdd.topic_data[_sensor_topic("the oximeter")]
    target_data = bdd.topic_data[TARGET_SYSTEM_TOPIC]
    matched_risks = matching_elements(sensor_data["risk"], target_data[hub_risk_field])
    assert matched_risks, (
        f"Mismatch: no matching risk values between the oximeter's own risk "
        f"score ({sensor_data['risk']}) and the central hub's {hub_risk_field} "
        f"({target_data[hub_risk_field]})"
    )
    # Asserts the configured band (66-100, oximeter.yaml highrisk_percent), not ROS1's weaker `> 10`,
    # which only passes there because ROS1 never forces a high-risk reading; with the patient pinned
    # into HighRisk1, the real band is deterministic and is what the scenario says.
    assert any_high_risk_percentage(matched_risks), (
        f"The oximeter never reported (and the central hub never echoed) a "
        f"high-risk (66-100) blood oxygenation reading, even though the "
        f"patient simulator was pinned to its HighRisk1 oxigenation state "
        f"for the whole capture; matched risk values were: {matched_risks}"
    )


@then("no patient risk level should be reported for that reading")
def no_patient_risk_reported(bdd):
    target_data = bdd.topic_data[TARGET_SYSTEM_TOPIC]
    for info in SENSORS.values():
        field = f"{info['hub_field']}_risk"
        assert not target_data.get(field), (
            f"Expected no {field} while the central hub is unavailable, "
            f"but captured: {target_data.get(field)}"
        )
