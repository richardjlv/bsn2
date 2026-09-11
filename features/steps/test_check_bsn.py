"""pytest-bdd scenarios + step definitions for ../check_bsn.feature.

Ported from ../steps/check_bsn_steps.py (kept in sync by hand). Assumes the
system under test is already running - either launched manually or via
../test/test_check_bsn_bdd.py's launch_testing harness, which
runs this file as a `pytest` subprocess (see that file for why).
"""

import os
import sys

from pytest_bdd import given, parsers, scenarios, then, when

from conftest import (  # noqa: E402
    CENTRAL_HUB_NODE, SENSORS, TARGET_SYSTEM_TOPIC, _kill_central_hub, _nodes_online, _sensor_topic,
)

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.asserts import any_high_risk_percentage, count_matching_elements, matching_elements  # noqa: E402
from utils.parsers import capture_topic_data, capture_topic_data_with_timeout  # noqa: E402

# Generates one test per Scenario/Outline row in check_bsn.feature.
scenarios(os.path.join(FEATURES_DIR, "check_bsn.feature"))

@given("the central hub is unavailable")
def central_hub_unavailable():
    # See conftest.py's _kill_central_hub for why killing is the only
    # reliable option here. Not restarted: these are the last two scenarios
    # in *this file* that need the hub down - see conftest.py's module
    # docstring for why that doesn't hold if another file's scenarios run in
    # the same session.
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
    # Unlike the shared "{sensor} reports a new vital sign reading" step, this
    # can't use a short fixed-line capture: the simulated patient's
    # oxigenation state (patient/config/patient_params.yaml's
    # oxigenation_State* transition matrix) only drifts into the high-risk
    # band (state 4, HighRisk1) a few percent of the time per ~5s state-change
    # attempt, so it takes on the order of minutes, not seconds, to show up
    # organically. A real timeout here means the system genuinely never
    # produced a high-risk reading - that's a real failure, and shortening
    # the window to dodge it would just hide that.
    capture_topic_data_with_timeout(
        bdd,
        [_sensor_topic("the oximeter"), TARGET_SYSTEM_TOPIC],
        timeout=240.0,
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
    assert any_high_risk_percentage(matched_risks), (
        f"The oximeter never reported (and the central hub never echoed) a "
        f"high-risk (66-100) blood oxygenation reading within the capture "
        f"window; matched risk values were: {matched_risks}"
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
