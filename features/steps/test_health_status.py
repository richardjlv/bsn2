"""pytest-bdd scenarios + step definitions for ../health_status.feature.

Self-contained, same structure as test_check_bsn.py. Assumes the system
under test is already running - either launched manually or via
../test/test_health_status_bdd.py's launch_testing harness, which
runs this file as a `pytest` subprocess (see test_check_bsn_bdd.py for why).
"""

import os
import sys

from pytest_bdd import given, scenarios, then, when

from conftest import TARGET_SYSTEM_TOPIC, _kill_central_hub  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.asserts import is_valid_percentage  # noqa: E402
from utils.parsers import capture_topic_data_with_timeout  # noqa: E402

THERMOMETER_TOPIC = "/sensor_data/thermometer"

# Generates one test per Scenario in health_status.feature.
scenarios(os.path.join(FEATURES_DIR, "health_status.feature"))


@given("the central hub is unable to process incoming readings")
def central_hub_unable_to_process():
    # See conftest.py's _kill_central_hub for why killing is the only
    # reliable option here. Not restarted since no later scenario in *this
    # file* needs it - see conftest.py's module docstring for why that
    # doesn't hold if another file's scenarios run in the same session.
    _kill_central_hub()


@when("the thermometer reports a reading")
def thermometer_reports_reading_bounded(bdd):
    # Bounded capture: see health_status.feature's D-healthstatus-01 comment.
    # timeout=5.0: the thermometer publishes at ~1Hz (sensor/config/
    # thermometer.yaml) and `ros2 topic echo` needs time to discover the
    # topic - a shorter timeout measured 0 rows captured, which would have
    # let no_new_health_status_produced pass without exercising anything.
    capture_topic_data_with_timeout(bdd, [THERMOMETER_TOPIC, TARGET_SYSTEM_TOPIC], timeout=5.0)


@then("a patient health status derived from that reading should be available")
def patient_health_status_available(bdd):
    target_data = bdd.topic_data.get(TARGET_SYSTEM_TOPIC, {})
    assert is_valid_percentage(target_data.get("patient_status")), (
        f"No valid patient health status was reported by the central hub: "
        f"{target_data.get('patient_status')}"
    )


@then("no new patient health status should be produced for that reading")
def no_new_health_status_produced(bdd):
    # Direct indexing, not .get(..., {}): a missing key means the capture
    # thread raised (utils/parsers.py's capture_topic_data_with_timeout
    # swallows exceptions) and must fail loudly, not be mistaken for "the
    # hub produced nothing" - see test_check_bsn.py's identical-shaped steps.
    thermometer_data = bdd.topic_data[THERMOMETER_TOPIC]
    assert any(thermometer_data.values()), (
        f"No thermometer reading was captured during the window, so this "
        f"scenario proved nothing either way: {thermometer_data}"
    )

    target_data = bdd.topic_data[TARGET_SYSTEM_TOPIC]
    assert not target_data.get("patient_status"), (
        f"Expected no patient health status while the central hub cannot "
        f"process readings, but captured: {target_data.get('patient_status')}"
    )
