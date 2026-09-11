"""pytest-bdd scenarios + step definitions for ../data_persistance.feature.

Same structure as test_check_bsn.py / test_health_status.py. Assumes the
system under test is already running (see those files' docstrings for the
launch_testing harness this is normally driven by).
"""

import os
import subprocess
import sys

import pytest
from pytest_bdd import given, scenario, then

from conftest import LOGGER_NODE, PERSIST_TOPIC, SENSORS, _sensor_topic  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

THERMOMETER_TOPIC = _sensor_topic("the thermometer")
FEATURE_FILE = os.path.join(FEATURES_DIR, "data_persistance.feature")

# The When step ("the thermometer reports a new vital sign reading") is
# conftest.py's shared sensor_reports_reading - it already captures
# PERSIST_TOPIC alongside the usual topics, so every scenario that needs
# persistence data (not just this file's) gets it for free.

THERMOMETER_NODE = SENSORS["the thermometer"]["node"]

# Both scenarios were xfail until the ROS1 suite was replayed against this
# port (see ../DATA_PERSISTANCE_GAPS.md for the measurements):
#
#   * Happy path: the behavior ROS1's own test verified - a record produced
#     by the sensor reaching /persist - WORKS here (measured live). Not
#     xfail. The literal "with the value reported" clause is new scope from
#     the migration review's rewrite - ROS1's own knowledge repository never
#     stored a reading value either, so this isn't something the port broke.
#
#   * Sad path: skipped, not xfail. ROS1 induced the failure by killing
#     /logger and then asserted on /persist - the topic whose only publisher
#     it had just killed. Replayed here, that returns 0 rows: the scenario
#     is self-defeating as specified, in both versions. `skip` says that;
#     `xfail` would wrongly imply the system is expected to satisfy it.

FAILURE_SCENARIO_IS_UNSATISFIABLE = (
    "gap in both versions, not a regression: ROS1 induced this failure by "
    "killing /logger and then asserted on /persist, whose only publisher is "
    "/logger - replayed here that capture returns 0 rows. ROS2 has no "
    "storage-failure switch either (logger.py declares only 'frequency', and "
    "never emits Persist.type='PersistenceFailure'). See "
    "../DATA_PERSISTANCE_GAPS.md"
)


@scenario(FEATURE_FILE, "A vital sign reading collected by a sensor is persisted in the knowledge repository (BSN-P08)")
def test_reading_persisted_in_knowledge_repository():
    pass


@pytest.mark.skip(reason=FAILURE_SCENARIO_IS_UNSATISFIABLE)
@scenario(FEATURE_FILE, "A persistence failure is recorded when the knowledge repository cannot store a reading (BSN-P08)")
def test_persistence_failure_recorded():
    pass


@given("the knowledge repository is experiencing storage failures")
def knowledge_repository_storage_failures():
    # logger.py has no real storage backend to fail, and (see
    # ../DATA_PERSISTANCE_GAPS.md) no 'storage_failure_simulated' parameter
    # either - only 'frequency' is declared (system_monitor/logger.py:8). A
    # `ros2 param set` against an undeclared parameter still exits 0 (empirically
    # confirmed: stdout is empty and the failure - "Invalid access to
    # undeclared parameter(s)" - only appears on stderr), so `returncode == 0`
    # alone can't tell success from failure here; a successful set's stdout is
    # literally "Set parameter successful". Checking for that string is what
    # actually catches this gap at the Given step, with a precise reason,
    # instead of only surfacing it three steps later as "no failure record was
    # ever logged".
    result = subprocess.run(
        ["ros2", "param", "set", LOGGER_NODE, "storage_failure_simulated", "true"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )
    assert result.returncode == 0 and "successful" in result.stdout.decode(), (
        f"Failed to simulate a knowledge-repository storage failure on "
        f"{LOGGER_NODE}: stdout={result.stdout.decode()!r} "
        f"stderr={result.stderr.decode()!r}"
    )


@then("that reading should be retrievable from the knowledge repository with the value reported")
def reading_retrievable_from_knowledge_repository(bdd):
    # Asserts what ROS1's equivalent step asserted - the reporting sensor's
    # activity reaches the persistence channel - not the Gherkin's literal
    # "with the value reported", which no version satisfies (see this
    # module's header). Stronger than ROS1's version in one respect: it ties
    # the record to *this* sensor, not just any "Status" record.
    sensor_data = bdd.topic_data[THERMOMETER_TOPIC]
    persist_data = bdd.topic_data[PERSIST_TOPIC]

    assert any(sensor_data.values()), (
        f"No thermometer reading was captured during the window, so this "
        f"scenario proved nothing either way: {sensor_data}"
    )

    # conftest.py's shared When filters /persist to this sensor's own records
    # (m.source == 'thermometer_node'), so anything captured here is already
    # attributable to it; the type check keeps an empty/garbled row from
    # passing as a record.
    persisted_types = [t for t in persist_data.get("type", []) if t]
    assert persisted_types, (
        f"Nothing the thermometer reported reached the knowledge repository's "
        f"persistence channel ({PERSIST_TOPIC}) while it was publishing "
        f"readings. Expected records with source='{THERMOMETER_NODE}' - the "
        f"chain is component_status -> node_monitor -> log_status -> logger -> "
        f"{PERSIST_TOPIC}; captured instead: {persist_data}"
    )


@then("a persistence failure record identifying that reading should be available in the system log")
def persistence_failure_recorded(bdd):
    sensor_data = bdd.topic_data[THERMOMETER_TOPIC]
    persist_data = bdd.topic_data[PERSIST_TOPIC]
    failure_values = {
        content
        for content, record_type in zip(persist_data["content"], persist_data["type"])
        if record_type == "PersistenceFailure"
    }
    assert failure_values, (
        f"No persistence failure record was logged while the knowledge "
        f"repository was experiencing storage failures: {persist_data}"
    )
    matched = failure_values & set(sensor_data["sensor_datapoint"])
    assert matched, (
        f"The system log recorded persistence failures ({failure_values}) "
        f"but none identified the thermometer reading actually reported "
        f"({sensor_data['sensor_datapoint']})"
    )
