"""pytest-bdd scenarios + step definitions for ../injector.feature.

Both scenarios are xfail: there is no uncertainty-injector node - nor any
`/uncertainty_<node>` or `log_uncertainty` publisher - anywhere in this ROS2
port. See ../INJECTOR_GAPS.md for the full writeup (mirrors how
../MANAGING_SYSTEM_GAPS.md documents the same class of gap for
managing_system.feature).

Every scenario in this file is blocked (unlike managing_system.feature,
where 1 of 9 is testable and the rest are kept out of collection entirely),
so both are collected via individual @scenario + `pytest.mark.xfail` -
same style as test_data_persistance.py, and for the same reason: an xfail
that still runs the real check documents *why* it fails, instead of hiding
the scenario from collection.

Assumes the system under test is already running - see conftest.py's module
docstring.
"""

import os
import subprocess
import sys

import pytest
from pytest_bdd import given, parsers, scenario, then

from conftest import LOGGER_NODE, _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import CHANNELS, INJECTOR_NODE, SENSORS, uncertainty_injection_topic  # noqa: E402
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "injector.feature")

GAP_REASON = (
    "no uncertainty-injector node (nor a log_uncertainty publisher) exists "
    "in this ROS2 port - see ../INJECTOR_GAPS.md"
)


@pytest.mark.xfail(reason=GAP_REASON)
@scenario(FEATURE_FILE, "The uncertainty injector is wired to each body sensor")
def test_injector_wired_to_each_body_sensor():
    pass


@pytest.mark.xfail(reason=GAP_REASON)
@scenario(FEATURE_FILE, "The uncertainty injector is wired to the system log")
def test_injector_wired_to_system_log():
    pass


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/
    services - see utils.parsers.get_rosnode_info_ros2. Duplicated (not
    imported) from test_target_system.py/test_managing_system.py: each
    test_*.py file here runs as its own pytest subprocess (see conftest.py's
    module docstring), so there's no shared module both would import from
    without introducing one anyway.
    """
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


@given("the uncertainty injector is running")
def uncertainty_injector_running():
    # Never true today - see this module's docstring / INJECTOR_GAPS.md. Real
    # check (not a hardcoded failure) so the assertion message says exactly
    # what is missing if someone re-runs this after porting the node.
    _nodes_online(INJECTOR_NODE)


@then(parsers.parse('the uncertainty injector should publish "uncertainty injection" to {sensor}'))
def injector_publishes_injection_to_sensor(sensor):
    sensor_node = SENSORS[sensor]["node"]
    topic = uncertainty_injection_topic(sensor)

    injector_info = _node_info(INJECTOR_NODE)
    assert any(pub["topic"] == topic for pub in injector_info["publishers"]), (
        f"{INJECTOR_NODE} does not publish {topic}"
    )
    sensor_info = _node_info(sensor_node)
    assert any(sub["topic"] == topic for sub in sensor_info["subscribers"]), (
        f"{sensor_node} does not subscribe to {topic}"
    )


@then('the uncertainty injector should publish "uncertainty log" to the system log')
def injector_publishes_log_to_system_log():
    topic = CHANNELS["uncertainty log"]
    injector_info = _node_info(INJECTOR_NODE)
    assert any(pub["topic"] == topic for pub in injector_info["publishers"]), (
        f"{INJECTOR_NODE} does not publish {topic}"
    )
    logger_info = _node_info(LOGGER_NODE)
    assert any(sub["topic"] == topic for sub in logger_info["subscribers"]), (
        f"{LOGGER_NODE} does not subscribe to {topic}"
    )
