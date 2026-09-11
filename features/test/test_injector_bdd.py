"""launch_testing harness for ../steps/test_injector.py.

Same stack as test_check_bsn_bdd.py/test_data_persistance_bdd.py
(central_hub's emergency_detection_launch.py) - injector.feature relies on
conftest.py's default `bsn_system` fixture, which expects exactly this
launch file's LIFECYCLE_NODES. Both scenarios are xfail (no injector node
exists to bring up - see ../features/INJECTOR_GAPS.md), so this harness
still asserts `returncode == 0`: pytest itself exits 0 for an xfail (not
xpass) result, same as every other harness in this package. See
harness.py's module docstring for the launch/skip contract this relies on.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory

from harness import run_bdd_steps, wait_for_lifecycle_active

LIFECYCLE_NODES = (
    "thermometer_node", "oximeter_node", "ecg_node",
    "abps_node", "abpd_node", "glucosemeter_node", "central_hub_node",
)


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("central_hub"), "launch", "emergency_detection_launch.py"
    )
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file)
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestInjector(unittest.TestCase):

    def test_bdd_scenarios(self):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        result = run_bdd_steps("steps/test_injector.py")
        assert result.returncode == 0, (
            "steps/test_injector.py failed - see the pytest "
            "output above for which scenario and why"
        )
