"""launch_testing harness for ../steps/test_check_bsn.py.

Brings up the same stack as steps/conftest.py's `bsn_system`
fixture (central_hub's emergency_detection_launch.py), so `colcon test` can
run this feature on its own instead of requiring a manual `ros2 launch` +
`pytest` first. See harness.py's module docstring for the launch/skip
contract this relies on.
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


class TestCheckBsn(unittest.TestCase):

    def test_bdd_scenarios(self):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        result = run_bdd_steps("steps/test_check_bsn.py")
        assert result.returncode == 0, (
            "steps/test_check_bsn.py failed - see the pytest "
            "output above for which scenario and why"
        )
