"""launch_testing harness for ../steps/test_managing_system.py.

Brings up the same stack as steps/conftest.py's `bsn_system`
fixture (central_hub's emergency_detection_launch.py) - that's enough for
`logger` and `node_monitor`, the two plain nodes this file's one *passing*
scenario needs. All 9 of managing_system.feature's scenarios have step
implementations; 8 are expected (`xfail`) to fail since `/enactor`,
`/reli_engine` and `/data_access` don't exist in this ROS2 port - see
../features/MANAGING_SYSTEM_GAPS.md for the full breakdown, and
harness.py's module docstring for the launch/skip contract this relies on.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory

from harness import nodes_online, run_bdd_steps, wait_for_lifecycle_active

LIFECYCLE_NODES = (
    "thermometer_node", "oximeter_node", "ecg_node",
    "abps_node", "abpd_node", "glucosemeter_node", "central_hub_node",
)
PLAIN_NODES = ("logger", "node_monitor")


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


class TestManagingSystem(unittest.TestCase):

    def test_bdd_scenarios(self):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        missing = nodes_online(*PLAIN_NODES)
        assert not missing, f"Nodes never came online: {missing}"

        result = run_bdd_steps("steps/test_managing_system.py")
        assert result.returncode == 0, (
            "steps/test_managing_system.py failed - see the "
            "pytest output above for which scenario and why"
        )
