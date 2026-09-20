"""launch_testing harness for ../steps/test_managing_system.py.
Same stack as conftest.py's `bsn_system`; `/enactor`, `/reli_engine` and `/data_access` are checked too,
so a node dying on startup fails here. Current xfails: ../MANAGING_SYSTEM_GAPS.md; contract: harness.py."""

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
PLAIN_NODES = ("logger", "node_monitor", "enactor", "reli_engine", "data_access")


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
