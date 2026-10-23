"""Fresh ROS stack per managing-system behavioral scenario.
Topology checks share an inspection launch. Each group writes its own internal
JUnit report; the outer launch-testing item aggregates all independent launches.
Current expected failures are documented in ../MANAGING_SYSTEM_GAPS.md.
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
PLAIN_NODES = ("logger", "node_monitor", "enactor", "reli_engine", "data_access")


# Each behavioral scenario gets a fresh launch, including a new controller and
# knowledge repository. Topology cases share a read-only inspection launch.
GROUPS = {
    "topology": ("-m", "topology"),
    "increases": ("-k", "test_sampling_rate_increases"),
    "recovers": ("-k", "test_sampling_rate_decreases"),
    "deadline": ("-k", "test_an_adaptation_command_is_issued_within_10_seconds"),
    "silent": ("-k", "test_no_adaptation_is_triggered"),
    "logging": ("-k", "test_a_failed_adaptation_attempt"),
}
# Optional diagnostic subset/order. The default deliberately runs recovery before
# the deadline case: a fresh launch must remove the old ordering dependency.
SELECTED_GROUPS = os.environ.get("BSN_MANAGING_GROUPS", ",".join(GROUPS)).split(",")
if (not SELECTED_GROUPS or len(set(SELECTED_GROUPS)) != len(SELECTED_GROUPS)
        or any(group not in GROUPS for group in SELECTED_GROUPS)):
    raise ValueError("BSN_MANAGING_GROUPS must contain distinct known group names")


@pytest.mark.launch_test
@launch_testing.parametrize("bdd_group", SELECTED_GROUPS)
def generate_test_description(bdd_group):
    launch_file = os.path.join(
        get_package_share_directory("central_hub"), "launch", "emergency_detection_launch.py"
    )
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file)
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {"bdd_group": bdd_group}


class TestManagingSystem(unittest.TestCase):

    def test_bdd_scenarios(self, bdd_group):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        missing = nodes_online(*PLAIN_NODES)
        assert not missing, f"Nodes never came online: {missing}"

        result = run_bdd_steps(
            "steps/test_managing_system.py",
            pytest_args=GROUPS[bdd_group], report_suffix="_" + bdd_group,
        )
        assert result.returncode == 0, (
            "steps/test_managing_system.py failed - see the "
            "pytest output above for which scenario and why"
        )
