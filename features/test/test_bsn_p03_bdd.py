"""launch_testing harness for ../steps/test_bsn_p03.py.

Unlike test_check_bsn_bdd.py/test_health_status_bdd.py, this brings up
emergency_detection_adaptation_launch.py, not the plain
emergency_detection_launch.py: BSN-P03.feature's second scenario needs the
thermometer's adaptation channel actually wired up (enable_adaptation=true
plus a running param_adapter) - see that launch file's docstring. See
harness.py's module docstring for the launch/skip contract this relies on.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory

from harness import nodes_online, run_bdd_steps, wait_for_lifecycle_active

# emergency_detection_adaptation_launch.py only starts these two as
# LifecycleNodes; patient_node and param_adapter are plain rclpy.node.Node.
LIFECYCLE_NODES = ("thermometer_node", "central_hub_node")
PLAIN_NODES = ("patient_node", "param_adapter")


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("central_hub"),
        "launch",
        "emergency_detection_adaptation_launch.py",
    )
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file)
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestBsnP03(unittest.TestCase):

    def test_bdd_scenarios(self):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        missing = nodes_online(*PLAIN_NODES)
        assert not missing, f"Nodes never came online: {missing}"

        result = run_bdd_steps("steps/test_bsn_p03.py")
        assert result.returncode == 0, (
            "steps/test_bsn_p03.py failed - see the pytest "
            "output above for which scenario and why"
        )
