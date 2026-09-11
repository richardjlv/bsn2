"""launch_testing harness for ../steps/test_target_system.py.

Same stack as test_check_bsn_bdd.py/test_health_status_bdd.py (central_hub's
emergency_detection_launch.py) - target_system.feature's topology and
patient-simulator scenarios exercise the full system, not the reduced one
BSN-P03 uses (see test_bsn_p03_bdd.py). param_adapter is checked separately
from LIFECYCLE_NODES: it's a plain rclpy.node.Node (system_monitor/
param_adapter.py), not a LifecycleNode, so `ros2 lifecycle get` doesn't apply
to it - but target_system.feature's "reconfiguration command" scenarios need
it up (and every sensor/hub registered with it) before they can observe
anything. See harness.py's module docstring for the launch/skip contract
this relies on.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory

from harness import (
    nodes_online, run_bdd_steps, wait_for_lifecycle_active, wait_for_topic_subscribers,
)

LIFECYCLE_NODES = (
    "thermometer_node", "oximeter_node", "ecg_node",
    "abps_node", "abpd_node", "glucosemeter_node", "central_hub_node",
)
PLAIN_NODES = ("patient_node", "param_adapter")


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


class TestTargetSystem(unittest.TestCase):

    def test_bdd_scenarios(self):
        pending = wait_for_lifecycle_active(LIFECYCLE_NODES)
        assert not pending, f"Nodes never reached 'active': {pending}"

        missing = nodes_online(*PLAIN_NODES)
        assert not missing, f"Nodes never came online: {missing}"

        # Every LIFECYCLE_NODES entry also has enable_adaptation=True in
        # emergency_detection_launch.py, so each one registers with
        # param_adapter on startup (shared_components/adaptation_handler.py).
        # 'active' above only proves on_configure/on_activate finished, not
        # that this separate async registration has - without waiting for it
        # too, the "reconfiguration command" scenarios below race it and fail
        # intermittently. See wait_for_topic_subscribers.
        reconfigure_topics = [f"/reconfigure_{name}" for name in LIFECYCLE_NODES]
        pending = wait_for_topic_subscribers(reconfigure_topics)
        assert not pending, f"No subscriber ever appeared on: {pending}"

        result = run_bdd_steps("steps/test_target_system.py")
        assert result.returncode == 0, (
            "steps/test_target_system.py failed - see the "
            "pytest output above for which scenario and why"
        )
