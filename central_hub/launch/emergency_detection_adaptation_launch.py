"""Variant of emergency_detection_launch.py for BDD scenarios that need a
sensor's adaptation channel actually wired up (see features/BSN-P03.feature's
"...at its maximum sampling rate" scenario).

Two differences from emergency_detection_launch.py, both required together -
see AdaptationHandler.register_with_effector (shared_components/
adaptation_handler.py) and ParamAdapter (system_monitor/param_adapter.py):

1. thermometer_node gets `enable_adaptation: true` (redundant with the
   value already set in sensor/config/thermometer.yaml, kept explicit here
   so this launch file's own precondition for #2 doesn't silently depend on
   that yaml never changing), so Sensor.__init__ actually calls
   register_with_effector() instead of never subscribing to its
   reconfigure_<node> channel at all.
2. A param_adapter node is started, so that registration call has an
   EffectorRegister service to register with in the first place.

Only the thermometer is parameterized this way, and only the nodes BSN-P03
actually needs are started (no logger/node_monitor/other five sensors) -
each node's own LifecycleManager (shared_components/lifecycle_manager.py)
auto-configures and auto-activates itself, so nothing here depends on
node_monitor to reach the active state.
"""

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_patient = os.path.join(
        get_package_share_directory("patient"), "config", "patient_params.yaml"
    )
    config_thermometer = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="patient",
                executable="patient",
                name="patient_node",
                emulate_tty=True,
                output="screen",
                parameters=[config_patient],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="thermometer_node",
                output="screen",
                emulate_tty=True,
                # Restates the yaml's own enable_adaptation: true explicitly
                # (see the module docstring) - later entries in `parameters`
                # win over earlier ones for the same key, so this is a no-op
                # today, not a real override.
                parameters=[config_thermometer, {"enable_adaptation": True}],
            ),
            launch_ros.actions.Node(
                package="central_hub",
                executable="emergency_detection",
                name="central_hub_node",
                emulate_tty=True,
                output="screen",
                # frequency 6.0 restores the legacy hub cadence: ROS1's
                # configurations/target_system/test_g4t1.launch (the very
                # launch file test_BSN-P03.launch included) set
                # `<param name="frequency" value="6" />`. Both hubs poll the
                # same way (ROS1 CentralHub.cpp: `while(ros::ok()){ ros::Rate
                # loop_rate(getFreq()); body(); loop_rate.sleep(); }`), so the
                # cadence is the whole difference: 6 Hz caps the
                # reading -> /target_system_data latency at ~167 ms, inside
                # BSN-P03's 250 ms budget. This launch never loads
                # central_hub.yaml (unlike emergency_detection_launch.py, the
                # main stack), so without this override the hub would fall
                # back to shared_components/core/config_manager_base.py's own
                # code-level default of "1.0" - a launch override either way,
                # the same way ROS1 set it per-launch rather than changing the
                # component's own default.
                #
                # Quoted, not 6.0: config_manager_base.py declares "frequency"
                # with a string default ("1.0") and reads it back via
                # get_parameter_value().string_value - rclpy locks a
                # parameter's type to whatever its default's type was, so an
                # unquoted float here raised InvalidParameterTypeException
                # ("expecting type 'STRING'") and killed this node at
                # startup. central_hub.yaml's own frequency: '6.0' is quoted
                # for the same reason (see emergency_detection_launch.py,
                # which loads that yaml instead and never hit this).
                #
                # "component" also has to be given explicitly here, for the
                # same "this launch never loads central_hub.yaml" reason as
                # frequency above: ConfigManagerBase declares it with a ""
                # default, and ConfigManager.__init__ calls validate()
                # unconditionally at construction time, which raises
                # ValueError("component name must be set") the moment
                # self.component is still empty - killing the node before
                # it can even reach 'unconfigured', let alone 'active'.
                # "central_hub" matches central_hub.yaml's own component:
                # 'central_hub'.
                parameters=[{"frequency": "6.0", "component": "central_hub"}],
            ),
            launch_ros.actions.Node(
                package="system_monitor",
                executable="param_adapter",
                name="param_adapter",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
