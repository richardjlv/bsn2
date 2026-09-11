"""Canonical Gherkin vocabulary -> ROS2 node/topic names.

Mirrors the ROS1 BDD suite's own interface_map.py (its `SYSTEM_MAP`, a single
flat dict from domain term to real name - see "Ajustes migracao BDDv2.md"
section 5.2 in the migration handoff docs): feature files use the domain
term ("the oximeter", "the knowledge repository", "adaptation strategy"),
and step definitions resolve the real ROS2 name from here instead of
hardcoding it - a rename becomes a one-line diff in this file instead of a
scattered edit across every scenario/step that touches it.

Includes entries for components that don't exist yet in this ROS2 port
(`/enactor`, `/reli_engine`, `/data_access`, `/injector` - see each feature's
*_GAPS.md) so their step definitions resolve names the same way as
everything else, rather than hardcoding a placeholder locally.
"""

# Sensor domain name -> ROS2 identifiers (migration doc table 3.2). Shared by
# every *.feature file that reports sensor readings.
SENSORS = {
    "the oximeter":    {"node": "oximeter_node",     "key": "oximeter",     "hub_field": "oxi"},
    "the ECG sensor":  {"node": "ecg_node",           "key": "ecg",         "hub_field": "ecg"},
    "the thermometer": {"node": "thermometer_node",   "key": "thermometer", "hub_field": "trm"},
    "the SBP sensor":  {"node": "abps_node",          "key": "abps",        "hub_field": "abps"},
    "the DBP sensor":  {"node": "abpd_node",          "key": "abpd",        "hub_field": "abpd"},
    "the glucometer":  {"node": "glucosemeter_node",  "key": "glucosemeter", "hub_field": "glc"},
}

CENTRAL_HUB_NODE = "central_hub_node"
TARGET_SYSTEM_TOPIC = "/target_system_data"

# "the system log" (canonical term, migration doc table 3.1) is the only
# persistence-related component that actually exists as a running node in
# this ROS2 codebase - see system_monitor/logger.py. Unlike the ROS1
# architecture (system log /logger -> knowledge repository /data_access as
# two separate nodes joined by the "persistence" channel), there is no
# standalone knowledge-repository node here yet: the "persistence" channel
# (/persist), published by the system log, is the sole observable surface
# for both "the system log" and "the knowledge repository" today.
LOGGER_NODE = "logger"
PERSIST_TOPIC = "/persist"

# Non-sensor domain component names -> ROS2 node names. The last four don't
# exist as running nodes today - see INJECTOR_GAPS.md / KNOWLEDGE_REPO_GAPS.md
# / MANAGING_SYSTEM_GAPS.md - but are named here anyway so their step
# definitions resolve the same way as everything else, instead of hardcoding
# a local placeholder each.
NODES = {
    "the central hub": CENTRAL_HUB_NODE,
    "the parameter adapter": "param_adapter",
    "the system log": LOGGER_NODE,
    "the log collector": "node_monitor",
    "the adaptation enactor": "enactor",
    "the reliability engine": "reli_engine",
    "the knowledge repository": "data_access",
    "the uncertainty injector": "injector",
}

PARAM_ADAPTER_NODE = NODES["the parameter adapter"]
LOG_COLLECTOR_NODE = NODES["the log collector"]
ENACTOR_NODE = NODES["the adaptation enactor"]
RELIABILITY_ENGINE_NODE = NODES["the reliability engine"]
KNOWLEDGE_REPO_NODE = NODES["the knowledge repository"]
INJECTOR_NODE = NODES["the uncertainty injector"]
PATIENT_NODE = "patient_node"

# Logical channel name (as it appears quoted in @topology scenarios) -> real
# ROS2 topic. Same idea as NODES: some of these have no publisher/subscriber
# today because the node on one end doesn't exist yet (see NODES, above).
CHANNELS = {
    "persistence": PERSIST_TOPIC,
    "monitoring data": TARGET_SYSTEM_TOPIC,
    "adaptation strategy": "/strategy",
    "adaptation exception": "/exception",
    "log event": "/event",
    "adaptation log": "/log_adapt",
    "reconfiguration request": "/reconfigure",
    "uncertainty log": "/log_uncertainty",
    "sensor event": "/log_event",
    "sensor status": "/log_status",
    "sensor energy status": "/log_energy_status",
}


def sensor_topic(sensor):
    return f"/sensor_data/{SENSORS[sensor]['key']}"


def resolve_node(component):
    """Resolve any Gherkin component name (a sensor, or another named node
    in NODES) to its ROS2 node name."""
    if component in SENSORS:
        return SENSORS[component]["node"]
    if component in NODES:
        return NODES[component]
    raise KeyError(f"Unknown component '{component}'; see SENSORS/NODES in interface_map.py")


def channel_topic(channel):
    """Resolve a logical channel name (e.g. "adaptation strategy") to its
    real ROS2 topic. For the two family channels below - one topic per
    target rather than one topic total - use reconfigure_topic /
    uncertainty_injection_topic instead."""
    return CHANNELS[channel]


def reconfigure_topic(component):
    """The parameter adapter's per-target reconfiguration channel ("the
    reconfiguration command" family, migration docs table 3.3). Only exists
    once `component` has registered with it - see
    shared_components/adaptation_handler.py.
    """
    return f"/reconfigure_{resolve_node(component)}"


def uncertainty_injection_topic(sensor):
    """The uncertainty injector's per-sensor channel ("the uncertainty
    injection" family, migration docs table 3.3). No publisher exists today
    on either end - see INJECTOR_GAPS.md.
    """
    return f"/uncertainty_{resolve_node(sensor)}"
