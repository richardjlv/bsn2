"""pytest-bdd scenario + step definitions for ../knowledge_repo.feature.
`/data_access` is a real node now (it was xfail for its absence, then for a wrong topic name).
Assumes the system under test is already running - see conftest.py's module docstring."""

import os
import subprocess
import sys
import time

from pytest_bdd import given, parsers, scenario, then

from conftest import _direct_graph_node, _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import CHANNELS, KNOWLEDGE_REPO_NODE, NODES  # noqa: E402
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "knowledge_repo.feature")


@scenario(FEATURE_FILE, "The knowledge repository is wired to the system log and to the central hub")
def test_knowledge_repository_wired():
    pass


def _node_info(node_name):
    if os.environ.get("BSN_TRACE_DIRECT_READINESS") == "1":
        _nodes_online(node_name)
        graph = _direct_graph_node()
        namespace, _, name = ("/" + node_name.strip("/")).rpartition("/")
        namespace = namespace or "/"
        return {
            "publishers": [{"topic": topic} for topic, _ in
                           graph.get_publisher_names_and_types_by_node(name, namespace)],
            "subscribers": [{"topic": topic} for topic, _ in
                            graph.get_subscriber_names_and_types_by_node(name, namespace)],
        }
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


def _has_endpoint(node_name, kind, topic, timeout_s=5.0):
    """Allow endpoint discovery without changing the topology contract."""
    deadline = time.monotonic() + timeout_s
    while True:
        if any(endpoint["topic"] == topic for endpoint in _node_info(node_name)[kind]):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.1)


@given("the knowledge repository is running")
def knowledge_repository_running():
    _nodes_online(KNOWLEDGE_REPO_NODE)


@then(parsers.parse('the knowledge repository should subscribe to "{channel}" published by {publisher}'))
def knowledge_repository_subscribes_to(channel, publisher):
    channel_topic = CHANNELS[channel]
    publisher_node = NODES[publisher]

    assert _has_endpoint(publisher_node, "publishers", channel_topic), (
        f"{publisher_node} does not publish {channel_topic}"
    )
    assert _has_endpoint(KNOWLEDGE_REPO_NODE, "subscribers", channel_topic), (
        f"{KNOWLEDGE_REPO_NODE} does not subscribe to {channel_topic}"
    )
