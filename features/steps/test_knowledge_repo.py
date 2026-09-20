"""pytest-bdd scenario + step definitions for ../knowledge_repo.feature.
`/data_access` is a real node now (it was xfail for its absence, then for a wrong topic name).
Assumes the system under test is already running - see conftest.py's module docstring."""

import os
import subprocess
import sys

from pytest_bdd import given, parsers, scenario, then

from conftest import _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import CHANNELS, KNOWLEDGE_REPO_NODE, NODES  # noqa: E402
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "knowledge_repo.feature")


@scenario(FEATURE_FILE, "The knowledge repository is wired to the system log and to the central hub")
def test_knowledge_repository_wired():
    pass


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/services
    (utils.parsers.get_rosnode_info_ros2). Duplicated across test_*.py files because each runs as its own
    pytest subprocess."""
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


@given("the knowledge repository is running")
def knowledge_repository_running():
    _nodes_online(KNOWLEDGE_REPO_NODE)


@then(parsers.parse('the knowledge repository should subscribe to "{channel}" published by {publisher}'))
def knowledge_repository_subscribes_to(channel, publisher):
    channel_topic = CHANNELS[channel]
    publisher_node = NODES[publisher]

    publisher_info = _node_info(publisher_node)
    assert any(pub["topic"] == channel_topic for pub in publisher_info["publishers"]), (
        f"{publisher_node} does not publish {channel_topic}"
    )
    repo_info = _node_info(KNOWLEDGE_REPO_NODE)
    assert any(sub["topic"] == channel_topic for sub in repo_info["subscribers"]), (
        f"{KNOWLEDGE_REPO_NODE} does not subscribe to {channel_topic}"
    )
