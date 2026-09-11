"""pytest-bdd scenario + step definitions for ../knowledge_repo.feature.

The one scenario here is xfail: there is no standalone knowledge-repository
node in this ROS2 port - see ../KNOWLEDGE_REPO_GAPS.md for the full writeup
(mirrors ../MANAGING_SYSTEM_GAPS.md's evidence for the same node, referenced
there as a prerequisite for managing_system.feature's #7 and #4).

Collected via @scenario + `pytest.mark.xfail` (not hidden from collection) -
same style as test_injector.py/test_data_persistance.py: the real check
still runs, so the failure output states exactly which node/topic is
missing instead of the scenario just silently not existing.

Assumes the system under test is already running - see conftest.py's module
docstring.
"""

import os
import subprocess
import sys

import pytest
from pytest_bdd import given, parsers, scenario, then

from conftest import _nodes_online  # noqa: E402

FEATURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FEATURES_DIR, "steps"))

from utils.interface_map import CHANNELS, KNOWLEDGE_REPO_NODE, NODES  # noqa: E402
from utils.parsers import get_rosnode_info_ros2  # noqa: E402

FEATURE_FILE = os.path.join(FEATURES_DIR, "knowledge_repo.feature")

GAP_REASON = (
    "no standalone knowledge-repository node (/data_access) exists in this "
    "ROS2 port, and nothing subscribes to /persist or /target_system_data "
    "either - see ../KNOWLEDGE_REPO_GAPS.md"
)


@pytest.mark.xfail(reason=GAP_REASON)
@scenario(FEATURE_FILE, "The knowledge repository is wired to the system log and to the central hub")
def test_knowledge_repository_wired():
    pass


def _node_info(node_name):
    """ros2 node info for `node_name`, parsed into publishers/subscribers/
    services - see utils.parsers.get_rosnode_info_ros2. Duplicated (not
    imported) from the other test_*.py files here: each runs as its own
    pytest subprocess (see conftest.py's module docstring), so there's no
    shared module all of them would import from without introducing one
    anyway.
    """
    result = subprocess.run(
        ["ros2", "node", "info", f"/{node_name}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
    )
    assert result.returncode == 0, f"Failed to get info for /{node_name}: {result.stderr.decode()}"
    return get_rosnode_info_ros2(result.stdout.decode("utf-8").splitlines())


@given("the knowledge repository is running")
def knowledge_repository_running():
    # Never true today - see this module's docstring / KNOWLEDGE_REPO_GAPS.md.
    # Real check (not a hardcoded failure) so the assertion message says
    # exactly what is missing if someone re-runs this after porting the node.
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
