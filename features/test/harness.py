"""Shared helpers for the launch_testing files under test/. Each test_*_bdd.py launches its own stack, waits for
the lifecycle nodes to be 'active', then runs ../steps/test_*.py as a pytest subprocess. A plain module (not an
installed package): pytest puts this directory on sys.path, so `import harness` works without setup.py."""

import os
import subprocess
import sys
import time

# The package root: the features/ directory itself (specs at the top level, steps in steps/, harnesses in test/).
FEATURES_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wait_for_lifecycle_active(node_names, timeout_s=60.0):
    """Poll `ros2 lifecycle get <name>` for each name until all report 'active' or `timeout_s` elapses.
    Returns the set still not active (empty on success) instead of raising. 60s, not 30s: each poll
    spawns a subprocess, and back-to-back suites can outrun 30s under contention."""
    deadline = time.time() + timeout_s
    pending = set(node_names)
    while pending and time.time() < deadline:
        for name in list(pending):
            try:
                result = subprocess.run(
                    ["ros2", "lifecycle", "get", name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5,
                )
            except subprocess.TimeoutExpired:
                # Slow service call, not a dead node - retry.
                continue
            if result.stdout.decode().strip().startswith("active"):
                pending.discard(name)
        if pending:
            time.sleep(1)
    return pending


def nodes_online(*node_names):
    """Names (without leading '/') still missing from `ros2 node list`."""
    result = subprocess.run(
        ["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    node_list = result.stdout.decode("utf-8").splitlines()
    return [name for name in node_names if f"/{name}" not in node_list]


def wait_for_topic_subscribers(topic_names, timeout_s=30.0):
    """Poll `ros2 topic info <name>` for each name until all report a subscriber, or `timeout_s` elapses.
    A node being 'active' doesn't mean adaptation_handler's async EffectorRegister registration finished,
    so topology checks on `reconfigure_<node>` race without this wait. Returns the topics still lacking one."""
    deadline = time.time() + timeout_s
    pending = set(topic_names)
    while pending and time.time() < deadline:
        for topic in list(pending):
            try:
                result = subprocess.run(
                    ["ros2", "topic", "info", topic],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5,
                )
            except subprocess.TimeoutExpired:
                continue
            for line in result.stdout.decode().splitlines():
                if line.startswith("Subscription count:"):
                    if int(line.split(":", 1)[1].strip()) > 0:
                        pending.discard(topic)
                    break
        if pending:
            time.sleep(0.5)
    return pending


def run_bdd_steps(relative_test_path):
    """Run one steps/test_*.py file as a `pytest` subprocess and return the CompletedProcess
    (stdout/stderr inherited, so step failures show up directly in this test's output)."""
    env = dict(os.environ)
    # colcon's `--pytest-args` sets PYTEST_ADDOPTS on the outer pytest; left in place it leaks into this
    # inner run and is applied relative to the wrong cwd (e.g. collecting a path that doesn't exist here).
    env.pop("PYTEST_ADDOPTS", None)
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-v", relative_test_path],
        cwd=FEATURES_DIR,
        env=env,
    )
