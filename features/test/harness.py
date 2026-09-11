"""Shared helpers for the launch_testing files next to this one under test/.

Each test/test_*_bdd.py brings up its own stack via generate_test_description
(a launch file per BDD feature, mirroring how ROS1 pointed a `rostest` .test
file at each feature), waits for the lifecycle nodes it cares about to reach
'active', then runs the corresponding ../steps/test_*.py as a `pytest`
subprocess - matching what those files' own docstrings already describe.
Centralized here instead of duplicated per file since all the harnesses need
the exact same two steps, just with different node/file names.

A plain module in test/, not an installed package: pytest puts a test file's
own directory on sys.path, so `import harness` resolves here without setup.py
installing anything. That keeps the specs and the code that runs them in one
source tree, with no build/ or site-packages copy that could go stale or
resolve FEATURES_DIR somewhere it does not exist.

Who launches what: ../steps/conftest.py's `bsn_system` fixture (and
test_bsn_p03.py's override of it) never launches or tears down anything
itself - it only verifies the right stack is already active, which by the
time run_bdd_steps below invokes it, this module's own
wait_for_lifecycle_active call already made true.
"""

import os
import subprocess
import sys
import time

# The package root, which is the features/ directory itself: the .feature
# specs sit at its top level, their step definitions in steps/, and these
# harnesses in test/. One directory up from here, with nothing installed in
# between (see this module's docstring).
FEATURES_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wait_for_lifecycle_active(node_names, timeout_s=30.0):
    """Poll `ros2 lifecycle get <name>` for each name until every one
    reports 'active' or `timeout_s` elapses.

    Returns the set of names still not active (empty on success) rather than
    raising, so callers can produce their own assertion message with context
    about which launch file/feature was being brought up.
    """
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
    """Poll `ros2 topic info <name>` for each name until every one reports at
    least one subscriber, or `timeout_s` elapses.

    A lifecycle node reaching 'active' (wait_for_lifecycle_active) only means
    its own on_configure/on_activate finished - it says nothing about
    shared_components/adaptation_handler.py's registration flow
    (EffectorRegister call -> async response callback -> create_subscription
    on `reconfigure_<node>`), which runs independently and can still be
    pending. A scenario that asserts on that subscription existing (e.g.
    target_system.feature's "reconfiguration command" topology checks) races
    it without this wait, and fails intermittently rather than reliably.

    Returns the set of topic names still without a subscriber (empty on
    success), same contract as wait_for_lifecycle_active.
    """
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
    """Run one steps/test_*.py file as a `pytest`
    subprocess. Returns the subprocess.CompletedProcess (stdout/stderr
    inherited, so pytest's own output - including step failures - shows up
    directly in this test's output instead of being swallowed).
    """
    env = dict(os.environ)
    # colcon's `--pytest-args` (e.g. to select a single *_bdd.py file) sets
    # PYTEST_ADDOPTS on the *outer* pytest process running this launch_test.
    # Left in place, it leaks into this inner pytest run and gets applied
    # relative to the wrong cwd - e.g. `--pytest-args test/test_bsn_p03_bdd.py`
    # makes this inner run also try (and fail) to collect that same path
    # under FEATURES_DIR, where it doesn't exist, aborting with "no tests
    # ran" before even looking at `relative_test_path`.
    env.pop("PYTEST_ADDOPTS", None)
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-v", relative_test_path],
        cwd=FEATURES_DIR,
        env=env,
    )
