"""Shared helpers for the launch_testing files under test/. Each test_*_bdd.py launches its own stack, waits for
the lifecycle nodes to be 'active', then runs ../steps/test_*.py as a pytest subprocess. A plain module (not an
installed package): pytest puts this directory on sys.path, so `import harness` works without setup.py."""

import os
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

# The package root: the features/ directory itself (specs at the top level, steps in steps/, harnesses in test/).
FEATURES_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run_ros_cli(command, timeout):
    """Run a ROS CLI command and enforce its timeout for child processes too.

    Some ROS CLI calls start descendants that inherit stdout/stderr.  Killing
    only the direct client lets those descendants keep ``communicate`` open,
    which defeats ``subprocess.run(timeout=...)`` and blocks readiness for far
    longer than its contract.  A separate process group lets the timeout end
    the complete CLI invocation.
    """
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.communicate()
        raise
    return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)


def _trace_readiness(node, started, result=None, state=None):
    directory = os.environ.get("BSN_TRACE_DIR")
    if not directory:
        return
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True)
    record = dict(schema=1, event="lifecycle_query", pid=os.getpid(),
                  feature_run=os.environ.get("BSN_TRACE_FEATURE"),
                  time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
                  node=node, duration_s=time.monotonic() - started,
                  timed_out=result is None and state is None,
                  query_method="service" if os.environ.get("BSN_TRACE_DIRECT_READINESS") == "1" else "cli")
    if state is not None:
        record.update(state_id=state.id, state_label=state.label)
    if result is not None:
        record.update(exit_code=result.returncode,
                      stdout=result.stdout.decode(errors="replace"),
                      stderr=result.stderr.decode(errors="replace"))
    with (path / ("readiness_%s.jsonl" % os.getpid())).open("a") as stream:
        stream.write(json.dumps(record) + "\n")


def _wait_for_lifecycle_services(node_names, timeout_s):
    """Optional trace runner probe: same active-state contract and deadline."""
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from lifecycle_msgs.srv import GetState
    context = Context()
    rclpy.init(args=[], context=context)
    probe = Node("bsn_trace_readiness_%s" % os.getpid(), context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    pending = set(node_names)
    deadline = time.monotonic() + timeout_s
    clients = {name: probe.create_client(GetState, "/%s/get_state" % name.strip("/"))
               for name in node_names}
    futures = {}
    try:
        while pending and time.monotonic() < deadline:
            for name in list(pending):
                if name not in futures and clients[name].service_is_ready():
                    futures[name] = (clients[name].call_async(GetState.Request()), time.monotonic())
                future, started = futures.get(name, (None, None))
                if future is not None and future.done():
                    response = future.result()
                    if response is not None:
                        _trace_readiness(name, started, state=response.current_state)
                    if response is not None and response.current_state.id == 3:
                        pending.discard(name)
                    del futures[name]
            executor.spin_once(timeout_sec=0.1)
            # A response can already be queued, in which case spin_once
            # returns immediately.  Keep a bounded probe cadence for nodes
            # that are still configuring instead of issuing service calls in
            # a tight loop and perturbing the startup being observed.
            time.sleep(0.1)
        for name in pending:
            _trace_readiness(name, deadline - timeout_s)
        return pending
    finally:
        executor.shutdown()
        probe.destroy_node()
        context.shutdown()


def wait_for_lifecycle_active(node_names, timeout_s=60.0):
    """Poll `ros2 lifecycle get <name>` for each name until all report 'active' or `timeout_s` elapses.
    Returns the set still not active (empty on success) instead of raising. 60s, not 30s: each poll
    spawns a subprocess, and back-to-back suites can outrun 30s under contention."""
    if os.environ.get("BSN_TRACE_DIRECT_READINESS") == "1":
        return _wait_for_lifecycle_services(node_names, timeout_s)
    deadline = time.time() + timeout_s
    pending = set(node_names)
    while pending and time.time() < deadline:
        for name in list(pending):
            started = time.monotonic()
            try:
                result = _run_ros_cli(["ros2", "lifecycle", "get", name], timeout=5)
            except subprocess.TimeoutExpired:
                # Slow service call, not a dead node - retry.
                _trace_readiness(name, started)
                continue
            _trace_readiness(name, started, result)
            if result.stdout.decode().strip().startswith("active"):
                pending.discard(name)
        if pending:
            time.sleep(1)
    return pending


def nodes_online(*node_names, timeout_s=10.0):
    """Names still absent from the graph after its discovery window.

    Lifecycle activation and DDS graph discovery are independent.  Retrying
    avoids treating a just-started plain node as unavailable merely because a
    single ``ros2 node list`` query arrived before its discovery update.
    """
    pending = set(node_names)
    deadline = time.monotonic() + timeout_s
    while pending and time.monotonic() < deadline:
        try:
            result = _run_ros_cli(["ros2", "node", "list"], timeout=5)
        except subprocess.TimeoutExpired:
            continue
        node_list = result.stdout.decode("utf-8").splitlines()
        pending = {name for name in pending if f"/{name}" not in node_list}
        if pending:
            time.sleep(0.5)
    return [name for name in node_names if name in pending]


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
    command = [sys.executable, "-m", "pytest", "-v", relative_test_path]
    # Um diretório novo por campanha evita confundir resultados antigos com os
    # cenários internos desta execução. Sem a variável, preserva o uso habitual.
    results_dir = env.get("BSN_BDD_RESULTS_DIR")
    if results_dir:
        destination = Path(results_dir).resolve()
        destination.mkdir(parents=True, exist_ok=True)
        report = destination / (Path(relative_test_path).stem + ".xml")
        command.extend(["--junitxml=" + str(report), "-rA"])
    if env.get("BSN_COVERAGE_CAMPAIGN"):
        # A coleta multiprocesso é iniciada pelo sitecustomize da campanha.
        command.extend(["-p", "no:pytest_cov", "-o", "addopts="])
    return subprocess.run(
        command,
        cwd=FEATURES_DIR,
        env=env,
    )
