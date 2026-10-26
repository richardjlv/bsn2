import subprocess
import importlib
import concurrent.futures
import os
import select
import time
from utils.constants import SENSOR_TOPICS, NON_SENSOR_TOPICS


def _stop_capture_process(process):
    """Stop a ``ros2 topic echo`` process without invalidating the capture timeout.

    The command-line client can keep its pipes open briefly after SIGTERM.  A
    bare ``wait()`` then turns a deliberately bounded capture into an
    unbounded test step.  Escalating to SIGKILL keeps the caller's observation
    window meaningful while preserving all data already read from stdout.
    """
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def format_entity(raw_string):
    # Check if any words in the string start with an uppercase letter
    words = raw_string.split()
    if any(word[0].isupper() for word in words):
        # If there are uppercase letters, format in camel case
        formatted_string = "".join(word.capitalize() for word in words)
    else:
        # Otherwise, format in snake case
        formatted_string = "_".join(word.lower() for word in words)

    # Add a forward slash at the beginning
    return f"/{formatted_string}"


def activate_node(node_name):
    """
    Activates a ROS 2 lifecycle node by setting it to active state.

    Args:
        node_name (str): The name of the node to activate.

    Returns:
        bool: True if the node was successfully activated, False otherwise.
    """
    try:
        # Get current lifecycle state
        current_state = get_node_lifecycle_state(node_name)
        
        if current_state is None:
            print(f"Node {node_name} is not a lifecycle node or not available.")
            return False
        
        print(f"Node {node_name} current state: {current_state}")
                
        if "inactive" in current_state.lower():
            if set_node_lifecycle_state(node_name, "activate"):
                print(f"Node {node_name} activated successfully.")
                return True
            else:
                print(f"Failed to activate node {node_name}.")
                return False
                
        elif "active" in current_state.lower():
            print(f"Node {node_name} is already active.")
            return True
        
    except Exception as e:
        print(f"Failed to activate node {node_name}: {e}")
        return False


def deactivate_node(node_name):
    """
    Deactivates a ROS 2 lifecycle node by setting it to inactive state.

    Args:
        node_name (str): The name of the node to deactivate.

    Returns:
        bool: True if the node was successfully deactivated, False otherwise.
    """
    try:
        # Get current lifecycle state
        current_state = get_node_lifecycle_state(node_name)
        
        if current_state is None:
            print(f"Node {node_name} is not a lifecycle node or not available.")
            return False
        
        print(f"Node {node_name} current state: {current_state}")
        
        if "active" in current_state.lower():
            print(f"Deactivating node {node_name}...")
            if set_node_lifecycle_state(node_name, "deactivate"):
                print(f"Node {node_name} deactivated successfully.")
                return True
            else:
                print(f"Failed to deactivate node {node_name}.")
                return False
                
        elif "inactive" in current_state.lower():
            print(f"Node {node_name} is already inactive.")
            return True
        
    except Exception as e:
        print(f"Error while deactivating node {node_name}: {e}")
        return False


# Interface contracts used by health_status. Explicit types also allow an
# observer to subscribe while the publisher is deliberately unavailable.
KNOWN_TOPIC_TYPES = {
    "/sensor_data/thermometer": "bsn_interfaces/msg/SensorData",
    "/target_system_data": "bsn_interfaces/msg/TargetSystemData",
    "/persist": "bsn_interfaces/msg/Persist",
}


def _topic_echo_command(topic, filter_expr=None):
    cmd = ["ros2", "topic", "echo", "--csv"]
    if filter_expr:
        cmd += ["--filter", filter_expr]
    cmd.append(topic)
    if topic in KNOWN_TOPIC_TYPES:
        cmd.append(KNOWN_TOPIC_TYPES[topic])
    return cmd


def get_message_attributes(topic_name):
    """Use declared BSN interfaces; discover types only for other topics."""
    message_type = KNOWN_TOPIC_TYPES.get(topic_name)
    if message_type is None:
        result = subprocess.run(
            ["ros2", "topic", "type", topic_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
        )
        if result.returncode != 0:
            raise RuntimeError("Cannot discover type of %s: %s" % (
                topic_name, result.stderr.decode("utf-8")))
        message_type = result.stdout.decode("utf-8").strip()

    module_name = message_type.replace("/", ".")
    class_name = module_name.split(".")[-1]  # Extract the last part as the class name
    module_name = ".".join(module_name.split(".")[:-1])  # Get the module part

    # Import the module and get the message class
    module = importlib.import_module(module_name)
    message_class = getattr(module, class_name)

    # Retrieve attributes
    attributes = []
    if hasattr(message_class, "__slots__"):
        attributes = list(message_class.__slots__)
    elif hasattr(message_class, "__dataclass_fields__"):
        attributes = list(message_class.__dataclass_fields__.keys())

    # Handle nested fields like 'header' and remove invalid fields
    expanded_attributes = []
    for attr in attributes:
        if attr == "header":
            expanded_attributes.extend(
                ["_header_stamp_sec", "_header_stamp_nanosec", "_header_frame_id"]
            )
        elif attr != "_check_fields":  # Exclude invalid or unnecessary fields
            expanded_attributes.append(attr)

    # Ensure header-related fields are at the front of the list
    return ["_header_stamp_sec", "_header_stamp_nanosec", "_header_frame_id"] + [
        attr for attr in expanded_attributes if attr not in ["_header", "_check_fields"]
    ]


def capture_csv_data(topic, line_limit=10, timeout=15.0, filter_expr=None):
    """
    Capture CSV data from a ROS2 topic and organize it into a dictionary with keys based on message keys.

    `timeout` is a safety net, not the normal exit path: a live topic reaches
    `line_limit` well before it and this returns early as always. It only
    matters when the publisher is gone (e.g. a step reuses this exact-worded
    step in a "hub is down" scenario) - without it, waiting on a line count
    that will never arrive hangs forever instead of returning whatever (empty)
    data was captured.

    `filter_expr`: optional `ros2 topic echo --filter` Python expression
    (using `m` for the message). Needed for a topic that many sources
    publish onto at very different rates - without it, `line_limit` counts
    against whatever is loudest, not against the source a step actually
    cares about (e.g. /persist: measured 841 lines in 15s system-wide, only
    14 of them a single sensor's readings - `line_limit=10` would almost
    never see one unfiltered).
    """
    message_keys = get_message_attributes(topic)
    if message_keys is None:
        print(f"Failed to retrieve message attributes for topic: {topic}")
        return {}
    
    output = {key[1:]: [] for key in message_keys}

    cmd = _topic_echo_command(topic, filter_expr)
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    deadline = time.time() + timeout
    try:
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            ready, _, _ = select.select([process.stdout], [], [], remaining)
            if not ready:  # timed out waiting for the next line
                break
            line = process.stdout.readline()
            if not line:  # Break if no more lines to read
                break

            values = line.strip().split(",")  # Split the CSV line into values

            if len(values) == len(
                output
            ):  # Ensure the line has the correct number of values
                # Map values to their respective keys in the output dictionary
                for key, value in zip(output.keys(), values):
                    output[key].append(value)

            # Break if all lists have at least `line_limit` items
            if all(len(v) >= line_limit for v in output.values()):
                break

    except Exception as e:
        print(f"Error capturing data from {topic}: {e}")
    finally:
        _stop_capture_process(process)

    return output


def capture_topic_data(context, topics, line_limit=10, timeout=15.0, filters=None):
    """
    Captures data from multiple ROS 2 topics using a ThreadPoolExecutor.

    Args:
        context: Behave context object to store topic data.
        topics (list): A list of ROS 2 topic names to capture data from.
        line_limit (int): Optional line limit for the data capture.
        timeout (float): Safety-net max wait per topic - see capture_csv_data.
        filters (dict): Optional {topic: filter_expr} - see capture_csv_data's
            filter_expr for why a shared/noisy topic needs one.

    Returns:
        None: Updates context.topic_data with the captured data for each topic.

    Raises:
        RuntimeError: if capturing any topic raised - re-raised rather than
            logged and swallowed. Leaving a topic out of context.topic_data
            instead would be indistinguishable from "the system genuinely
            published nothing on it", which a sad-path step asserting
            *absence* of data (via topic_data.get(...)) would then read as a
            pass for the wrong reason - the capture broke, not the system.
    """
    filters = filters or {}
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {}
        for topic in topics:
            futures[executor.submit(
                capture_csv_data, topic, line_limit, timeout, filters.get(topic)
            )] = topic

        for future in concurrent.futures.as_completed(futures):
            topic = futures[future]
            try:
                context.topic_data[topic] = future.result()
            except Exception as e:
                raise RuntimeError(f"Error while capturing data from {topic}: {e}") from e


def capture_csv_data_with_timeout(topic, timeout=3.0, filter_expr=None):
    """
    Capture CSV data from a ROS2 topic for at most `timeout` seconds.

    Unlike capture_csv_data (which blocks until `line_limit` full rows have
    arrived), this returns whatever rows arrived - possibly none - once the
    time budget runs out. Two kinds of caller need that:

    - steps asserting *absence* of data, where waiting for a line count on a
      topic that legitimately publishes nothing (e.g. a deactivated node)
      would hang forever;
    - steps comparing two topics published at very different rates, where a
      per-topic line count closes each window at a different wall-clock
      moment and the two can end up describing disjoint stretches of time
      (see conftest.py's sensor_reports_reading for the failure that caused).

    `filter_expr`: same `ros2 topic echo --filter` expression capture_csv_data
    takes - see its docstring for why a shared, noisy topic needs one.
    """
    message_keys = get_message_attributes(topic)
    output = {key[1:]: [] for key in message_keys}

    cmd = _topic_echo_command(topic, filter_expr)
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    deadline = time.time() + timeout
    try:
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            ready, _, _ = select.select([process.stdout], [], [], remaining)
            if not ready:  # timed out waiting for the next line
                break
            line = process.stdout.readline()
            if not line:
                break
            values = line.strip().split(",")
            if len(values) == len(output):
                for key, value in zip(output.keys(), values):
                    output[key].append(value)
    finally:
        _stop_capture_process(process)

    return output


def capture_topic_data_with_timeout(context, topics, timeout=3.0, filters=None):
    """Same as capture_topic_data, bounded by wall-clock time instead of a line
    count - see its docstring for why a capture failure is raised, not logged
    and swallowed. That matters even more for callers of *this* function:
    they use it specifically to assert *absence* of data, so a swallowed
    capture error and a topic that genuinely published nothing would
    otherwise be indistinguishable.

    Because every topic here shares one wall-clock window, this is also the
    right choice whenever a step compares values across topics published at
    different rates - a line count would close each window at a different
    moment. `filters` is the same {topic: filter_expr} map capture_topic_data
    takes.
    """
    filters = filters or {}
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {
            executor.submit(
                capture_csv_data_with_timeout, topic, timeout, filters.get(topic)
            ): topic
            for topic in topics
        }
        for future in concurrent.futures.as_completed(futures):
            topic = futures[future]
            try:
                context.topic_data[topic] = future.result()
            except Exception as e:
                raise RuntimeError(f"Error while capturing data from {topic}: {e}") from e


def get_rosnode_info_ros2(lines):
    node_info = {
        "publishers": [],
        "subscribers": [],
        "services": {"servers": [], "clients": []},
        "actions": {"servers": [], "clients": []},
    }

    def parse_topic_lines(start_index, keyword):
        topics = []
        i = start_index
        while i < len(lines) and lines[i].strip().startswith(keyword):
            line = lines[i].strip().split(": ")
            if len(line) == 2:
                topic = line[0].strip()
                msg_type = line[1].strip()
                topics.append({"topic": topic, "type": msg_type})
            i += 1
        return topics, i

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if line.startswith("Subscribers:"):
            node_info["subscribers"], i = parse_topic_lines(i + 1, "/")

        elif line.startswith("Publishers:"):
            node_info["publishers"], i = parse_topic_lines(i + 1, "/")

        elif line.startswith("Service Servers:"):
            i += 1
            while i < len(lines) and lines[i].strip().startswith("/"):
                service = lines[i].strip().split(": ")[0]
                node_info["services"]["servers"].append(service)
                i += 1

        elif line.startswith("Service Clients:"):
            i += 1
            while i < len(lines) and lines[i].strip().startswith("/"):
                client = lines[i].strip().split(": ")[0]
                node_info["services"]["clients"].append(client)
                i += 1

        elif line.startswith("Action Servers:"):
            i += 1
            while i < len(lines) and lines[i].strip().startswith("/"):
                action_server = lines[i].strip().split(": ")[0]
                node_info["actions"]["servers"].append(action_server)
                i += 1

        elif line.startswith("Action Clients:"):
            i += 1
            while i < len(lines) and lines[i].strip().startswith("/"):
                action_client = lines[i].strip().split(": ")[0]
                node_info["actions"]["clients"].append(action_client)
                i += 1
        else:
            i += 1

    return node_info


def get_node_lifecycle_state(node_name):
    """
    Get the current lifecycle state of a node.
    
    Args:
        node_name (str): The name of the node.
        
    Returns:
        str: The current state of the node, or None if not a lifecycle node.
    """
    try:
        result = subprocess.run(
            ["ros2", "lifecycle", "get", node_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5
        )
        
        if result.returncode == 0:
            return result.stdout.decode().strip().split()[0]
        else:
            return None
    except subprocess.TimeoutExpired:
        print(f"Timeout getting lifecycle state for {node_name}")
        return None
    except Exception as e:
        print(f"Error getting lifecycle state for {node_name}: {e}")
        return None


def set_node_lifecycle_state(node_name, transition):
    """
    Set the lifecycle state of a node.
    
    Args:
        node_name (str): The name of the node.
        transition (str): The transition to perform (configure, activate, deactivate, cleanup, shutdown).
        
    Returns:
        bool: True if successful, False otherwise.
    """
    try:
        result = subprocess.run(
            ["ros2", "lifecycle", "set", node_name, transition],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        return result.returncode == 0
        
    except Exception as e:
        print(f"Error setting lifecycle state for {node_name}: {e}")
        return False

def process_real_time_topics(context, capture_function, topics, duration=10):
    """
    Process topics concurrently and organize results into context categories.
    ROS2 version of the original ROS1 function.

    Args:
        context: Behave context object to store categorized topic data
        capture_function: Function to capture topic data
        topics: List of topic names to process
        duration: Optional duration parameter
    """
    from concurrent.futures import ThreadPoolExecutor, as_completed
    
    # Initialize context storage if not exists
    if not hasattr(context, 'sensor_data'):
        context.sensor_data = {}
    if not hasattr(context, 'target_system_data'):
        context.target_system_data = {}
    if not hasattr(context, 'non_sensor'):
        context.non_sensor = {}
    with ThreadPoolExecutor() as executor:
        # Submit capture tasks for all topics
        future_to_topic = {
            executor.submit(capture_function, topic, 10): topic
            for topic in topics
        }
        
        for future in as_completed(future_to_topic):
            topic = future_to_topic[future]
            try:
                parsed_data = future.result()
                
                # Categorize the data based on topic type
                if topic == '/target_system_data':
                    context.target_system_data[topic] = parsed_data
                elif topic in NON_SENSOR_TOPICS:
                    context.non_sensor[topic] = parsed_data
                elif any(sensor_topic in topic for sensor_topic in SENSOR_TOPICS):
                    context.sensor_data[topic] = parsed_data
                    
                    # Check for high-risk data in sensor topics
                    if 'risk_level' in parsed_data:
                        high_risk_found = any('high' in str(risk).lower() 
                                            for risk in parsed_data['risk_level'])
                        if high_risk_found:
                            if not hasattr(context, 'found_high_risk'):
                                context.found_high_risk = []
                            context.found_high_risk.append(topic)
                else:
                    # Default to non_sensor for unknown topics
                    context.non_sensor[topic] = parsed_data
                    
                print(f"Processed {topic}: {len(parsed_data) if parsed_data else 0} data points")
                
            except Exception as e:
                print(f"Error processing topic {topic}: {e}")
def restart_central_hub_node(context):
    """
    Restart the central_hub_node after it has been shutdown.
    
    This function attempts multiple restart strategies:
    1. Lifecycle state transitions (if supported)
    2. Manual node restart as separate process
    3. Full system restart as fallback
    """
    import subprocess
    import time
    
    # Strategy 1: Try lifecycle state transitions first
    try:
        print("Attempting lifecycle-based restart...")
        
        # Try to configure the node
        result = subprocess.run([
            'ros2', 'lifecycle', 'set', '/central_hub_node', 'configure'
        ], capture_output=True, text=True, timeout=10)
        print('passed here in configuration')
        if result.returncode == 0:
            # Configuration successful, try to activate
            time.sleep(2)
            result = subprocess.run([
                'ros2', 'lifecycle', 'set', '/central_hub_node', 'activate'
            ], capture_output=True, text=True, timeout=10)
            print('passed here in activation')
            if result.returncode == 0:
                print("Central hub restarted via lifecycle transitions")
                time.sleep(5)  # Wait for full initialization
                return True
        
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        print("Lifecycle restart failed, trying manual restart...")
    
    # Strategy 2: Manual node restart as separate process
    try:
        print("Starting new central_hub_node process...")
        
        # Start a new central hub node process
        context.current_launch = subprocess.Popen(
        ['ros2', 'launch', 'central_hub', 'central_hub_standalone_launch.py'],
        stdout=subprocess.DEVNULL, 
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
        )
        
        time.sleep(10)  # Wait for node to fully start
        
        # Verify the node is online
        result = subprocess.run(['ros2', 'node', 'list'], 
                              stdout=subprocess.PIPE, text=True)
        
        if '/central_hub_node' in result.stdout:
            print("Central hub restarted as separate process")
            return True
        else:
            print("Manual restart failed, node not found in node list")
            
    except Exception as e:
        print(f"Manual restart failed: {e}")
