import subprocess
import importlib
import concurrent.futures
import select
import time


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


def activate_node(node_name, package_name, executable_name):
    """
    Activates a ROS 2 node by running it.

    Args:
        node_name (str): The name of the node to activate.
        package_name (str): The ROS 2 package containing the node.
        executable_name (str): The executable name of the node.

    Returns:
        subprocess.Popen: The process object for the activated node.
    """
    try:
        process = subprocess.Popen(
            ["ros2", "run", package_name, executable_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"Node {node_name} activated.")
        return process
    except Exception as e:
        print(f"Failed to activate node {node_name}: {e}")
        return None


def deactivate_node(node_name):
    """
    Deactivates a ROS 2 node by killing it.

    Args:
        node_name (str): The name of the node to deactivate.

    Returns:
        bool: True if the node was successfully deactivated, False otherwise.
    """
    try:
        result = subprocess.run(
            ["ros2", "node", "kill", "--node-name", node_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode == 0:
            print(f"Node {node_name} deactivated.")
            return True
        else:
            print(f"Failed to deactivate node {node_name}.")
            return False
    except Exception as e:
        print(f"Error while deactivating node {node_name}: {e}")
        return False


def get_message_attributes(topic_name):
    """Dynamically get the attributes of a ROS2 message type."""
    # Convert 'format_data/msg/Data' to 'format_data.msg.Data'
    result = subprocess.run(
        ["ros2", "topic", "type", topic_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )
    if result.returncode != 0:
        raise Exception(f"Error getting node info: {result.stderr.decode('utf-8')}")

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
    output = {key[1:]: [] for key in message_keys}

    cmd = ["ros2", "topic", "echo", "--csv"]
    if filter_expr:
        cmd += ["--filter", filter_expr]
    cmd.append(topic)
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
        print(f"An error occurred: {e}")
    finally:
        process.terminate()  # Ensure the process exits properly
        process.wait()  # Ensure the process exits properly

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

    cmd = ["ros2", "topic", "echo", "--csv"]
    if filter_expr:
        cmd += ["--filter", filter_expr]
    cmd.append(topic)
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
        process.terminate()
        process.wait()

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
