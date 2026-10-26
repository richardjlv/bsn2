import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time
from collections import deque, defaultdict
import math

# ROS2 Messages and Services
from bsn_interfaces.msg import (
    Status, Event, Strategy, AdaptationCommand, Exception as ExceptionMsg
)
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from std_msgs.msg import Header


# ROS2 node name -> BSN component identifier. The rest of the adaptation
# subsystem speaks the BSN key space end to end: DataAccess reports
# reliability per "/g3t1_1" (data_access.process_query), ReliabilityEngine
# emits strategies for those same names (reli_engine.execute), and
# Controller.apply_reli_strategy already special-cases "/g4t1" for the hub.
# Lifecycle Events are the exception - they carry `node.get_name()`
# ("oximeter_node") - so they are translated on arrival. Without that, a
# component ends up with r_curr/r_ref keyed by BSN name and
# freq/exception_buffer/kp_individual keyed by node name, and
# apply_reli_strategy raises KeyError on the first of those it touches.
# Kept in sync with data_access.component_mapping and
# param_adapter.bsn_to_ros_mapping.
NODE_TO_BSN_COMPONENT = {
    "thermometer_node": "/g3t1_3",
    "oximeter_node": "/g3t1_1",
    "ecg_node": "/g3t1_2",
    "abps_node": "/g3t1_4",
    "abpd_node": "/g3t1_5",
    "glucosemeter_node": "/g3t1_6",
    "central_hub_node": "/g4t1",
}


def bsn_component_name(name):
    """Resolve `name` to the BSN component identifier used as the key for
    every per-component dict in the Enactor. Idempotent: a name that is
    already a BSN identifier ("/g3t1_1") is returned unchanged.
    """
    key = name.lstrip("/")
    if key in NODE_TO_BSN_COMPONENT:
        return NODE_TO_BSN_COMPONENT[key]
    return name if name.startswith("/") else f"/{key}"


class Enactor(Node):
    """
    Enactor component for BSN system
    
   
    Receives adaptation strategies from engines
    Translates strategies into component-specific commands
    Sends adaptation commands to components via ParamAdapter
    """

    def __init__(self, node_name='enactor'):
        super().__init__(node_name)
        self.get_logger().info("Starting BSN Enactor")
        
        # Declare parameters (matching C++ original)
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('stability_margin', 0.02)
        
        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.stability_margin = self.get_parameter('stability_margin').value
        
        self.invocations = {}  # Maps component name to deque of success/failure (1/0)
        self.exception_buffer = {}  # Track exceptions by component
        self.freq = {}  # Current frequency by component
        self.r_curr = {}  # Current reliability by component
        self.c_curr = {}  # Current cost by component
        self.r_ref = {}  # Reference reliability by component
        self.c_ref = {}  # Reference cost by component
        self.replicate_task = {}  # Task replication count by component
        
        self.cycles = 0
        self.adaptation_parameter = "reliability"  # Default value
        
        # Setup ROS interfaces
        self.setup()
        
        # Get adaptation parameter from engine
        self.receive_adaptation_parameter()
        
        # Main processing timer (equivalent to body() in C++)
        self.create_timer(1.0/self.frequency, self.body)
        
        self.get_logger().info(f'Enactor initialized with freq={self.frequency}')
    
    def setup(self):
        """Setup ROS interfaces (equivalent to setUp() in C++)"""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers (matching C++ original). The enactor's only
        # AdaptationCommand publisher is log_adapt, as in ROS1's
        # Controller.cpp:12 - logger.py persists each command and relays it
        # to 'reconfigure' for delivery (Logger.cpp:39-47), so the system log
        # is what sends reconfiguration requests to the parameter adapter
        # (managing_system.feature). Publishing to 'reconfigure' here as well
        # would deliver every command twice.
        self.adapt = self.create_publisher(
            AdaptationCommand, 'log_adapt', qos_profile)

        self.except_pub = self.create_publisher(
            ExceptionMsg, 'exception', qos_profile)
        
        # Subscribers
        self.strategy_sub = self.create_subscription(
            Strategy, 'strategy', self.receive_strategy, qos_profile)
        
        # Service clients
        self.engine_client = self.create_client(EngineRequest, 'EngineRequest')
        self.data_access_client = self.create_client(DataAccessRequest, 'DataAccessRequest')
        
        # Wait for services
        self._wait_for_services()
    
    def _wait_for_services(self):
        """Wait for required services to be available"""
        while not self.engine_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EngineRequest service not available, waiting...')
        
        while not self.data_access_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DataAccessRequest service not available, waiting...')
    
    def tear_down(self):
        """Cleanup (equivalent to tearDown() in C++)"""
        self.get_logger().info("Tearing down Enactor")
        # Clear all data structures
        self.invocations.clear()
        self.exception_buffer.clear()
        self.freq.clear()
        self.r_curr.clear()
        self.c_curr.clear()
        self.r_ref.clear()
        self.c_ref.clear()
        self.replicate_task.clear()
    
    def body(self):
        """Main processing cycle (equivalent to body() in C++)"""
        self.cycles += 1
        
        # Limit cycles to prevent infinite counting
        if self.cycles <= 60 * self.frequency:
            # Only start adaptation after system has stabilized (60 cycles)
            if self.cycles > 60:
                self.receive_status()
        else:
            # Reset cycles to prevent overflow
            self.cycles = 61
            self.receive_status()
    
    def receive_status(self):
        """Query component status from DataAccess (matching C++ original)"""
        request = DataAccessRequest.Request()
        request.name = "/enactor"  # Important: must match C++ format
        
        if self.adaptation_parameter == "reliability":
            request.query = "all:reliability:"
        else:
            request.query = "all:cost:"
        
        future = self.data_access_client.call_async(request)
        future.add_done_callback(self._status_response_callback)
    
    def _status_response_callback(self, future):
        """Process data access response with component status"""
        try:
            response = future.result()
            if response is None or response.content == "":
                self.get_logger().warn("Empty response from data access")
                return
            
            self._process_status_response(response.content)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to process status: {str(e)}")
    
    def _process_status_response(self, content):
        """Process status response content (matching C++ parsing logic)"""
        # Parse response (format: "component1:value1,value2,value3;component2:...")
        pairs = content.split(';')
        
        for pair in pairs:
            if ':' not in pair or pair.strip() == "":
                continue
                
            component_data = pair.split(':')
            if len(component_data) < 2:
                continue
                
            component = component_data[0].strip()
            content_values = component_data[1].strip()
            
            # Parse values (podem ser múltiplos valores separados por vírgula)
            values = content_values.split(',')
            if not values:
                continue
            
            try:
                # Use the LAST value (matching C++ behavior)
                current_value = float(values[-1])
            except ValueError:
                continue
                
            # Update current values based on adaptation parameter
            if self.adaptation_parameter == "reliability":
                self.r_curr[component] = current_value
                self.apply_reli_strategy(component)
            else:  # cost
                self.c_curr[component] = current_value
                self.apply_cost_strategy(component)
    
    def receive_strategy(self, msg):
        """Process strategy messages from adaptation engines (matching C++ original)"""
        strategy_content = msg.content
        self.get_logger().info(f"Received strategy: {strategy_content}")
        
        # Parse strategy content (format: "component1:value1;component2:value2")
        # Matching C++ split behavior
        refs = strategy_content.split(';')
        
        for ref in refs:
            if ':' not in ref or ref.strip() == "":
                continue
                
            pair = ref.split(':')
            if len(pair) != 2:
                continue
                
            component = pair[0].strip()
            try:
                value = float(pair[1].strip())
            except ValueError:
                continue
            
            # Apply strategy based on adaptation parameter (matching C++ logic)
            if self.adaptation_parameter == "reliability":
                self.r_ref[component] = value
            else:
                self.c_ref[component] = value
    
    def receive_adaptation_parameter(self):
        """Get adaptation parameter from engine (matching C++ original)"""
        request = EngineRequest.Request()
        
        future = self.engine_client.call_async(request)
        future.add_done_callback(self._adaptation_parameter_response)
    
    def _adaptation_parameter_response(self, future):
        """Process engine response with adaptation parameter"""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Empty response from engine")
                return
                
            parameter = response.content.strip()
            if parameter in ["reliability", "cost"]:
                self.adaptation_parameter = parameter
                self.get_logger().info(f"Adaptation parameter set to: {parameter}")
            else:
                self.get_logger().error(f"Invalid adaptation parameter: {parameter}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to get adaptation parameter: {str(e)}")
    
    def print_status(self):
        """Print current status (equivalent to print() in C++)"""
        self.get_logger().info(f"=== Enactor Status ===")
        self.get_logger().info(f"Cycles: {self.cycles}")
        self.get_logger().info(f"Adaptation Parameter: {self.adaptation_parameter}")
        self.get_logger().info(f"Active Components: {len(self.r_curr)}")
        
        for component in self.r_curr.keys():
            reliability = self.r_curr.get(component, 0.0)
            cost = self.c_curr.get(component, 0.0)
            frequency = self.freq.get(component, 0.0)
            exceptions = self.exception_buffer.get(component, 0)
            
            self.get_logger().info(
                f"  {component}: R={reliability:.3f}, C={cost:.3f}, "
                f"F={frequency:.3f}, E={exceptions}")
    
    # Abstract methods that must be implemented by subclasses
    def receive_event(self, msg):
        """
        Abstract method: Process events from components
        Must be implemented by subclass (e.g., Controller)
        """
        raise NotImplementedError("Subclass must implement receive_event()")
    
    def apply_reli_strategy(self, component):
        """
        Abstract method: Apply reliability strategy to component
        Must be implemented by subclass (e.g., Controller)
        """
        raise NotImplementedError("Subclass must implement apply_reli_strategy()")
    
    def apply_cost_strategy(self, component):
        """
        Abstract method: Apply cost strategy to component
        Must be implemented by subclass (e.g., Controller)
        """
        raise NotImplementedError("Subclass must implement apply_cost_strategy()")
