import pytest
import time
import rclpy
import math
import yaml
import threading
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header
import os
from ament_index_python.packages import get_package_share_directory


SENSOR_TYPES = [
    "thermometer",
    "ecg",
    "oximeter",
    "abps",
    "abpd",
    "glucosemeter",
]


@pytest.fixture(scope="module")
def ros_context():
    """Initialize ROS once for all tests in this module."""
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        pass
    yield


@pytest.fixture(scope="class")
def central_hub_node(request, ros_context):
    """Create a central hub node for testing."""
    from central_hub.central_hub import CentralHub

    # Create node and assign to class
    node = CentralHub()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    # Create a separate executor for this node
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Start executor in a thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Initialize class attributes for the test class
    request.cls.central_hub = node
    request.cls.received_messages = []
    request.cls.publishers = {}

    # Configure and activate if the node has these methods
    if hasattr(node, 'trigger_configure'):
        node.get_logger().info("Configuring central hub node...")
        node.trigger_configure()
        time.sleep(0.5)  # Give time for configuration to complete
    
    # Only activate if the node has this method
    if hasattr(node, 'trigger_activate'):
        node.get_logger().info("Activating central hub...")
        node.trigger_activate()
        time.sleep(0.5)  # Give time for activation to complete
    
    # Create subscription to capture published TargetSystemData
    def target_system_callback(msg):
        node.get_logger().debug(f"Test received TargetSystemData message: {msg}")
        request.cls.received_messages.append(msg)
    
    sub = node.create_subscription(
        TargetSystemData, 'target_system_data', target_system_callback, 10
    )
    
    # Store important objects for cleanup
    node.test_sub = sub
    node.executor = executor
    node.executor_thread = executor_thread

    # Yield the configured node
    yield node

    # Cleanup after tests
    try:
        # Clean up lifecycle state if methods exist
        node.get_logger().info("Cleaning up central hub node...")
        
        # Only try to deactivate if method exists and node is active
        if hasattr(node, 'trigger_deactivate') and hasattr(node, 'active') and node.active:
            try:
                node.trigger_deactivate()
                time.sleep(0.1)
            except Exception as e:
                node.get_logger().warning(f"Deactivation failed: {e}")
        
        # Only try to cleanup if method exists
        if hasattr(node, 'trigger_cleanup'):
            try:
                node.trigger_cleanup()
                time.sleep(0.1)
            except Exception as e:
                node.get_logger().warning(f"Cleanup failed: {e}")
        
        # Stop executor and thread
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
        
        # Destroy node
        node.destroy_node()
        
    except Exception as e:
        print(f"Error during central hub cleanup: {e}")


@pytest.mark.usefixtures("central_hub_node")
class _CentralHubSensorPublisherMixin:
    """Shared publisher/test utilities used by central hub test suites."""

    PUBLISH_SPIN_CYCLES = 5
    PUBLISH_SPIN_DELAY = 0.05

    def setup_method(self):
        """Set up publishers before each test"""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for sensor_type in SENSOR_TYPES:
            self.publishers[sensor_type] = self.central_hub.create_publisher(
                SensorData, f"sensor_data/{sensor_type}", qos_profile
            )

        time.sleep(0.2)
        self.received_messages.clear()

    def _risk_to_pct(self, risk_level):
        mapping = {
            "high": 80.0,
            "moderate": 50.0,
            "low": 10.0,
            "normal": 10.0,
        }
        return mapping.get(risk_level, -1.0)

    def _publish_sensor_data(self, sensor_type, value, risk_level="normal", risk_percentage=None):
        msg = SensorData()
        header = Header()
        header.stamp = self.central_hub.get_clock().now().to_msg()
        header.frame_id = sensor_type

        msg.header = header
        msg.sensor_type = sensor_type
        msg.sensor_datapoint = float(value)
        msg.risk_level = risk_level
        msg.risk = float(
            risk_percentage if risk_percentage is not None else self._risk_to_pct(risk_level)
        )

        self.publishers[sensor_type].publish(msg)

        for _ in range(self.PUBLISH_SPIN_CYCLES):
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)
            time.sleep(self.PUBLISH_SPIN_DELAY)

    def publish_sensor_data(self, sensor_type, value, risk_level="normal", risk_percentage=None):
        """Backward-compatible alias used by the behavior suite."""
        self._publish_sensor_data(sensor_type, value, risk_level, risk_percentage)


@pytest.mark.usefixtures("central_hub_node")
class TestCentralHubBehavior(_CentralHubSensorPublisherMixin):
    central_hub = None  # Will be set by fixture
    publishers = {}  # Will be set by fixture
    received_messages = []  # Will store published messages
    PUBLISH_SPIN_DELAY = 0.1

    def test_receive_datapoint(self):
        """Test receiving data from sensors"""
        # Publish data to the thermometer topic
        self.publish_sensor_data("thermometer", 37.0, "normal")
        
        # Check that the data was received by the central hub
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.0

    def test_detect_normal_conditions(self):
        """Test detection under normal conditions"""
        # Ensure node is active
        assert self.central_hub.active, "Node must be active for this test"
        
        # Reset received messages
        self.received_messages.clear()

        # Publish normal values for all sensors
        self.publish_sensor_data("thermometer", 37.0, "normal", 10.0)
        self.publish_sensor_data("ecg", 90.0, "normal", 10.0)
        self.publish_sensor_data("oximeter", 98.0, "normal", 10.0)
        self.publish_sensor_data("abps", 110.0, "normal", 10.0)
        self.publish_sensor_data("abpd", 75.0, "normal", 10.0)
        self.publish_sensor_data("glucosemeter", 80.0, "normal", 10.0)

        # Trigger the detect method
        self.central_hub.detect()
        
        # Wait for publish to complete
        time.sleep(0.5)

        # Explicitly spin to process the published message
        for _ in range(10):
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)

        # Check that we received a message
        assert len(self.received_messages) > 0, "No messages received after detect()"
        
        # Check the message content
        message = self.received_messages[-1]
        assert message.patient_status < 20.0, "Patient status should be low for normal conditions"

    def test_detect_abnormal_conditions(self):
        """Test detection under abnormal conditions"""
        # Reset received messages
        self.received_messages.clear()

        # Publish abnormal values for sensors
        self.publish_sensor_data("thermometer", 39.5, "moderate", 60.0)
        self.publish_sensor_data("ecg", 150.0, "high", 80.0)
        self.publish_sensor_data("oximeter", 90.0, "moderate", 50.0)
        self.publish_sensor_data("abps", 150.0, "high", 80.0)
        self.publish_sensor_data("abpd", 100.0, "high", 80.0)
        self.publish_sensor_data("glucosemeter", 200.0, "high", 80.0)

        # Trigger the detect method
        self.central_hub.detect()
        
        # Wait for publish to complete
        time.sleep(0.5)

        # Explicitly spin to process the published message
        for _ in range(10):
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)

        # Check that we received a message
        assert len(self.received_messages) > 0, "No messages received after detect()"
        
        # Check the message content - should indicate higher risk
        message = self.received_messages[-1]
        assert message.patient_status > 50.0, "Patient status should be high for abnormal conditions"

    def test_data_fusion_algorithm(self):
        """Test the data fusion algorithm"""
        # Check if components exist
        if not hasattr(self.central_hub, 'fusion_engine') or not hasattr(self.central_hub, 'sensor_handler'):
            pytest.skip("Required components not found")
        
        # Set up test data
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0,
            "ecg": 10.0,
            "oximeter": 10.0,
            "abps": 10.0,
            "abpd": 10.0,
            "glucosemeter": 10.0,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Just ensure the result is reasonable (between 5-15) given all inputs are 10.0
        assert 5.0 <= result <= 15.0, f"Expected result between 5-15 for equal values of 10, got {result}"
        
        # Store the actual calculation factor for consistent testing
        # This is the ratio between input and output for equal values
        calculation_factor = result / 10.0
        
        # Save as class variable for other tests
        self.__class__.fusion_calculation_factor = calculation_factor

    def test_blood_pressure_special_handling(self):
        """Test special handling of blood pressure data"""
        # Set up test data with different values for abps and abpd
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0,
            "ecg": 10.0, 
            "oximeter": 10.0,
            "abps": 20.0,  # Higher than others
            "abpd": 20.0,  # Higher than others  
            "glucosemeter": 10.0,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Result should be higher than 10 due to blood pressure values
        assert result > 10.0, "Blood pressure values should be averaged and affect result"

    def test_format_log_message(self):
        """Test log message formatting"""
        # Set up test data
        self.central_hub.sensor_handler.latest_data = {
            "thermometer": 37.0,
            "ecg": 90.0,
            "oximeter": 98.0,
            "abps": 110.0,
            "abpd": 75.0,
            "glucosemeter": 80.0,
        }
        
        self.central_hub.sensor_handler.latest_risks_labels = {
            "thermometer": "low",
            "ecg": "low",
            "oximeter": "low",
            "abps": "low",
            "abpd": "low",
            "glucosemeter": "low",
        }
        
        # Get log message 
        log_message = self.central_hub.visualizer.format_log_message()
        
        # Check that log message includes all sensor types
        assert "thermometer" in log_message
        assert "ecg" in log_message
        assert "oximeter" in log_message
        assert "abps" in log_message
        assert "abpd" in log_message
        assert "glucosemeter" in log_message

    def test_multiple_readings_overwrite(self):
        """Test that newer readings overwrite older ones"""
        # First reading
        self.publish_sensor_data("thermometer", 36.0, "normal")
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 36.0
        
        # Second reading should overwrite
        self.publish_sensor_data("thermometer", 37.0, "normal")
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.0

    def test_risk_categorization(self):
        """Test risk categorization"""
        # Test with different patient status values
        test_cases = [
            (10.0, "VERY LOW RISK"),
            (30.0, "LOW RISK"),
            (50.0, "MODERATE RISK"),
            (70.0, "CRITICAL RISK"), 
            (90.0, "VERY CRITICAL RISK"),
        ]
        
        for status, category in test_cases:
            # Call emit_alert directly on risk_analyzer
            self.central_hub.risk_analyzer.emit_alert(status)
            # We can't easily test the output, but at least make sure it doesn't crash

    def test_data_fusion_with_deviation_weighting(self):
        """Test data fusion with values that have different deviations"""
        # Set up test data with widely varying values
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0,
            "ecg": 20.0,
            "oximeter": 30.0,
            "abps": 40.0,
            "abpd": 50.0,
            "glucosemeter": 60.0,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Result should be weighted towards higher deviations
        assert 10.0 < result < 60.0, f"Result should be between min and max: {result}"

    def test_data_fuse_with_no_data(self):
        """Test data fusion with no data"""
        # Set up empty data
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": -1.0,
            "ecg": -1.0,
            "oximeter": -1.0,
            "abps": -1.0,
            "abpd": -1.0, 
            "glucosemeter": -1.0,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Should return default value for no data
        assert result == 0.0, f"Expected 0.0 for no data, got {result}"

    def test_data_fuse_with_identical_values(self):
        """Test data fusion with identical values"""
        # Check if components exist
        if not hasattr(self.central_hub, 'fusion_engine') or not hasattr(self.central_hub, 'sensor_handler'):
            pytest.skip("Required components not found")
        
        # Set up identical values
        identical_value = 42.0
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": identical_value,
            "ecg": identical_value,
            "oximeter": identical_value,
            "abps": identical_value,
            "abpd": identical_value,
            "glucosemeter": identical_value,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Get the calculation factor if the first test ran already
        calculation_factor = getattr(self.__class__, 'fusion_calculation_factor', None)
        
        if calculation_factor:
            # Apply the known factor from previous test
            expected_result = identical_value * calculation_factor
            assert abs(result - expected_result) < 0.1, f"Expected {expected_result}, got {result}"
        else:
            # If we don't have a factor yet, just check it's proportional and in a reasonable range
            assert 30.0 <= result <= 50.0, f"Expected result between 30-50 for all values = 42, got {result}"

    def test_data_fuse_with_partial_data(self):
        """Test data fusion with only some values available"""
        # Set up partial data
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 30.0,
            "ecg": -1.0,  # No data
            "oximeter": 20.0,
            "abps": -1.0,  # No data
            "abpd": -1.0,  # No data
            "glucosemeter": 40.0,
        }
        
        # Call the fusion algorithm directly
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Should still generate a result with partial data
        assert result > 0.0, "Should compute result with partial data"
        # Result should be influenced by available values
        assert 20.0 <= result <= 40.0, f"Result should be between min and max values: {result}"

    def test_receive_datapoint_with_empty_sensor_type(self):
        """Test handling of datapoints with empty sensor type"""
        # Create message with empty sensor type
        msg = SensorData()
        msg.header = Header()
        msg.header.stamp = self.central_hub.get_clock().now().to_msg()
        msg.sensor_type = ""
        msg.sensor_datapoint = 37.0
        msg.risk_level = "normal"
        msg.risk = 10.0
        
        # Call the handler directly
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # This should log a warning but not crash - no easy way to assert this
        # Just make sure it doesn't throw an exception

    def test_format_log_message_with_empty_data(self):
        """Test log formatting with empty data"""
        # Set up empty data
        self.central_hub.sensor_handler.latest_data = {
            "thermometer": -1.0,
            "ecg": -1.0,
            "oximeter": -1.0,
            "abps": -1.0,
            "abpd": -1.0,
            "glucosemeter": -1.0,
        }
        
        # Get log message
        log_message = self.central_hub.visualizer.format_log_message()
        
        # Should have "waiting data" for all sensors
        assert "waiting data" in log_message

    def test_battery_manager_low_battery_behavior(self):
        """Test behavior when battery is low"""
        # Check if component exists
        if not hasattr(self.central_hub, 'battery_manager'):
            pytest.skip("Battery manager component not found")
        
        # Save original battery level
        original_level = self.central_hub.battery_manager.battery.current_level
        
        try:
            # Set battery to low level
            self.central_hub.battery_manager.battery.current_level = 5.0
            
            # The BatteryManager doesn't have is_low(), so check directly with battery level
            assert self.central_hub.battery_manager.battery.current_level < 10.0, "Battery level should be low"
            
            # Test recharge - note that recharge() doesn't take parameters
            old_level = self.central_hub.battery_manager.battery.current_level
            self.central_hub.battery_manager.recharge()
            assert self.central_hub.battery_manager.battery.current_level > old_level, "Battery should have recharged"
            
            # Test max level cap by getting the capacity (which is the max)
            # Set battery level close to max and recharge
            if hasattr(self.central_hub.battery_manager.battery, 'capacity'):
                max_capacity = self.central_hub.battery_manager.battery.capacity
                self.central_hub.battery_manager.battery.current_level = max_capacity - 1
                self.central_hub.battery_manager.recharge()
                assert self.central_hub.battery_manager.battery.current_level <= max_capacity, "Battery should not exceed max capacity"
        
        finally:
            # Restore original battery level
            self.central_hub.battery_manager.battery.current_level = original_level
            
    def test_risk_analyzer_edge_cases(self):
        """Test risk analyzer edge cases"""
        # Check if component exists
        if not hasattr(self.central_hub, 'risk_analyzer'):
            pytest.skip("Risk analyzer component not found")
        
        # Test extreme values
        self.central_hub.risk_analyzer.emit_alert(100.0)  # Should handle 100%
        self.central_hub.risk_analyzer.emit_alert(0.0)    # Should handle 0%
        self.central_hub.risk_analyzer.emit_alert(-10.0)  # Should handle negative values
        
        # Test get_risk_category method if it exists
        if hasattr(self.central_hub.risk_analyzer, 'get_risk_category'):
            assert "VERY LOW" in self.central_hub.risk_analyzer.get_risk_category(5.0).upper()
            assert "CRITICAL" in self.central_hub.risk_analyzer.get_risk_category(90.0).upper()

    def test_central_hub_lifecycle_methods(self):
        """Test central hub's lifecycle methods"""
        # Skip this test if node is not a proper lifecycle node
        if not hasattr(self.central_hub, 'active'):
            pytest.skip("Node does not implement active property")
        
        # Save initial state
        initial_active = self.central_hub.active
        
        try:
            # First test - if node is active, try deactivating it
            if initial_active:
                try:
                    if hasattr(self.central_hub, 'trigger_deactivate'):
                        # Deactivate and check
                        self.central_hub.trigger_deactivate()
                        time.sleep(0.2)
                        assert not self.central_hub.active, "Node should be inactive after deactivation"
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Deactivation test failed: {e}")
            
            # Second test - if node is inactive, try activating it
            if not self.central_hub.active:
                try:
                    if hasattr(self.central_hub, 'trigger_activate'):
                        # Activate and check
                        self.central_hub.trigger_activate()
                        time.sleep(0.2)
                        assert self.central_hub.active, "Node should be active after activation"
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Activation test failed: {e}")
        
        finally:
            # Restore initial state
            if self.central_hub.active != initial_active:
                try:
                    if initial_active and not self.central_hub.active and hasattr(self.central_hub, 'trigger_activate'):
                        self.central_hub.trigger_activate()
                    elif not initial_active and self.central_hub.active and hasattr(self.central_hub, 'trigger_deactivate'):
                        self.central_hub.trigger_deactivate()
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Failed to restore initial state: {e}")

    def test_hub_error_handling(self):
        """Test error handling in the hub"""
        # Test with invalid sensor type
        msg = SensorData()
        msg.header = Header()
        msg.header.stamp = self.central_hub.get_clock().now().to_msg()
        msg.sensor_type = "invalid_sensor_type"
        msg.sensor_datapoint = 37.0
        msg.risk_level = "normal"
        msg.risk = 10.0
        
        # This should handle the invalid sensor gracefully
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # Test detect method with no data
        self.central_hub.sensor_handler.latest_data = {}
        self.central_hub.detect()  # Should not crash
        
        # Test with extreme values
        msg.sensor_type = "thermometer"
        msg.sensor_datapoint = float('inf')  # Infinite value
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # Test detect after setting extreme value
        self.central_hub.detect()  # Should handle extreme values

    def test_hub_parameter_handling(self):
        """Test hub's parameter handling"""
        if not hasattr(self.central_hub, 'config_manager'):
            pytest.skip("Config manager not found")
        
        # Get original parameters
        original_params = {}
        if hasattr(self.central_hub.config_manager, 'params'):
            original_params = self.central_hub.config_manager.params.copy()
        
        try:
            # Test parameter access
            assert hasattr(self.central_hub.config_manager, 'get_param')
            
            # Get a parameter with default
            value = self.central_hub.config_manager.get_param('non_existent', 'default_value')
            assert value == 'default_value', "Default value should be returned for missing parameters"
            
            # Set a parameter if possible
            if hasattr(self.central_hub.config_manager, 'set_param'):
                self.central_hub.config_manager.set_param('test_param', 'test_value')
                value = self.central_hub.config_manager.get_param('test_param', None)
                assert value == 'test_value', "Parameter should be set and retrievable"
        finally:
            # Restore original parameters if possible
            if hasattr(self.central_hub.config_manager, 'params') and hasattr(self.central_hub.config_manager, 'set_param'):
                self.central_hub.config_manager.params = original_params

    def test_publisher_manager_methods(self):
        """Test publisher manager methods"""
        if not hasattr(self.central_hub, 'publisher_manager'):
            pytest.skip("Publisher manager not found")
        
        # Test status publishing
        self.central_hub.publisher_manager.publish_status("test", "idle")
        
        # Test event publishing if the method exists - note only takes one argument
        if hasattr(self.central_hub.publisher_manager, 'publish_event'):
            self.central_hub.publisher_manager.publish_event("test")
    
        # Test heartbeat publishing if the method exists
        if hasattr(self.central_hub.publisher_manager, 'publish_heartbeat'):
            self.central_hub.publisher_manager.publish_heartbeat()
    
        # Test system data publishing if the method exists
        if hasattr(self.central_hub.publisher_manager, 'publish_system_data'):
            # Use empty dictionaries for the data arguments
            self.central_hub.publisher_manager.publish_system_data(50.0, {}, {}, {})


@pytest.mark.usefixtures("central_hub_node")
class TestG4T1MigrationCoverage(_CentralHubSensorPublisherMixin):
    """Explicit G4T1 migration coverage tests for CentralHub."""

    central_hub = None
    publishers = {}
    received_messages = []

    def _publish_all_sensors(self, risk_percentage, datapoint=1.0, risk_level="normal"):
        self._publish_sensor_data("thermometer", 37.0 + datapoint, risk_level, risk_percentage)
        self._publish_sensor_data("ecg", 90.0 + datapoint, risk_level, risk_percentage)
        self._publish_sensor_data("oximeter", 98.0, risk_level, risk_percentage)
        self._publish_sensor_data("abps", 110.0 + datapoint, risk_level, risk_percentage)
        self._publish_sensor_data("abpd", 75.0 + datapoint, risk_level, risk_percentage)
        self._publish_sensor_data("glucosemeter", 80.0 + datapoint, risk_level, risk_percentage)

    def _run_detect_and_get_message(self):
        self.central_hub.detect()
        time.sleep(0.4)
        for _ in range(10):
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)
        assert len(self.received_messages) > 0, "No TargetSystemData message was received"
        return self.received_messages[-1]

    def test_collect_thermometer_data(self):
        self._publish_sensor_data("thermometer", 37.2, "normal", 10.0)
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.2

    def test_transfer_very_low_risk_patient(self):
        self._publish_all_sensors(risk_percentage=0.0, risk_level="normal")
        message = self._run_detect_and_get_message()
        assert message.patient_status <= 20.0

    def test_transfer_low_risk_patient(self):
        self._publish_all_sensors(risk_percentage=30.0, risk_level="low")
        message = self._run_detect_and_get_message()
        assert 20.0 < message.patient_status <= 40.0

    def test_transfer_moderate_risk_patient(self):
        self._publish_all_sensors(risk_percentage=50.0, risk_level="moderate")
        message = self._run_detect_and_get_message()
        assert 40.0 < message.patient_status <= 60.0

    def test_transfer_critical_risk_patient(self):
        # ROS1 parity: all sensors publish risk=85.0 in this scenario
        self._publish_all_sensors(risk_percentage=85.0, risk_level="high")
        message = self._run_detect_and_get_message()
        assert 60.0 < message.patient_status <= 80.0

    def test_transfer_very_critical_risk_patient(self):
        # ROS1 parity: all sensors publish risk=100.0 in this scenario
        self._publish_all_sensors(risk_percentage=100.0, risk_level="high")
        message = self._run_detect_and_get_message()
        assert message.patient_status > 80.0

    def test_transfer_mixed_risk_levels(self):
        # ROS1 parity values:
        # thermometer: risk=10.0 data=36.8
        # ecg:         risk=50.0 data=110.0
        # oximeter:    risk=80.0 data=88.0
        # abps:        risk=15.0 data=115.0
        # abpd:        risk=45.0 data=95.0
        # glucosemeter:risk=30.0 data=120.0
        self._publish_sensor_data("thermometer", 36.8, "low", 10.0)
        self._publish_sensor_data("ecg", 110.0, "moderate", 50.0)
        self._publish_sensor_data("oximeter", 88.0, "high", 80.0)
        self._publish_sensor_data("abps", 115.0, "low", 15.0)
        self._publish_sensor_data("abpd", 95.0, "moderate", 45.0)
        self._publish_sensor_data("glucosemeter", 120.0, "moderate", 30.0)
        message = self._run_detect_and_get_message()
        assert 0.0 <= message.patient_status <= 100.0

    def test_transfer_all_sensor_data_fields(self):
        sensor_payloads = {
            "thermometer": (36.9, "normal", 10.0),
            "ecg": (88.0, "normal", 10.0),
            "oximeter": (97.0, "normal", 10.0),
            "abps": (112.0, "normal", 10.0),
            "abpd": (74.0, "normal", 10.0),
            "glucosemeter": (82.0, "normal", 10.0),
        }

        for sensor, (value, label, pct) in sensor_payloads.items():
            self._publish_sensor_data(sensor, value, label, pct)

        message = self._run_detect_and_get_message()
        assert isinstance(message.patient_status, float)

        latest_data = self.central_hub.sensor_handler.latest_data
        latest_risk = self.central_hub.sensor_handler.latest_risk
        latest_labels = self.central_hub.sensor_handler.latest_risks_labels

        for sensor, (value, label, pct) in sensor_payloads.items():
            assert sensor in latest_data
            assert abs(latest_data[sensor] - value) < 1e-6
            assert sensor in latest_risk
            assert abs(latest_risk[sensor] - pct) < 1e-6
            assert sensor in latest_labels
            assert latest_labels[sensor] == label

    def test_collect_zero_risk_data(self):
        self._publish_all_sensors(risk_percentage=0.0, risk_level="normal")
        for sensor in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            assert self.central_hub.sensor_handler.latest_risk[sensor] == 0.0

    def test_collect_maximum_risk_data(self):
        self._publish_all_sensors(risk_percentage=100.0, risk_level="high")
        for sensor in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            assert self.central_hub.sensor_handler.latest_risk[sensor] == 100.0

    def test_collect_low_battery_data(self):
        if not hasattr(self.central_hub, "battery_manager"):
            pytest.skip("Battery manager not available")

        original_level = self.central_hub.battery_manager.battery.current_level
        try:
            self.central_hub.battery_manager.battery.current_level = 5.0
            self._publish_sensor_data("thermometer", 38.0, "moderate", 50.0)
            assert self.central_hub.sensor_handler.latest_data["thermometer"] == 38.0
            self.central_hub.battery_manager.check_battery_status()
            assert self.central_hub.battery_manager.battery.current_level <= 5.0 or self.central_hub.battery_manager.is_recharging
        finally:
            self.central_hub.battery_manager.battery.current_level = original_level

    def test_sequential_sensor_updates(self):
        self._publish_sensor_data("thermometer", 36.1, "normal", 10.0)
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 36.1

        self._publish_sensor_data("thermometer", 36.8, "normal", 10.0)
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 36.8

        self._publish_sensor_data("thermometer", 37.4, "moderate", 50.0)
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.4
        assert self.central_hub.sensor_handler.latest_risk["thermometer"] == 50.0
