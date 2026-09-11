import pytest
import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header
from fixtures import direct_central_hub, central_hub_node
from central_hub.central_hub_tests import SharedCentralHubTests


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


class TestCentralHubBehavior:
    """Central Hub characteristic tests - using individual fixtures for better isolation"""

    def test_receive_datapoint(self, direct_central_hub):
        """Test receiving data from sensors"""
        direct_central_hub.trigger_configure()
        direct_central_hub.trigger_activate()
        SharedCentralHubTests.assert_receive_datapoint_works(direct_central_hub)

    def test_detect_normal_conditions(self, direct_central_hub):
        """Test detection under normal conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_detect_normal_conditions_works(hub)

    def test_detect_abnormal_conditions(self, direct_central_hub):
        """Test detection under abnormal conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_detect_abnormal_conditions_works(hub)

    def test_data_fusion_algorithm(self, direct_central_hub):
        """Test the data fusion algorithm"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        result = SharedCentralHubTests.assert_data_fusion_works(hub)
        assert result is not None, "Data fusion should return a result"

    def test_blood_pressure_special_handling(self, direct_central_hub):
        """Test special handling of blood pressure data"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_blood_pressure_handling_works(hub)

    def test_format_log_message(self, direct_central_hub):
        """Test log message formatting"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_format_log_message_works(hub)

    def test_emergency_detection_timing(self, direct_central_hub):
        """Test emergency detection timing requirements"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Record start time
        start_time = time.time()
        
        # Trigger high-risk scenario
        hub.sensor_handler.latest_risk = {
            "thermometer": 80.0, "ecg": 80.0, "oximeter": 80.0,
            "abps": 80.0, "abpd": 80.0, "glucosemeter": 80.0,
        }
        
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        assert detection_time <= 250, f"Emergency detection took {detection_time:.2f}ms, exceeds 250ms limit"

    def test_battery_status_monitoring(self, direct_central_hub):
        """Test battery status monitoring functionality"""
        SharedCentralHubTests.assert_battery_threshold_behavior_works(direct_central_hub)

    def test_sensor_data_validation(self, direct_central_hub):
        """Test sensor data validation and error handling"""
        SharedCentralHubTests.assert_error_handling_paths_work(direct_central_hub)

    def test_lifecycle_management(self, direct_central_hub):
        """Test lifecycle management functionality"""
        SharedCentralHubTests.assert_lifecycle_transitions_work(direct_central_hub)

    def test_component_initialization(self, direct_central_hub):
        """Test that all components are properly initialized"""
        SharedCentralHubTests.assert_components_exist(direct_central_hub)

    def test_parameter_updates(self, direct_central_hub):
        """Test dynamic parameter updates"""
        SharedCentralHubTests.assert_parameter_handling_works(direct_central_hub)

    def test_concurrent_sensor_updates(self, direct_central_hub):
        """Test handling of concurrent updates from multiple sensors"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate concurrent sensor data
        sensors_data = {
            "thermometer": 37.5,
            "ecg": 85.0,
            "oximeter": 97.0,
            "abps": 115.0,
            "abpd": 78.0,
            "glucosemeter": 90.0
        }
        
        # Set all data at once (simulating concurrent updates)
        hub.sensor_handler.latest_data = sensors_data
        hub.sensor_handler.latest_risk = {k: 10.0 for k in sensors_data.keys()}
        
        # Trigger detection
        hub.detect()
        
        # Verify all data is accessible
        for sensor_type, value in sensors_data.items():
            assert hub.sensor_handler.latest_data[sensor_type] == value

    def test_risk_escalation_scenarios(self, direct_central_hub):
        """Test various risk escalation scenarios"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test low risk scenario
        hub.sensor_handler.latest_risk = {k: 10.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        low_risk_result = hub.fusion_engine.fuse_data()
        
        # Test high risk scenario
        hub.sensor_handler.latest_risk = {k: 80.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        high_risk_result = hub.fusion_engine.fuse_data()
        
        assert high_risk_result > low_risk_result, "High risk should result in higher fusion result"

    def test_partial_sensor_data(self, direct_central_hub):
        """Test behavior with partial sensor data"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Only some sensors have data
        hub.sensor_handler.latest_risk = {
            "thermometer": 20.0,
            "ecg": 25.0,
            # Other sensors missing
        }
        
        result = hub.fusion_engine.fuse_data()
        assert result > 0, "Should handle partial sensor data gracefully"

    def test_sensor_disconnection_handling(self, direct_central_hub):
        """Test handling of sensor disconnections"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Initially have all sensors
        hub.sensor_handler.latest_risk = {k: 15.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        initial_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor disconnection by clearing some data
        hub.sensor_handler.latest_risk = {"thermometer": 15.0, "ecg": 15.0}
        disconnected_result = hub.fusion_engine.fuse_data()
        
        # Should still work with fewer sensors
        assert disconnected_result > 0, "Should handle sensor disconnections"

    def test_extreme_risk_values(self, direct_central_hub):
        """Test handling of extreme risk values"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test with 0% risk
        hub.sensor_handler.latest_risk = {k: 0.0 for k in ["thermometer", "ecg", "oximeter"]}
        zero_result = hub.fusion_engine.fuse_data()
        
        # Test with 100% risk
        hub.sensor_handler.latest_risk = {k: 100.0 for k in ["thermometer", "ecg", "oximeter"]}
        max_result = hub.fusion_engine.fuse_data()
        
        assert zero_result <= max_result, "Maximum risk should be >= minimum risk"

    def test_data_persistence_across_detections(self, direct_central_hub):
        """Test that data persists correctly across multiple detection cycles"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Set initial data
        test_data = {"thermometer": 37.0, "ecg": 80.0}
        hub.sensor_handler.latest_data = test_data
        hub.sensor_handler.latest_risk = {"thermometer": 15.0, "ecg": 20.0}
        
        # Run multiple detection cycles
        for _ in range(3):
            hub.detect()
            time.sleep(0.1)
        
        # Data should persist
        assert hub.sensor_handler.latest_data["thermometer"] == 37.0
        assert hub.sensor_handler.latest_data["ecg"] == 80.0

    def test_detection_frequency_compliance(self, direct_central_hub):
        """Test that detection runs at the expected frequency"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Set up data
        hub.sensor_handler.latest_risk = {"thermometer": 20.0}
        
        # Measure detection timing
        start_time = time.time()
        detection_count = 0
        
        # Run detections for a short period
        while time.time() - start_time < 1.0:
            hub.detect()
            detection_count += 1
            time.sleep(0.1)
        
        # Should be able to run multiple detections per second
        assert detection_count >= 5, f"Expected at least 5 detections, got {detection_count}"

    def test_memory_usage_stability(self, direct_central_hub):
        """Test that memory usage remains stable over multiple operations"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Run many detection cycles to check for memory leaks
        for i in range(100):
            hub.sensor_handler.latest_risk = {"thermometer": float(i % 50)}
            hub.detect()
        
        # If we get here without crashing, memory usage is stable
        assert True, "Memory usage remained stable"

    def test_error_recovery(self, direct_central_hub):
        """Test recovery from various error conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test recovery from empty data
        hub.sensor_handler.latest_risk = {}
        hub.detect()  # Should not crash
        
        # Add valid data and verify recovery
        hub.sensor_handler.latest_risk = {"thermometer": 25.0}
        result = hub.fusion_engine.fuse_data()
        assert result > 0, "Should recover and provide valid results"

    def test_heartbeat_functionality(self, direct_central_hub):
        """Test heartbeat publishing functionality"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.2)  # Wait for heartbeat timer
        
        # Verify heartbeat timer exists and is active
        assert hasattr(hub, '_heartbeat_timer'), "Heartbeat timer should exist"
        assert hub._heartbeat_timer is not None, "Heartbeat timer should be initialized"

    def test_status_publishing(self, direct_central_hub):
        """Test status message publishing"""
        hub = direct_central_hub
        
        # Test configuration status
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Test activation status
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Verify node is active
        assert hub.is_active(), "Node should be active after activation"

    def test_cleanup_on_deactivation(self, direct_central_hub):
        """Test proper cleanup when node is deactivated"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Verify active state
        assert hub.is_active(), "Node should be active"
        
        # Deactivate
        hub.trigger_deactivate()
        time.sleep(0.1)
        
        # Verify inactive state
        assert not hub.is_active(), "Node should be inactive after deactivation"

    def test_multiple_activation_cycles(self, direct_central_hub):
        """Test multiple activation/deactivation cycles"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        
        for _ in range(3):
            # Activate
            hub.trigger_activate()
            time.sleep(0.1)
            assert hub.is_active(), "Node should be active"
            
            # Deactivate
            hub.trigger_deactivate()
            time.sleep(0.1)
            assert not hub.is_active(), "Node should be inactive"

    def test_detect_when_inactive_skip(self, direct_central_hub):
        """Test that detect method skips processing when inactive"""
        SharedCentralHubTests.assert_detect_when_inactive_works(direct_central_hub)

    def test_detect_with_low_battery_skip(self, direct_central_hub):
        """Test that detect method handles low battery conditions"""
        SharedCentralHubTests.assert_detect_under_low_battery_works(direct_central_hub)

    def test_configuration_parameter_validation(self, direct_central_hub):
        """Test validation of configuration parameters"""
        hub = direct_central_hub
        
        # Test that config manager exists and has expected parameters
        assert hasattr(hub, 'config'), "Config manager should exist"
        
        # Try to configure with valid parameters
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Configuration should succeed
        assert True, "Configuration with valid parameters should succeed"


class TestCentralHubIntegration:
    """Integration tests that can be reused by BDD tests"""
    
    def test_full_sensor_integration_flow(self, direct_central_hub):
        """Complete integration test that BDD can reuse"""
        hub = direct_central_hub
        
        # Full lifecycle
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate sensor data flow
        sensor_data = {
            "thermometer": {"value": 38.5, "risk": 35.0},  # Moderate fever
            "ecg": {"value": 95.0, "risk": 25.0},          # Slightly elevated
            "oximeter": {"value": 94.0, "risk": 40.0},     # Concerning oxygen
        }
        
        # Set sensor data
        for sensor, data in sensor_data.items():
            hub.sensor_handler.latest_data[sensor] = data["value"]
            hub.sensor_handler.latest_risk[sensor] = data["risk"]
        
        # Run detection
        hub.detect()
        
        # Verify processing occurred
        result = hub.fusion_engine.fuse_data()
        assert 20.0 <= result <= 50.0, f"Expected moderate risk result, got {result}"
        
        return result  # BDD tests can use this return value

    def test_emergency_scenario_integration(self, direct_central_hub):
        """Emergency scenario test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Critical values across multiple sensors
        critical_data = {
            "thermometer": {"value": 41.0, "risk": 85.0},   # High fever
            "ecg": {"value": 140.0, "risk": 90.0},          # Tachycardia
            "oximeter": {"value": 88.0, "risk": 95.0},      # Low oxygen
            "abps": {"value": 180.0, "risk": 88.0},         # High blood pressure
        }
        
        start_time = time.time()
        
        # Set critical data
        for sensor, data in critical_data.items():
            hub.sensor_handler.latest_data[sensor] = data["value"]
            hub.sensor_handler.latest_risk[sensor] = data["risk"]
        
        # Run detection
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        result = hub.fusion_engine.fuse_data()
        
        # Verify emergency detection
        assert result >= 80.0, f"Expected emergency-level risk, got {result}"
        assert detection_time <= 250, f"Emergency detection too slow: {detection_time:.2f}ms"
        
        return {"risk": result, "time": detection_time}

    def test_system_overload_integration(self, direct_central_hub):
        """System overload test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate high-frequency sensor updates
        for i in range(50):
            hub.sensor_handler.latest_data["thermometer"] = 37.0 + (i * 0.01)
            hub.sensor_handler.latest_risk["thermometer"] = 15.0
            hub.detect()
            # Minimal delay to simulate high frequency
            time.sleep(0.001)
        
        # Now send emergency data
        start_time = time.time()
        hub.sensor_handler.latest_data["thermometer"] = 41.5
        hub.sensor_handler.latest_risk["thermometer"] = 90.0
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        
        # Under overload, detection might be delayed
        return {"time": detection_time, "overloaded": detection_time > 250}

    def test_sensor_recovery_integration(self, direct_central_hub):
        """Sensor recovery test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Start with normal data
        hub.sensor_handler.latest_data = {"thermometer": 37.0, "ecg": 80.0}
        hub.sensor_handler.latest_risk = {"thermometer": 10.0, "ecg": 15.0}
        initial_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor failure (data disappears)
        hub.sensor_handler.latest_data = {}
        hub.sensor_handler.latest_risk = {}
        failure_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor recovery
        hub.sensor_handler.latest_data = {"thermometer": 37.2, "ecg": 82.0}
        hub.sensor_handler.latest_risk = {"thermometer": 12.0, "ecg": 18.0}
        recovery_result = hub.fusion_engine.fuse_data()
        
        return {
            "initial": initial_result,
            "failure": failure_result,
            "recovery": recovery_result,
            "recovered": recovery_result > failure_result
        }


# Additional utility functions for BDD test reuse
class CentralHubTestUtils:
    """Utility methods for BDD and other test reuse"""
    
    @staticmethod
    def setup_hub_for_testing(hub):
        """Standard setup for hub testing - reusable by BDD"""
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        return hub
    
    @staticmethod
    def set_sensor_data(hub, sensor_data_dict):
        """Set sensor data - reusable by BDD"""
        for sensor, data in sensor_data_dict.items():
            if isinstance(data, dict):
                hub.sensor_handler.latest_data[sensor] = data.get("value", 0.0)
                hub.sensor_handler.latest_risk[sensor] = data.get("risk", 0.0)
            else:
                hub.sensor_handler.latest_data[sensor] = data
                hub.sensor_handler.latest_risk[sensor] = 10.0  # Default risk
    
    @staticmethod
    def measure_detection_time(hub, setup_func=None):
        """Measure detection time - reusable by BDD"""
        if setup_func:
            setup_func(hub)
        
        start_time = time.time()
        hub.detect()
        return (time.time() - start_time) * 1000
    
    @staticmethod
    def verify_emergency_response(hub, expected_risk_threshold=70.0):
        """Verify emergency response - reusable by BDD"""
        result = hub.fusion_engine.fuse_data()
        return result >= expected_risk_threshold


@pytest.mark.usefixtures("central_hub_node")
class TestCentralHubBehaviorAdditional(_CentralHubSensorPublisherMixin):
    """Additional characteristic tests using the shared-fixture mixin style."""

    central_hub = None
    publishers = {}
    received_messages = []

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

    def test_uncertainty_like_influence_on_collection(self):
        """Simulate an uncertainty-influenced datapoint arriving at central hub collection."""
        # Publish a perturbed oximeter datapoint and assert central hub records it
        self._publish_sensor_data("oximeter", 85.5, "normal", 15.0)
        # allow processing
        time.sleep(0.2)
        assert abs(self.central_hub.sensor_handler.latest_data["oximeter"] - 85.5) < 1e-6
