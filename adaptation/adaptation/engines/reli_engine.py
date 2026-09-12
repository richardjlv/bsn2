import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import Dict, List, Optional
import re

from adaptation.engines.base_engine import Engine


from bsn_interfaces.srv import DataAccessRequest
from bsn_interfaces.msg import Strategy


class ReliabilityEngine(Engine):
    """
    Handles system reliability adaptation using MAPE-K loop
    """

    def __init__(self):
        super().__init__("reliability_engine")

        # ReliabilityEngine specific attributes matching ReliabilityEngine.hpp
        self.setpoint = 0.0
        self.offset = 0.0
        self.gain = 0.0
        self.tolerance = 0.02
        self.cycles = 0
        self.prefix = "R_"

        # ROS2 publisher for strategy
        self.strategy_publisher: Optional[rclpy.publisher.Publisher] = None

        # Additional parameters
        self.declare_parameter("setpoint", 0.9)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("gain", 0.0)
        self.declare_parameter("tolerance", 0.02)

        self.get_logger().info("ReliabilityEngine initialized")

    def setup(self):
        """Setup ReliabilityEngine"""
        # Call parent setup
        super().setup()

        # Get ReliabilityEngine specific parameters
        self.setpoint = self.get_parameter("setpoint").value
        self.offset = self.get_parameter("offset").value
        self.gain = self.get_parameter("gain").value
        self.tolerance = self.get_parameter("tolerance").value

        # Setup strategy publisher
        self.strategy_publisher = self.create_publisher(Strategy, "strategy", 10)

        self.get_logger().info(
            f"ReliabilityEngine setup - setpoint: {self.setpoint}, tolerance: {self.tolerance}"
        )

    def get_prefix(self) -> str:
        """Get prefix"""
        return self.prefix

    def initialize_strategy(self, terms: List[str]) -> Dict[str, float]:
        """Initialize strategy"""
        strategy = {}
        for term in terms:
            strategy[term] = 1.0  # Default value matching C++
        return strategy

    def initialize_priority(self, terms: List[str]) -> Dict[str, int]:
        """Initialize priority"""
        priority = {}
        for term in terms:
            if term.startswith("R_"):
                priority[term] = 50  # Default priority for R_ terms
        return priority
    

    def monitor(self):
        """Monitor phase"""
        self.get_logger().debug("Monitor phase started")
        self.cycles += 1

        # Reset strategy values
        for key in self.strategy:
            if key.startswith("CTX_"):
                self.strategy[key] = 0.0
            elif key.startswith("R_"):
                self.strategy[key] = 1.0
            elif key.startswith("F_"):
                self.strategy[key] = 1.0

        # Request reliability data from DataAccess
        self._request_reliability_data()

        # Request context data from DataAccess
        self._request_context_data()

        # Continue to analyze phase
        self.analyze()

    def _request_reliability_data(self):
        """Request reliability data"""
        try:
            if not self.data_access_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    "DataAccess service not available for reliability data"
                )
                return

            request = DataAccessRequest.Request()
            request.name = "/engine"
            request.query = f"all:reliability:{int(self.info_quant)}"

            future = self.data_access_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result():
                # `.content` - the response is a DataAccessRequest_Response
                # message, not the string these handlers parse.
                response_content = future.result().content
                self._process_reliability_response(response_content)
            else:
                self.get_logger().error("Failed to connect to data access node.")

        except Exception as e:
            self.get_logger().error(f"Error requesting reliability data: {e}")

    def _process_reliability_response(self, response_content: str):
        """Process reliability response"""
        try:
            if not response_content:
                return

            # Expected format: "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"
            pairs = response_content.split(";")

            for pair in pairs:
                if ":" in pair:
                    component, data = pair.split(":", 1)

                    # Process component name
                    first = component.lstrip("/")  # Remove leading '/'
                    first = first.upper()  # Convert to uppercase

                    # Insert underscore before 'T'
                    t_index = first.find("T")
                    if t_index != -1:
                        first = first[:t_index] + "_" + first[t_index:]

                    # Extract reliability value (last value in comma-separated list)
                    values = data.split(",")
                    if values:
                        try:
                            reliability_value = float(values[-1])
                            strategy_key = f"R_{first}"
                            self.strategy[strategy_key] = reliability_value

                            self.get_logger().debug(
                                f"Updated {strategy_key} = {reliability_value}"
                            )
                        except ValueError:
                            self.get_logger().warn(
                                f"Invalid reliability value: {values[-1]}"
                            )

        except Exception as e:
            self.get_logger().error(f"Error processing reliability response: {e}")

    def _request_context_data(self):
        """Request context data"""
        try:
            if not self.data_access_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    "DataAccess service not available for context data"
                )
                return

            request = DataAccessRequest.Request()
            request.name = "/engine"
            request.query = "all:event:1"

            future = self.data_access_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result():
                # `.content`, same as the reliability query above.
                response_content = future.result().content
                self._process_context_response(response_content)
            else:
                self.get_logger().error(
                    "Failed to connect to data access node for context."
                )

        except Exception as e:
            self.get_logger().error(f"Error requesting context data: {e}")

    def _process_context_response(self, response_content: str):
        """Process context response"""
        try:
            if not response_content:
                return

            # Expected format: "/g3t1_1:activate;/g4t1:deactivate;..."
            pairs = response_content.split(";")

            for pair in pairs:
                if ":" in pair:
                    component, event = pair.split(":", 1)

                    # Process component name
                    first = component.lstrip("/")
                    first = first.upper()

                    # Special handling for G4T1
                    if first == "G4T1":
                        ctx_key = "CTX_G4_T1"
                        r_key = "R_G4_T1"

                        if event == "activate":
                            self.strategy[ctx_key] = 1.0
                        elif event == "deactivate":
                            self.strategy[ctx_key] = 0.0
                            self.deactivated_components[r_key] = 1
                    else:
                        # Insert underscore before 'T'
                        t_index = first.find("T")
                        if t_index != -1:
                            first = first[:t_index] + "_" + first[t_index:]

                        ctx_key = f"CTX_{first}"
                        r_key = f"R_{first}"

                        if event == "activate":
                            self.strategy[ctx_key] = 1.0
                        elif event == "deactivate":
                            self.strategy[ctx_key] = 1.0  # Still set to 1 for non-G4T1
                            self.deactivated_components[r_key] = 1
                            self.strategy[r_key] = 1.0  # Set R_ to 1 when deactivated

                    self.get_logger().debug(
                        f"Updated context {ctx_key} for event: {event}"
                    )

        except Exception as e:
            self.get_logger().error(f"Error processing context response: {e}")

    def analyze(self):
        """Analyze phase"""
        self.get_logger().debug("Analyze phase started")

        # Calculate current reliability
        r_curr = self.calculate_qos(self.target_system_model, self.strategy)
        error = self.setpoint - r_curr

        self.get_logger().debug(
            f"Current reliability: {r_curr}, setpoint: {self.setpoint}, error: {error}"
        )

        # Check if error is outside tolerance
        if (error > self.setpoint * self.tolerance) or (
            error < -self.tolerance * self.setpoint
        ):
            # Check if it's time to actuate
            if self.cycles >= self.monitor_freq / self.actuation_freq:
                self.cycles = 0
                self.get_logger().info(
                    f"Reliability error detected: {error:.4f} - Planning adaptation"
                )
                self.plan()

    def plan(self):
        """Plan phase"""
        self.get_logger().debug("Plan phase started")

        # Calculate current values
        r_curr = self.calculate_qos(self.target_system_model, self.strategy)
        error = self.setpoint - r_curr

        self.get_logger().debug(
            f"Planning: setpoint={self.setpoint}, r_curr={r_curr}, error={error}"
        )

        # Get R_ components, and reset every active one's strategy value
        # before searching - unconditionally to r_curr for one that is
        # active and not deactivated, or to 1.0 for one that is deactivated.
        # Matches ReliabilityEngine.cpp:203-216 exactly: the earlier version
        # of this method tied that reset to the sign of `error` instead
        # (r_curr when error>0, else 1.0 for *every* active term), which
        # forced every component to its ceiling whenever the system was
        # already over-reliable - not what the reference implementation
        # does, and not needed, since the offset/gain search below already
        # decides which direction to move each term.
        r_vec = []
        for key in self.strategy:
            if not key.startswith("R_"):
                continue
            task = key[2:]
            ctx_key = f"CTX_{task}"
            f_key = f"F_{task}"

            # Skip inactive components (ctx=0 or f=0) - avoids looping
            # forever trying to move a term that can never affect the
            # product it belongs to.
            if self.strategy.get(ctx_key, 0) == 0 or self.strategy.get(f_key, 0) == 0:
                continue

            if not self.deactivated_components.get(key, 0):
                r_vec.append(key)
                self.strategy[key] = r_curr
            else:
                self.strategy[key] = 1.0

        # Reorder r_vec based on priority
        r_vec.sort(key=lambda x: self.priority.get(x, 0))

        # Generate solutions. For each candidate starting term, coordinate-
        # descend: offset every active term toward r_curr, climb the
        # candidate term step by step toward the setpoint, then climb every
        # other term the same way, each holding the others' results fixed.
        # Matches ReliabilityEngine.cpp:224-283. The earlier version of this
        # method only ever took a *single* `gain * error` step on a *single*
        # term per solution - fine if one lightly-off component can be
        # nudged into tolerance in one shot, but this system's formula
        # multiplies together all seven components
        # (resource/model/reliability.formula), so with more than one of
        # them degraded a single small step on a single term can never climb
        # the aggregate into a +-2% band. Confirmed live: without this loop,
        # `plan()` logged "Did not converge :(" every cycle while the
        # measured error grew instead of shrinking.
        solutions = []

        for i_key in r_vec:
            # Offset: move every active term toward r_curr, a shared
            # starting point for this candidate's search.
            for term in r_vec:
                if error > 0:
                    self.strategy[term] = r_curr * (1 - self.offset)
                elif error < 0:
                    self.strategy[term] = min(1.0, r_curr * (1 + self.offset))
            r_new = self.calculate_qos(self.target_system_model, self.strategy)

            prev = dict(self.strategy)
            if error > 0:
                while True:
                    prev = dict(self.strategy)
                    r_prev = r_new
                    self.strategy[i_key] += self.gain * error
                    r_new = self.calculate_qos(self.target_system_model, self.strategy)
                    if not (r_new < self.setpoint and r_prev < r_new and 0 < self.strategy[i_key] < 1):
                        break
            elif error < 0:
                while True:
                    prev = dict(self.strategy)
                    r_prev = r_new
                    self.strategy[i_key] += self.gain * error
                    r_new = self.calculate_qos(self.target_system_model, self.strategy)
                    if not (r_new > self.setpoint and r_prev > r_new and 0 < self.strategy[i_key] < 1):
                        break

            # Roll back to the state just before the loop's last step - the
            # candidate term's best value from this climb.
            self.strategy = prev
            r_new = self.calculate_qos(self.target_system_model, self.strategy)

            for j_key in r_vec:
                if j_key == i_key:
                    continue
                if error > 0:
                    while True:
                        prev = dict(self.strategy)
                        r_prev = r_new
                        self.strategy[j_key] += self.gain * error
                        r_new = self.calculate_qos(self.target_system_model, self.strategy)
                        if not (r_new < self.setpoint and r_prev < r_new and 0 < self.strategy[j_key] < 1):
                            break
                elif error < 0:
                    while True:
                        prev = dict(self.strategy)
                        r_prev = r_new
                        self.strategy[j_key] += self.gain * error
                        r_new = self.calculate_qos(self.target_system_model, self.strategy)
                        if not (r_new > self.setpoint and r_prev > r_new and 0 < self.strategy[j_key] < 1):
                            break
                self.strategy = prev
                r_new = self.calculate_qos(self.target_system_model, self.strategy)

            solutions.append(dict(self.strategy))

        # Test solutions
        for solution in solutions:
            self.strategy = solution
            r_new = self.calculate_qos(self.target_system_model, self.strategy)

            # Check if solution meets requirements
            if r_new > self.setpoint * (
                1 - self.tolerance
            ) and r_new < self.setpoint * (1 + self.tolerance):

                self.get_logger().info(f"Found solution with reliability: {r_new:.4f}")
                self.execute()
                return

        self.get_logger().info("Did not converge :(")

    def execute(self):
        """Execute phase"""
        self.get_logger().debug("Execute phase started")

        # Build content string
        content = ""
        flag = False

        for key, value in self.strategy.items():
            if key.startswith("R_"):
                # Convert R_ term to component name
                aux = key.lower()  # Convert to lowercase
                parts = aux.split("_")  # Split by underscore

                if len(parts) >= 3:
                    component_name = f"/{parts[1]}{parts[2]}"  # /g3t1
                    if len(parts) > 3:
                        component_name += f"_{parts[3]}"  # /g3t1_1

                    content += f"{component_name}:{value:.6f},"
                    flag = True

        # Remove last comma and add semicolon
        if flag:
            content = content.rstrip(",")

        # Publish strategy message
        if content:
            strategy_msg = Strategy()
            strategy_msg.source = "/engine"
            strategy_msg.target = "/enactor"
            strategy_msg.content = content

            self.strategy_publisher.publish(strategy_msg)

            self.get_logger().info(f"Published strategy: {content}")
        else:
            self.get_logger().warn("No strategy content to publish")


def main(args=None):
    """Main function for ReliabilityEngine node"""
    rclpy.init(args=args)

    try:
        # Create ReliabilityEngine instance
        engine = ReliabilityEngine()

        # Setup engine
        engine.setup()

        # Start engine execution
        engine.get_logger().info("Starting ReliabilityEngine...")

        # Run body() method (main MAPE-K loop)
        engine.body()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in ReliabilityEngine: {e}")
    finally:
        if "engine" in locals():
            engine.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
