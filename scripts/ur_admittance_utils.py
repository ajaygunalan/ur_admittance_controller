#!/usr/bin/env python3
"""
Shared utilities and base classes for UR Admittance Controller testing and monitoring.
Consolidates common functionality to reduce code duplication.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
import threading
import time
import sys
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
class TestMode(Enum):
    """Test modes for the unified test system"""
    IMPEDANCE = "impedance"
    SAFE_STARTUP = "safe_startup"
    SYSTEM_STATUS = "system_status"
    CONTINUOUS_MONITOR = "continuous_monitor"
@dataclass
class TestConfig:
    """Configuration for test execution"""
    # Common parameters
    controller_name: str = "ur_admittance_controller"
    service_timeout: float = 5.0
    progress_update_rate: float = 2.0
    
    # Force test parameters
    force_topic: str = "/ft_sensor_readings"
    test_force: float = 20.0
    test_duration: float = 2.0
    
    # Safe startup parameters
    pose_error_topic: str = "/ur_admittance_controller/pose_error"
    max_error_threshold: float = 0.15
    test_timeout: float = 15.0
    
    # System status parameters
    check_period: float = 5.0
    focus_controller: str = ""
    realtime_logging: bool = False
class URAdmittanceTestBase(Node):
    """Base class for all UR Admittance tests with shared functionality"""
    
    def __init__(self, node_name: str, config: TestConfig = TestConfig()):
        super().__init__(node_name)
        self.config = config
        
        # Common state
        self._lock = threading.Lock()
        self._shutdown_event = threading.Event()
        self._test_active = False
        self._errors: List[str] = []
        
        # Common clients
        self._service_clients: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}
        self._subscribers: Dict[str, Any] = {}
        
        # Setup logging
        self._setup_logging()
        
    def _setup_logging(self):
        """Configure unified logging"""
        self.get_logger().info(f"Initialized {self.__class__.__name__} with config: {self.config}")
        
    def create_service_client(self, name: str, service_type: type, service_name: str) -> Any:
        """Create and cache a service client"""
        if name not in self._service_clients:
            self._service_clients[name] = self.create_client(service_type, service_name)
        return self._service_clients[name]
    
    def create_publisher(self, name: str, msg_type: type, topic: str, qos: int = 10) -> Any:
        """Create and cache a publisher"""
        if name not in self._publishers:
            self._publishers[name] = self.create_publisher(msg_type, topic, qos)
        return self._publishers[name]
        
    def create_subscriber(self, name: str, msg_type: type, topic: str, 
                         callback: Callable, qos: int = 10) -> Any:
        """Create and cache a subscriber"""
        if name not in self._subscribers:
            self._subscribers[name] = self.create_subscription(msg_type, topic, callback, qos)
        return self._subscribers[name]
    
    def wait_for_service(self, client_name: str, timeout: Optional[float] = None) -> bool:
        """Wait for a service with proper error handling"""
        timeout = timeout or self.config.service_timeout
        client = self._service_clients.get(client_name)
        
        if not client:
            self.log_error(f"Service client '{client_name}' not found")
            return False
            
        if not client.wait_for_service(timeout_sec=timeout):
            self.log_error(f"Service '{client_name}' not available after {timeout}s")
            return False
            
        return True
    
    def call_service_async(self, client_name: str, request: Any, 
                          timeout: Optional[float] = None) -> Optional[Any]:
        """Call a service asynchronously with error handling"""
        timeout = timeout or self.config.service_timeout
        client = self._service_clients.get(client_name)
        
        if not client:
            self.log_error(f"Service client '{client_name}' not found")
            return None
            
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if not future.done():
                self.log_error(f"Service call '{client_name}' timed out")
                return None
                
            return future.result()
            
        except Exception as e:
            self.log_error(f"Service call '{client_name}' failed: {str(e)}")
            return None
    
    def log_error(self, msg: str):
        """Thread-safe error logging"""
        with self._lock:
            self._errors.append(msg)
        self.get_logger().error(msg)
        
    def log_info(self, msg: str):
        """Thread-safe info logging"""
        self.get_logger().info(msg)
        
    def log_warn(self, msg: str):
        """Thread-safe warning logging"""
        self.get_logger().warn(msg)
    
    def get_errors(self) -> List[str]:
        """Get all logged errors"""
        with self._lock:
            return self._errors.copy()
    
    def clear_errors(self):
        """Clear error log"""
        with self._lock:
            self._errors.clear()
    
    def shutdown(self):
        """Clean shutdown of the node"""
        self._shutdown_event.set()
        
        # Clean up publishers, subscribers, and clients
        for pub in self._publishers.values():
            self.destroy_publisher(pub)
        for sub in self._subscribers.values():
            self.destroy_subscription(sub)
        for client in self._service_clients.values():
            self.destroy_client(client)
            
        self._publishers.clear()
        self._subscribers.clear()
        self._service_clients.clear()
class ForceTestMixin:
    """Mixin for force-based testing functionality"""
    
    def create_wrench_msg(self, force_x: float = 0.0, force_y: float = 0.0, 
                         force_z: float = 0.0, torque_x: float = 0.0, 
                         torque_y: float = 0.0, torque_z: float = 0.0) -> WrenchStamped:
        """Create a properly formatted WrenchStamped message"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = force_x
        msg.wrench.force.y = force_y
        msg.wrench.force.z = force_z
        msg.wrench.torque.x = torque_x
        msg.wrench.torque.y = torque_y
        msg.wrench.torque.z = torque_z
        return msg
    
    def apply_force_sequence(self, force_magnitude: float, duration: float, 
                           axis: str = 'x') -> bool:
        """Apply force for a duration then remove it"""
        force_pub = self._publishers.get('force')
        if not force_pub:
            self.log_error("Force publisher not initialized")
            return False
            
        # Wait for subscribers
        start_time = self.get_clock().now()
        while force_pub.get_subscription_count() == 0:
            if (self.get_clock().now() - start_time).nanoseconds > 5e9:
                self.log_warn("No subscribers for force topic")
                break
            time.sleep(0.1)
        
        # Apply force
        force_values = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        force_values[axis] = force_magnitude
        
        msg = self.create_wrench_msg(**{f'force_{k}': v for k, v in force_values.items()})
        force_pub.publish(msg)
        self.log_info(f"Applied {force_magnitude}N force in {axis} direction")
        
        # Non-blocking wait
        def remove_force():
            time.sleep(duration)
            if not self._shutdown_event.is_set():
                zero_msg = self.create_wrench_msg()
                force_pub.publish(zero_msg)
                self.log_info("Force removed")
        
        thread = threading.Thread(target=remove_force, daemon=True)
        thread.start()
        
        return True
class StiffnessControlMixin:
    """Mixin for stiffness parameter control"""
    
    def set_stiffness_parameters(self, stiffness_values: List[float]) -> bool:
        """Set stiffness parameters via service call"""
        param_client = self._service_clients.get('parameters')
        if not param_client:
            self.log_error("Parameter service client not initialized")
            return False
            
        # Create parameter
        param = Parameter()
        param.name = 'admittance.stiffness'
        param.value = ParameterValue(type=7, double_array_value=stiffness_values)
        
        # Create and send request
        request = SetParameters.Request()
        request.parameters = [param]
        
        result = self.call_service_async('parameters', request)
        if result and result.results[0].successful:
            self.log_info(f"Stiffness set to: {stiffness_values}")
            return True
        else:
            error_msg = "Unknown error"
            if result:
                error_msg = result.results[0].reason
            self.log_error(f"Failed to set stiffness: {error_msg}")
            return False
class SystemMonitorMixin:
    """Mixin for system monitoring functionality"""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._expected_controllers = []
        self._data_received = {}
        
    def check_controllers(self) -> Dict[str, str]:
        """Check controller states"""
        list_ctrl_client = self._service_clients.get('list_controllers')
        if not list_ctrl_client:
            self.log_error("Controller list service not initialized")
            return {}
            
        request = ListControllers.Request()
        result = self.call_service_async('list_controllers', request, timeout=3.0)
        
        if not result:
            return {}
            
        controller_states = {}
        for controller in result.controller:
            controller_states[controller.name] = controller.state
            
        return controller_states
    
    def check_data_flow(self) -> Dict[str, bool]:
        """Check data flow status"""
        with self._lock:
            return self._data_received.copy()
    
    def update_data_status(self, topic: str, received: bool = True):
        """Update data reception status"""
        with self._lock:
            self._data_received[topic] = received
class TestRunner:
    """Unified test runner that can execute different test modes"""
    
    def __init__(self, config: TestConfig = TestConfig()):
        self.config = config
        self.executor = None
        self.node = None
        
    def run(self, mode: TestMode) -> int:
        """Run the specified test mode"""
        try:
            rclpy.init()
            
            # Create appropriate test node based on mode
            if mode == TestMode.IMPEDANCE:
                from ur_admittance_tests import ImpedanceTest
                self.node = ImpedanceTest(self.config)
            elif mode == TestMode.SAFE_STARTUP:
                from ur_admittance_tests import SafeStartupTest
                self.node = SafeStartupTest(self.config)
            elif mode == TestMode.SYSTEM_STATUS:
                from ur_admittance_tests import SystemStatusMonitor
                self.node = SystemStatusMonitor(self.config)
            else:
                raise ValueError(f"Unknown test mode: {mode}")
            
            # Setup executor
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Run in separate thread
            executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            executor_thread.start()
            
            # Execute the test
            result = self.node.execute()
            
            return 0 if result else 1
            
        except KeyboardInterrupt:
            if self.node:
                self.node.log_info("Test interrupted by user")
            return 130
        except Exception as e:
            if self.node:
                self.node.log_error(f"Test failed: {str(e)}")
            else:
                print(f"Failed to initialize: {str(e)}")
            return 1
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.node:
            self.node.shutdown()
            self.node.destroy_node()
        if self.executor:
            self.executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()