#!/usr/bin/env python3
"""
System Status Monitor for UR Admittance Controller

This script provides continuous monitoring of the admittance controller's
health and operational status. It periodically checks:
- Controller state (active/inactive/error)
- Controller availability
- Other active controllers in the system

The monitor publishes status updates and logs warnings/errors for
operator awareness.

Usage:
    ros2 run ur_admittance_controller system_status.py
    
Parameters:
    focus_controller: Name of controller to monitor
    check_period: Time between status checks (seconds)
    use_sim: Whether running in simulation mode

Author: UR Robotics Team
Date: 2024
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
import time


class SystemStatusMonitor(Node):
    """
    ROS2 node for monitoring admittance controller system status.
    
    Periodically queries the controller manager and publishes
    status information.
    """
    
    def __init__(self):
        """Initialize the system status monitor node."""
        super().__init__('ur_admittance_status')
        
        # Declare parameters
        self.declare_parameter('focus_controller', 'ur_admittance_controller')
        self.declare_parameter('check_period', 15.0)  # seconds
        self.declare_parameter('use_sim', True)
        
        self.focus_controller = self.get_parameter('focus_controller').value
        self.check_period = self.get_parameter('check_period').value
        self.use_sim = self.get_parameter('use_sim').value
        
        self.controller_client = self.create_client(
            ListControllers, 
            '/controller_manager/list_controllers'
        )
        
        self.status_publisher = self.create_publisher(
            String, 
            '/ur_admittance_controller/system_status', 
            10
        )
        
        # Create periodic timer for status checks
        self.timer = self.create_timer(self.check_period, self.check_system_status)
        
        self.get_logger().info(
            f"System Status Monitor started. Checking every {self.check_period}s"
        )
        
        # Schedule initial check after 2 seconds
        self.initial_timer = self.create_timer(2.0, self.initial_check)
    
    def initial_check(self) -> None:
        """
        Perform initial system check shortly after startup.
        
        This gives the system time to initialize before first check.
        """
        self.check_system_status()
        # Make this a one-shot timer by destroying it after first use
        self.destroy_timer(self.initial_timer)
    
    def check_system_status(self) -> None:
        """
        Check system status by querying the controller manager.
        
        Sends async request to list all controllers and their states.
        """
        # Check if controller manager is available
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Controller manager not available")
            return
        
        # Send async request to list controllers
        request = ListControllers.Request()
        future = self.controller_client.call_async(request)
        future.add_done_callback(self.handle_controller_response)
    
    def handle_controller_response(self, future) -> None:
        """
        Process the controller list response from controller manager.
        
        Extracts status of the focus controller and publishes status message.
        
        Args:
            future: Completed future containing ListControllers response
        """
        try:
            response = future.result()
            status_msg = String()
            
            # Search for our focus controller in the list
            controller_found = False
            controller_state = "unknown"
            
            for controller in response.controller:
                if controller.name == self.focus_controller:
                    controller_found = True
                    controller_state = controller.state
                    break
            
            if controller_found:
                if controller_state == "active":
                    status_msg.data = f"✅ {self.focus_controller} is ACTIVE"
                    self.get_logger().info(status_msg.data)
                elif controller_state == "inactive":
                    status_msg.data = f"⏸️  {self.focus_controller} is INACTIVE"
                    self.get_logger().warn(status_msg.data)
                else:
                    status_msg.data = f"❓ {self.focus_controller} state: {controller_state}"
                    self.get_logger().warn(status_msg.data)
            else:
                status_msg.data = f"❌ {self.focus_controller} NOT FOUND"
                self.get_logger().error(status_msg.data)
            
            # Publish status
            self.status_publisher.publish(status_msg)
            
            # Log all active controllers
            active_controllers = [c.name for c in response.controller if c.state == "active"]
            if active_controllers:
                self.get_logger().debug(f"Active controllers: {', '.join(active_controllers)}")
            
        except Exception as e:
            self.get_logger().error(f"Error checking controller status: {str(e)}")


def main(args=None) -> None:
    """
    Main entry point for the system status monitor.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    try:
        node = SystemStatusMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()