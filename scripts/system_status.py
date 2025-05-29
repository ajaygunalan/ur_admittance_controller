#!/usr/bin/env python3
"""
System status monitor for UR Admittance Controller.
Monitors controller state and provides periodic status updates.
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
import time


class SystemStatusMonitor(Node):
    def __init__(self):
        super().__init__('ur_admittance_status')
        
        self.declare_parameter('focus_controller', 'ur_admittance_controller')
        self.declare_parameter('check_period', 15.0)
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
        
        self.timer = self.create_timer(self.check_period, self.check_system_status)
        
        self.get_logger().info(
            f"System Status Monitor started. Checking every {self.check_period}s"
        )
        
        # Initial check after 2 seconds
        self.create_timer(2.0, self.initial_check, timer_period_ns=0)
    
    def initial_check(self):
        """Perform initial system check"""
        self.check_system_status()
    
    def check_system_status(self):
        """Check and report system status"""
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Controller manager not available")
            return
        
        request = ListControllers.Request()
        future = self.controller_client.call_async(request)
        future.add_done_callback(self.handle_controller_response)
    
    def handle_controller_response(self, future):
        """Handle the controller list response"""
        try:
            response = future.result()
            status_msg = String()
            
            # Find our focus controller
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


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SystemStatusMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()