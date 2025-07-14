#!/usr/bin/env python3

"""
Test script for cyclone controller.
Usage: 
  python3 test_cyclone.py on     - Turn cyclone ON
  python3 test_cyclone.py off    - Turn cyclone OFF
  python3 test_cyclone.py status - Get cyclone status
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class CycloneTest(Node):
    def __init__(self):
        super().__init__('cyclone_test')
        
        # Create service clients
        self.set_cyclone_client = self.create_client(SetBool, 'cyclone/set_state')
        self.get_status_client = self.create_client(Trigger, 'cyclone/get_status')
        
        # Create subscriber for status topic
        self.status_sub = self.create_subscription(
            Bool, 'cyclone/status', self.status_callback, 10)
        
        self.get_logger().info("Cyclone test client initialized")

    def status_callback(self, msg):
        """Callback for status topic"""
        self.get_logger().info(f"Status topic: Cyclone is {'ON' if msg.data else 'OFF'}")

    def wait_for_services(self, timeout=10.0):
        """Wait for cyclone services to become available"""
        self.get_logger().info("Waiting for cyclone services...")
        
        if not self.set_cyclone_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Set state service not available!")
            return False
            
        if not self.get_status_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Get status service not available!")
            return False
            
        self.get_logger().info("All services available!")
        return True

    def set_cyclone(self, state):
        """Turn cyclone on or off"""
        request = SetBool.Request()
        request.data = state
        
        self.get_logger().info(f"Requesting cyclone {'ON' if state else 'OFF'}...")
        
        try:
            future = self.set_cyclone_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ Success: {response.message}")
                else:
                    self.get_logger().error(f"✗ Failed: {response.message}")
                return response.success
            else:
                self.get_logger().error("✗ Service call failed (timeout or error)")
                return False
        except Exception as e:
            self.get_logger().error(f"✗ Exception during service call: {e}")
            return False

    def get_status(self):
        """Get cyclone status"""
        request = Trigger.Request()
        
        self.get_logger().info("Requesting cyclone status...")
        
        try:
            future = self.get_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ Status: {response.message}")
                else:
                    self.get_logger().error(f"✗ Failed: {response.message}")
                return response.success
            else:
                self.get_logger().error("✗ Service call failed (timeout or error)")
                return False
        except Exception as e:
            self.get_logger().error(f"✗ Exception during service call: {e}")
            return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_cyclone.py [on|off|status]")
        print("  on     - Turn cyclone ON")
        print("  off    - Turn cyclone OFF") 
        print("  status - Get cyclone status")
        return

    command = sys.argv[1].lower()
    
    rclpy.init()
    
    try:
        test_node = CycloneTest()
        
        # Wait for services to be available
        if not test_node.wait_for_services():
            test_node.get_logger().error("Services not available. Is cyclone_controller running?")
            return
        
        # Execute command
        if command == "on":
            success = test_node.set_cyclone(True)
        elif command == "off":
            success = test_node.set_cyclone(False)
        elif command == "status":
            success = test_node.get_status()
        else:
            print(f"Invalid command: {command}")
            print("Use: on, off, or status")
            return
            
        if success:
            test_node.get_logger().info("Command completed successfully")
        else:
            test_node.get_logger().error("Command failed")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
