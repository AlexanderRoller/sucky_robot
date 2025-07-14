#!/usr/bin/env python3

"""
Test script for servo controller.
Usage: 
  python3 test_servo.py open   - Open doors
  python3 test_servo.py close  - Close doors
  python3 test_servo.py status - Get door status
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class ServoTest(Node):
    def __init__(self):
        super().__init__('servo_test')
        
        # Create service clients
        self.set_doors_client = self.create_client(SetBool, 'doors/set_state')
        self.get_status_client = self.create_client(Trigger, 'doors/get_status')
        
        # Create subscriber for status topic
        self.status_sub = self.create_subscription(
            Bool, 'doors/status', self.status_callback, 10)
        
        self.get_logger().info("Servo test client initialized")

    def status_callback(self, msg):
        """Callback for status topic"""
        self.get_logger().info(f"Status topic: Doors are {'OPEN' if msg.data else 'CLOSED'}")

    def wait_for_services(self, timeout=10.0):
        """Wait for servo services to become available"""
        self.get_logger().info("Waiting for servo services...")
        
        if not self.set_doors_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Set doors service not available!")
            return False
            
        if not self.get_status_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Get status service not available!")
            return False
            
        self.get_logger().info("All services available!")
        return True

    def set_doors(self, open_doors):
        """Open or close doors"""
        request = SetBool.Request()
        request.data = open_doors
        
        self.get_logger().info(f"Requesting doors {'OPEN' if open_doors else 'CLOSED'}...")
        
        try:
            future = self.set_doors_client.call_async(request)
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
        """Get door status"""
        request = Trigger.Request()
        
        self.get_logger().info("Requesting door status...")
        
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
        print("Usage: python3 test_servo.py [open|close|status]")
        print("  open   - Open doors")
        print("  close  - Close doors") 
        print("  status - Get door status")
        return

    command = sys.argv[1].lower()
    
    rclpy.init()
    
    try:
        test_node = ServoTest()
        
        # Wait for services to be available
        if not test_node.wait_for_services():
            test_node.get_logger().error("Services not available. Is servo_controller running?")
            return
        
        # Execute command
        if command == "open":
            success = test_node.set_doors(True)
        elif command == "close":
            success = test_node.set_doors(False)
        elif command == "status":
            success = test_node.get_status()
        else:
            print(f"Invalid command: {command}")
            print("Use: open, close, or status")
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
