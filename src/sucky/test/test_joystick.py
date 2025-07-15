#!/usr/bin/env python3

"""
Test script for sucky joy controller functionality.
This script simulates button presses to test the sucky joy controller without needing a physical PS4 controller.

Usage:
  python3 test_joystick.py cyclone  - Test cyclone toggle
  python3 test_joystick.py doors    - Test doors toggle  
  python3 test_joystick.py emergency - Test emergency stop
  python3 test_joystick.py status   - Test status check
  python3 test_joystick.py all      - Test all buttons in sequence
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time


class JoystickTester(Node):
    def __init__(self):
        super().__init__('joystick_tester')
        
        # Publisher for joy messages
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        
        self.get_logger().info("Joystick tester ready")

    def publish_button_press(self, button_index, button_name):
        """Publish a joy message with the specified button pressed"""
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create button array with enough elements (PS4 controller has 16 buttons)
        joy_msg.buttons = [0] * 16
        joy_msg.buttons[button_index] = 1  # Press the button
        
        # Add some axes data (PS4 controller has 8 axes)
        joy_msg.axes = [0.0] * 8
        
        self.get_logger().info(f"Pressing {button_name} (button {button_index})")
        self.joy_pub.publish(joy_msg)
        
        # Brief pause then release
        time.sleep(0.1)
        
        # Release button
        joy_msg.buttons[button_index] = 0
        self.joy_pub.publish(joy_msg)

    def test_cyclone(self):
        """Test cyclone toggle (Square button - index 0)"""
        self.publish_button_press(0, "Square (Cyclone Toggle)")

    def test_doors(self):
        """Test doors toggle (Triangle button - index 2)"""
        self.publish_button_press(2, "Triangle (Doors Toggle)")

    def test_emergency(self):
        """Test emergency stop (Circle button - index 1)"""
        self.publish_button_press(1, "Circle (Emergency Stop)")

    def test_status(self):
        """Test status check (Cross button - index 3)"""
        self.publish_button_press(3, "Cross (Status Check)")

    def test_all(self):
        """Test all buttons in sequence"""
        self.get_logger().info("Testing all buttons in sequence...")
        
        # Test status first
        self.test_status()
        time.sleep(2.0)
        
        # Test cyclone on
        self.test_cyclone()
        time.sleep(2.0)
        
        # Test doors open
        self.test_doors()
        time.sleep(2.0)
        
        # Check status again
        self.test_status()
        time.sleep(2.0)
        
        # Test emergency stop
        self.test_emergency()
        time.sleep(2.0)
        
        # Final status check
        self.test_status()
        
        self.get_logger().info("All tests completed!")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    test_type = sys.argv[1].lower()
    
    rclpy.init()
    
    try:
        tester = JoystickTester()
        
        if test_type == 'cyclone':
            tester.test_cyclone()
        elif test_type == 'doors':
            tester.test_doors()
        elif test_type == 'emergency':
            tester.test_emergency()
        elif test_type == 'status':
            tester.test_status()
        elif test_type == 'all':
            tester.test_all()
        else:
            print(f"Unknown test type: {test_type}")
            print(__doc__)
            sys.exit(1)
        
        # Keep the node alive briefly to ensure message is sent
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
