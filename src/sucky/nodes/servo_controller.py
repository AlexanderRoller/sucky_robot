#!/usr/bin/env python3

"""
ROS2 Node for controlling door servos via Arduino over serial.

This node provides ROS2 services to open and close the doors connected to servo motors
controlled by an Arduino. The Arduino code must be updated to handle door commands.

Published Topics:
  - /doors/status (std_msgs/Bool): Current door state (True=OPEN, False=CLOSED)

Services:
  - /doors/set_state (std_srvs/SetBool): Open (True) or close (False) the doors
  - /doors/get_status (std_srvs/Trigger): Get current door status

Parameters:
  - serial_port (string): Arduino serial port (default: /dev/ttyACM0)
  - baud_rate (int): Serial communication baud rate (default: 9600)
  - timeout (double): Serial communication timeout (default: 2.0)

Arduino Commands:
  - DOOR_OPEN: Opens both doors (left servo to 180°, right servo to 0°)
  - DOOR_CLOSE: Closes both doors (left servo to 0°, right servo to 180°)
  - STATUS: Returns status of both cyclone and doors

Usage:
  ros2 run sucky servo_controller.py
  
  or with parameters:
  ros2 run sucky servo_controller.py --ros-args -p serial_port:=/dev/ttyACM0

Test the node:
  ros2 run sucky test_servo.py open
  ros2 run sucky test_servo.py close
  ros2 run sucky test_servo.py status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import serial
import time


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 2.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Serial connection and state
        self.serial_conn = None
        self.doors_open = False
        
        # Publishers
        self.status_pub = self.create_publisher(Bool, 'doors/status', 10)
        
        # Services
        self.set_doors_srv = self.create_service(
            SetBool, 'doors/set_state', self.set_doors_callback)
        self.get_status_srv = self.create_service(
            Trigger, 'doors/get_status', self.get_status_callback)
        
        # Status publishing timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Initialize serial connection
        self.connect_serial()
        
        self.get_logger().info(f"Servo controller ready on {self.serial_port}")

    def connect_serial(self):
        """Establish serial connection to Arduino"""
        try:
            self.get_logger().info(f"Connecting to Arduino on {self.serial_port}...")
            
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            self.get_logger().info("Serial port opened, waiting for Arduino initialization...")
            time.sleep(3.0)  # Wait for Arduino to be ready
            
            # Clear any pending data
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            # Get initial status to sync state
            status_response = self.send_command("STATUS")
            self.get_logger().info(f"Initial status response from Arduino: '{status_response}'")
            
            if status_response and "Doors: OPEN" in status_response:
                self.doors_open = True
                self.get_logger().info("Parsed door state as OPEN")
            elif status_response and "Doors: CLOSED" in status_response:
                self.doors_open = False
                self.get_logger().info("Parsed door state as CLOSED")
            else:
                self.get_logger().warn(f"Could not parse Arduino door state from: '{status_response}'")
                # Try legacy parsing for backward compatibility
                if status_response and "Door status: OPEN" in status_response:
                    self.doors_open = True
                    self.get_logger().info("Parsed door state as OPEN (legacy format)")
                elif status_response and "Door status: CLOSED" in status_response:
                    self.doors_open = False
                    self.get_logger().info("Parsed door state as CLOSED (legacy format)")
            
            self.get_logger().info(f"Connected successfully, initial door state: {'OPEN' if self.doors_open else 'CLOSED'}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            return False

    def send_command(self, command):
        """Send command to Arduino and return response"""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.get_logger().error("Serial connection not available")
                return None
            
            self.get_logger().debug(f"Sending command: {command}")
            
            # Clear buffers to ensure clean communication
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            # Send command
            cmd = f"{command}\n"
            self.serial_conn.write(cmd.encode())
            self.serial_conn.flush()
            
            # Wait for response
            time.sleep(0.5)
            
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode().strip()
                self.get_logger().debug(f"Arduino response: {response}")
                
                # For STATUS command, Arduino sends multiple lines - consume all of them
                if command == "STATUS":
                    all_response = [response]
                    while self.serial_conn.in_waiting > 0:
                        additional_line = self.serial_conn.readline().decode().strip()
                        if additional_line:
                            all_response.append(additional_line)
                            self.get_logger().debug(f"Arduino additional response: {additional_line}")
                    # Return the full response for parsing
                    return "\n".join(all_response)
                
                return response
            else:
                self.get_logger().warn(f"No response from Arduino for command: {command}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return None

    def set_doors_callback(self, request, response):
        """Service callback to set door state"""
        self.get_logger().info(f"Setting doors to: {'OPEN' if request.data else 'CLOSED'}")
        
        try:
            command = "DOOR_OPEN" if request.data else "DOOR_CLOSE"
            result = self.send_command(command)
            
            expected_response = "Doors OPEN" if request.data else "Doors CLOSED"
            if result is not None and result.upper() == expected_response.upper():
                self.doors_open = request.data
                response.success = True
                response.message = f"Doors {'OPENED' if request.data else 'CLOSED'}"
                self.get_logger().info(f"Success: {response.message}")
            else:
                response.success = False
                response.message = f"Failed to communicate with Arduino or unexpected response: {result}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_status_callback(self, request, response):
        """Service callback to get door status"""
        self.get_logger().info("Door status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result is not None:
                # Update internal state based on Arduino response
                if "Doors: OPEN" in result:
                    self.doors_open = True
                elif "Doors: CLOSED" in result:
                    self.doors_open = False
                # Legacy format support
                elif "Door status: OPEN" in result:
                    self.doors_open = True
                elif "Door status: CLOSED" in result:
                    self.doors_open = False
                
                response.success = True
                response.message = f"Doors are {'OPEN' if self.doors_open else 'CLOSED'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def publish_status(self):
        """Publish current door status"""
        msg = Bool()
        msg.data = self.doors_open
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown"""
        # Close doors before shutdown for safety
        if self.doors_open:
            self.get_logger().info("Closing doors before shutdown...")
            self.send_command("DOOR_CLOSE")
        
        # Close serial connection
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        servo_controller = ServoController()
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'servo_controller' in locals():
            servo_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
