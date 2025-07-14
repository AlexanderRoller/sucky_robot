#!/usr/bin/env python3

"""
ROS2 Node for controlling cyclone vacuum via Arduino over serial.
Provides services to turn the cyclone on and off.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import serial
import time


class CycloneController(Node):
    def __init__(self):
        super().__init__('cyclone_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 2.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Serial connection and state
        self.serial_conn = None
        self.cyclone_state = False
        
        # Publishers
        self.status_pub = self.create_publisher(Bool, 'cyclone/status', 10)
        
        # Services
        self.set_cyclone_srv = self.create_service(
            SetBool, 'cyclone/set_state', self.set_cyclone_callback)
        self.get_status_srv = self.create_service(
            Trigger, 'cyclone/get_status', self.get_status_callback)
        
        # Status publishing timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Initialize serial connection
        self.connect_serial()
        
        self.get_logger().info(f"Cyclone controller ready on {self.serial_port}")

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
            
            if status_response and "Cyclone: ON" in status_response:
                self.cyclone_state = True
                self.get_logger().info("Parsed cyclone state as ON")
            elif status_response and "Cyclone: OFF" in status_response:
                self.cyclone_state = False
                self.get_logger().info("Parsed cyclone state as OFF")
            else:
                self.get_logger().warn(f"Could not parse Arduino cyclone state from: '{status_response}'")
                # Try legacy parsing for backward compatibility
                if status_response and ": ON" in status_response.upper():
                    self.cyclone_state = True
                    self.get_logger().info("Parsed state as ON (legacy format)")
                elif status_response and ": OFF" in status_response.upper():
                    self.cyclone_state = False
                    self.get_logger().info("Parsed state as OFF (legacy format)")
            
            self.get_logger().info(f"Connected successfully, initial cyclone state: {'ON' if self.cyclone_state else 'OFF'}")
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

    def set_cyclone_callback(self, request, response):
        """Service callback to set cyclone state"""
        self.get_logger().info(f"Setting cyclone to: {'ON' if request.data else 'OFF'}")
        
        try:
            command = "ON" if request.data else "OFF"
            result = self.send_command(command)
            
            if result is not None and ((request.data and result.upper() == "CYCLONE ON") or (not request.data and result.upper() == "CYCLONE OFF")):
                self.cyclone_state = request.data
                response.success = True
                response.message = f"Cyclone turned {'ON' if request.data else 'OFF'}"
                self.get_logger().info(f"Success: {response.message}")
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino or unexpected response"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_status_callback(self, request, response):
        """Service callback to get cyclone status"""
        self.get_logger().info("Status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result is not None:
                # Update internal state based on Arduino response
                if "Cyclone: ON" in result:
                    self.cyclone_state = True
                elif "Cyclone: OFF" in result:
                    self.cyclone_state = False
                # Legacy format support
                elif ": ON" in result.upper():
                    self.cyclone_state = True
                elif ": OFF" in result.upper():
                    self.cyclone_state = False
                
                response.success = True
                response.message = f"Cyclone is {'ON' if self.cyclone_state else 'OFF'}"
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
        """Publish current cyclone status"""
        msg = Bool()
        msg.data = self.cyclone_state
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown"""
        # Turn off cyclone before shutdown for safety
        if self.cyclone_state:
            self.get_logger().info("Turning off cyclone before shutdown...")
            self.send_command("OFF")
        
        # Close serial connection
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        cyclone_controller = CycloneController()
        rclpy.spin(cyclone_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'cyclone_controller' in locals():
            cyclone_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
