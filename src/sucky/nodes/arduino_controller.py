#!/usr/bin/env python3

"""
ROS2 Node for controlling both cyclone vacuum and door servos via Arduino over serial.

This unified controller handles both cyclone and door operations through a single
serial connection to the Arduino, preventing communication conflicts that occur
when multiple nodes try to access the same serial port.

Published Topics:
  - /cyclone/status (std_msgs/Bool): Current cyclone state (True=ON, False=OFF)
  - /doors/status (std_msgs/Bool): Current door state (True=OPEN, False=CLOSED)

Services:
  - /cyclone/set_state (std_srvs/SetBool): Turn cyclone on (True) or off (False)
  - /cyclone/get_status (std_srvs/Trigger): Get current cyclone status
  - /doors/set_state (std_srvs/SetBool): Open (True) or close (False) the doors
  - /doors/get_status (std_srvs/Trigger): Get current door status

Parameters:
  - serial_port (string): Arduino serial port (default: /dev/ttyACM0)
  - baud_rate (int): Serial communication baud rate (default: 115200)
  - timeout (double): Serial communication timeout (default: 2.0)

Arduino Commands:
  - ON: Turn cyclone on
  - OFF: Turn cyclone off
  - DOOR_OPEN: Opens both doors (left servo to 0°, right servo to 180°)
  - DOOR_CLOSE: Closes both doors (left servo to 180°, right servo to 0°)
  - STATUS: Returns status of both cyclone and doors

Usage:
  ros2 run sucky arduino_controller.py
  
  or with parameters:
  ros2 run sucky arduino_controller.py --ros-args -p serial_port:=/dev/ttyACM0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import serial
import time
import threading


class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
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
        self.doors_open = False
        self.serial_lock = threading.Lock()  # Prevent concurrent serial access
        
        # Publishers
        self.cyclone_status_pub = self.create_publisher(Bool, 'cyclone/status', 10)
        self.doors_status_pub = self.create_publisher(Bool, 'doors/status', 10)
        
        # Cyclone services
        self.set_cyclone_srv = self.create_service(
            SetBool, 'cyclone/set_state', self.set_cyclone_callback)
        self.get_cyclone_status_srv = self.create_service(
            Trigger, 'cyclone/get_status', self.get_cyclone_status_callback)
        
        # Door services
        self.set_doors_srv = self.create_service(
            SetBool, 'doors/set_state', self.set_doors_callback)
        self.get_doors_status_srv = self.create_service(
            Trigger, 'doors/get_status', self.get_doors_status_callback)
        
        # Status publishing timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Initialize serial connection
        self.connect_serial()
        
        self.get_logger().info(f"Arduino controller ready on {self.serial_port}")
        self.get_logger().info("Controlling both cyclone and doors through unified interface")

    def connect_serial(self):
        """Establish serial connection to Arduino"""
        try:
            self.get_logger().info(f"Connecting to Arduino on {self.serial_port}...")
            
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                
                self.get_logger().info("Serial port opened, waiting for Arduino initialization...")
                
                # Wait for Arduino ready signal
                ready_received = False
                start_time = time.time()
                while not ready_received and (time.time() - start_time) < 10.0:
                    if self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode().strip()
                            if line == "ARDUINO_READY":
                                ready_received = True
                                self.get_logger().info("Arduino ready signal received")
                            elif line:
                                self.get_logger().debug(f"Arduino startup: {line}")
                        except UnicodeDecodeError:
                            pass
                    time.sleep(0.1)
                
                if not ready_received:
                    self.get_logger().warn("Arduino ready signal not received, proceeding anyway")
                
                # Get initial status
                status_response = self.send_command_unsafe("STATUS")
                if status_response:
                    self.get_logger().info("Initial status received:")
                    for line in status_response.split('\n'):
                        if line.strip():
                            self.get_logger().info(f"  {line.strip()}")
                    self.parse_status_response(status_response)
                else:
                    self.get_logger().info("No initial status received, using defaults")
                
                self.get_logger().info("Connected successfully!")
                self.get_logger().info(f"  Initial cyclone state: {'ON' if self.cyclone_state else 'OFF'}")
                self.get_logger().info(f"  Initial door state: {'OPEN' if self.doors_open else 'CLOSED'}")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            return False

    def send_command_unsafe(self, command):
        """Send command to Arduino without lock (for internal use when lock is already held)"""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.get_logger().error("Serial connection not available")
                return None
            
            self.get_logger().debug(f"Sending command: {command}")
            
            # Send command
            cmd = f"{command}\n"
            self.serial_conn.write(cmd.encode())
            self.serial_conn.flush()
            
            # Wait for response
            time.sleep(0.5)
            
            response_lines = []
            start_time = time.time()
            
            # Read response with timeout
            while (time.time() - start_time) < 2.0:
                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode().strip()
                        if line:
                            response_lines.append(line)
                            self.get_logger().debug(f"Arduino response: {line}")
                    except UnicodeDecodeError as e:
                        self.get_logger().warn(f"Failed to decode Arduino response: {e}")
                        continue
                else:
                    # No more data available, wait a bit and check again
                    time.sleep(0.1)
                    if self.serial_conn.in_waiting == 0:
                        break
            
            if response_lines:
                return "\n".join(response_lines)
            else:
                self.get_logger().warn(f"No response from Arduino for command: {command}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return None

    def send_command(self, command):
        """Send command to Arduino with thread safety"""
        with self.serial_lock:
            return self.send_command_unsafe(command)

    def parse_status_response(self, response):
        """Parse Arduino STATUS response and update internal state"""
        if not response:
            self.get_logger().debug("No status response to parse - using default states")
            return
        
        # Parse cyclone state
        if "Cyclone: ON" in response:
            self.cyclone_state = True
        elif "Cyclone: OFF" in response:
            self.cyclone_state = False
        
        # Parse door state  
        if "Doors: OPEN" in response:
            self.doors_open = True
        elif "Doors: CLOSED" in response:
            self.doors_open = False

    def set_cyclone_callback(self, request, response):
        """Service callback to set cyclone state"""
        self.get_logger().info(f"Setting cyclone to: {'ON' if request.data else 'OFF'}")
        
        try:
            command = "ON" if request.data else "OFF"
            result = self.send_command(command)
            
            if result:
                # Check for success in response
                if request.data and ("Cyclone ON" in result or "Cyclone already ON" in result):
                    self.cyclone_state = True
                    response.success = True
                    response.message = "Cyclone turned ON"
                elif not request.data and ("Cyclone OFF" in result or "Cyclone already OFF" in result):
                    self.cyclone_state = False
                    response.success = True
                    response.message = "Cyclone turned OFF"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino"
                
            self.get_logger().info(f"Cyclone control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def set_doors_callback(self, request, response):
        """Service callback to set door state"""
        self.get_logger().info(f"Setting doors to: {'OPEN' if request.data else 'CLOSED'}")
        
        try:
            command = "DOOR_OPEN" if request.data else "DOOR_CLOSE"
            result = self.send_command(command)
            
            if result:
                # Check for success in response - look for completion messages
                if request.data and ("Doors OPEN" in result or "opening" in result or "Doors already OPEN" in result):
                    self.doors_open = True
                    response.success = True
                    response.message = "Doors OPENED"
                elif not request.data and ("Doors CLOSED" in result or "closing" in result or "Doors already CLOSED" in result):
                    self.doors_open = False
                    response.success = True
                    response.message = "Doors CLOSED"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino"
                
            self.get_logger().info(f"Door control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_cyclone_status_callback(self, request, response):
        """Service callback to get cyclone status"""
        self.get_logger().info("Cyclone status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
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

    def get_doors_status_callback(self, request, response):
        """Service callback to get door status"""
        self.get_logger().info("Door status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
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
        """Publish current status for both cyclone and doors"""
        # Publish cyclone status
        cyclone_msg = Bool()
        cyclone_msg.data = self.cyclone_state
        self.cyclone_status_pub.publish(cyclone_msg)
        
        # Publish door status
        doors_msg = Bool()
        doors_msg.data = self.doors_open
        self.doors_status_pub.publish(doors_msg)

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Arduino controller...")
        
        with self.serial_lock:
            # Turn off cyclone and close doors before shutdown for safety
            if self.cyclone_state:
                self.get_logger().info("Turning off cyclone before shutdown...")
                self.send_command_unsafe("OFF")
            
            if self.doors_open:
                self.get_logger().info("Closing doors before shutdown...")
                self.send_command_unsafe("DOOR_CLOSE")
            
            # Close serial connection
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.get_logger().info("Serial connection closed")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        arduino_controller = ArduinoController()
        rclpy.spin(arduino_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'arduino_controller' in locals():
            arduino_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
