#!/usr/bin/env python3
"""
Arduino Serial Controller for Sucky Robot

This script provides a Python interface to communicate with the Arduino
controlling the cyclone and doors via USB serial connection.

Commands supported:
- ON: Turn cyclone ON
- OFF: Turn cyclone OFF
- DOOR_OPEN: Open doors
- DOOR_CLOSE: Close doors
- STATUS: Show system status
- HELP: Show available commands

Usage:
    python3 arduino_serial_controller.py [--port /dev/ttyACM0] [--interactive]
"""

import serial
import time
import argparse
import sys
import threading
from typing import Optional


class ArduinoController:
    """Class to handle serial communication with the Arduino."""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize the Arduino controller.
        
        Args:
            port: Serial port path (e.g., /dev/ttyACM0, /dev/ttyACM0)
            baudrate: Serial communication baud rate
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """
        Establish serial connection to Arduino.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            print(f"Connecting to Arduino on {self.port}...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Arduino resets when serial connection opens, wait for it to initialize
            print("Waiting for Arduino to initialize...")
            time.sleep(3)  # Increased wait time
            
            # Clear any initial boot messages
            if self.serial_conn.in_waiting > 0:
                self.serial_conn.read_all()
            
            # Mark as connected first, then test
            self.connected = True
            
            # Test connection by sending STATUS command
            print("Testing connection...")
            if self._test_connection():
                print(f"Successfully connected to Arduino on {self.port}")
                return True
            else:
                print(f"Failed to communicate with Arduino on {self.port}")
                self.disconnect()
                return False
                
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def _test_connection(self) -> bool:
        """
        Test the connection by sending a simple command.
        
        Returns:
            True if Arduino responds, False otherwise
        """
        try:
            # Send STATUS command directly without using send_command to avoid recursion
            command_bytes = "STATUS\n".encode('utf-8')
            self.serial_conn.write(command_bytes)
            self.serial_conn.flush()
            
            # Wait for response
            time.sleep(0.5)
            response = self.read_response(timeout=3.0)
            
            return len(response) > 0
            
        except Exception as e:
            print(f"Connection test failed: {e}")
            return False
    
    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.connected = False
        print("Disconnected from Arduino")
    
    def send_command(self, command: str) -> bool:
        """
        Send a command to the Arduino.
        
        Args:
            command: Command string to send
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected or not self.serial_conn:
            print("Error: Not connected to Arduino")
            return False
            
        try:
            # Send command with newline
            command_bytes = (command.strip() + '\n').encode('utf-8')
            self.serial_conn.write(command_bytes)
            self.serial_conn.flush()
            
            # Read response
            time.sleep(0.1)  # Give Arduino time to respond
            response = self.read_response()
            
            if response:
                print(response)
                return True
            else:
                print("No response from Arduino")
                return False
                
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def read_response(self, timeout: float = 2.0) -> str:
        """
        Read response from Arduino.
        
        Args:
            timeout: Maximum time to wait for response
            
        Returns:
            Response string from Arduino
        """
        if not self.connected or not self.serial_conn:
            return ""
            
        response_lines = []
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < timeout:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        response_lines.append(line)
                        # If we get a complete response (status block or single line), break
                        if line.startswith("===") and line.endswith("==="):
                            break
                        elif any(keyword in line.lower() for keyword in ['on', 'off', 'open', 'closed', 'error']):
                            # Single line response
                            time.sleep(0.1)  # Wait a bit more for any additional lines
                            break
                else:
                    time.sleep(0.01)
                    
            # Read any remaining lines
            while self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    response_lines.append(line)
                    
        except Exception as e:
            print(f"Error reading response: {e}")
            
        return '\n'.join(response_lines)
    
    def cyclone_on(self) -> bool:
        """Turn the cyclone on."""
        return self.send_command("ON")
    
    def cyclone_off(self) -> bool:
        """Turn the cyclone off."""
        return self.send_command("OFF")
    
    def doors_open(self) -> bool:
        """Open the doors."""
        return self.send_command("DOOR_OPEN")
    
    def doors_close(self) -> bool:
        """Close the doors."""
        return self.send_command("DOOR_CLOSE")
    
    def get_status(self) -> bool:
        """Get system status."""
        return self.send_command("STATUS")
    
    def get_help(self) -> bool:
        """Get available commands."""
        return self.send_command("HELP")


def find_arduino_port() -> Optional[str]:
    """
    Attempt to find the Arduino port automatically.
    
    Returns:
        Port path if found, None otherwise
    """
    import glob
    import os
    
    # Common Arduino ports on Linux
    possible_ports = [
        '/dev/ttyACM*',  # Most common for Arduino Uno/Nano
        '/dev/ttyUSB*',  # USB-to-serial adapters
        '/dev/ttyAMA*'   # Raspberry Pi serial
    ]
    
    for pattern in possible_ports:
        ports = glob.glob(pattern)
        for port in sorted(ports):  # Sort to get consistent ordering
            try:
                # Check if port exists and is accessible
                if os.path.exists(port):
                    # Try to open briefly without disturbing the Arduino too much
                    test_serial = serial.Serial(port, 115200, timeout=0.1)
                    time.sleep(0.1)  # Very brief wait
                    test_serial.close()
                    print(f"Found potential Arduino port: {port}")
                    return port
            except (serial.SerialException, PermissionError):
                continue
            except Exception:
                continue
                
    return None


def interactive_mode(controller: ArduinoController) -> None:
    """
    Run in interactive mode allowing user to send commands.
    
    Args:
        controller: Arduino controller instance
    """
    print("\n=== Arduino Interactive Mode ===")
    print("Available commands:")
    print("  on, off, open, close, status, help, quit")
    print("Or type any raw command to send to Arduino")
    print("================================\n")
    
    command_map = {
        'on': 'ON',
        'off': 'OFF',
        'open': 'DOOR_OPEN',
        'close': 'DOOR_CLOSE',
        'status': 'STATUS',
        'help': 'HELP'
    }
    
    try:
        while True:
            user_input = input("Arduino> ").strip().lower()
            
            if user_input in ['quit', 'exit', 'q']:
                break
            elif user_input == '':
                continue
            elif user_input in command_map:
                controller.send_command(command_map[user_input])
            else:
                # Send raw command
                controller.send_command(user_input.upper())
                
    except KeyboardInterrupt:
        print("\nExiting interactive mode...")
    except EOFError:
        print("\nExiting interactive mode...")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Arduino Serial Controller for Sucky Robot')
    parser.add_argument('--port', '-p', default=None, 
                       help='Serial port (e.g., /dev/ttyACM0). Auto-detect if not specified.')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='Serial baud rate (default: 115200)')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Run in interactive mode')
    parser.add_argument('--command', '-c', type=str,
                       help='Single command to send (ON, OFF, DOOR_OPEN, DOOR_CLOSE, STATUS, HELP)')
    parser.add_argument('--debug', '-d', action='store_true',
                       help='Enable debug output')
    parser.add_argument('--test-connection', '-t', action='store_true',
                       help='Just test the connection and exit')
    
    args = parser.parse_args()
    
    # Auto-detect port if not specified
    port = args.port
    if not port:
        if args.debug:
            print("Auto-detecting Arduino port...")
        port = find_arduino_port()
        if not port:
            print("Error: Could not find Arduino port. Please specify with --port")
            if args.debug:
                print("Try running 'ls /dev/tty*' to see available ports")
            sys.exit(1)
    
    if args.debug:
        print(f"Using port: {port}")
        print(f"Baud rate: {args.baudrate}")
    
    # Create controller and connect
    controller = ArduinoController(port=port, baudrate=args.baudrate)
    
    if not controller.connect():
        print("Failed to connect to Arduino")
        if args.debug:
            print("\nTroubleshooting tips:")
            print("1. Make sure Arduino is plugged in via USB")
            print("2. Check that Arduino is programmed with the correct firmware")
            print("3. Try closing any other programs using the serial port (Arduino IDE, PlatformIO monitor, etc.)")
            print("4. Try a different USB cable or port")
            print("5. Check port permissions: sudo usermod -a -G dialout $USER")
        sys.exit(1)
    
    try:
        if args.test_connection:
            # Just test connection and exit
            print("Connection test successful!")
        elif args.command:
            # Send single command and exit
            controller.send_command(args.command.upper())
        elif args.interactive:
            # Run interactive mode
            interactive_mode(controller)
        else:
            # Default: get status
            controller.get_status()
            
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
