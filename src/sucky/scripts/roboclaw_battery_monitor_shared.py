#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import serial
import struct
import time
import os
import fcntl
import errno
from threading import Lock

class RoboclawBatteryMonitorShared(Node):
    """
    Battery monitor that reads voltage directly from serial port
    with very low update rates to minimize interference with ROS2 control.
    """
    
    def __init__(self):
        super().__init__('roboclaw_battery_monitor_shared')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('address', 128)
        self.declare_parameter('publish_rate', 0.2)  # Very slow rate to minimize interference
        self.declare_parameter('timeout', 3.0)  # Longer timeout for shared access
        self.declare_parameter('retry_attempts', 2)  # Fewer retries to minimize interference
        self.declare_parameter('min_voltage', 22.0)  # V - low battery warning
        self.declare_parameter('max_voltage', 29.4)  # V - full battery (7S LiPo)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.address = self.get_parameter('address').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.retry_attempts = self.get_parameter('retry_attempts').get_parameter_value().integer_value
        self.min_voltage = self.get_parameter('min_voltage').get_parameter_value().double_value
        self.max_voltage = self.get_parameter('max_voltage').get_parameter_value().double_value
        
        # Publishers
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        
        # Battery state
        self.last_valid_voltage = None
        self.error_count = 0
        self.consecutive_errors = 0
        self.connection_retries = 0
        self.max_connection_retries = 5
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        
        # Timer for periodic reading
        self.timer = self.create_timer(1.0 / self.publish_rate, self.read_and_publish_battery_voltage)
        
        self.get_logger().info(f'Roboclaw battery monitor (shared) initialized at address {self.address}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz to minimize interference with ROS2 control')
        self.get_logger().info(f'Using serial port: {self.serial_port}')
    
    def connect_serial(self):
        """Initialize serial connection to roboclaw with shared access considerations"""
        try:
            # Close existing connection if any
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                
            # Try to open serial connection with shared access
            self.serial_conn = serial.Serial(
                self.serial_port, 
                115200,  # Standard roboclaw baud rate
                timeout=self.timeout,
                write_timeout=self.timeout,
                exclusive=False  # Allow shared access
            )
            
            # Wait for connection to stabilize
            time.sleep(0.5)  # Longer delay for shared access
            
            # Clear buffers
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.get_logger().info('Serial connection to roboclaw established (shared mode)')
            self.connection_retries = 0
            
        except (serial.SerialException, OSError) as e:
            self.connection_retries += 1
            if self.connection_retries <= self.max_connection_retries:
                self.get_logger().warning(f'Failed to connect to roboclaw (attempt {self.connection_retries}/{self.max_connection_retries}): {e}')
            else:
                self.get_logger().error(f'Failed to connect to roboclaw after {self.max_connection_retries} attempts: {e}')
            self.serial_conn = None
        except Exception as e:
            self.get_logger().error(f'Unexpected error connecting to roboclaw: {e}')
            self.serial_conn = None
    
    def calculate_crc16(self, data):
        """Calculate CRC16 for roboclaw communication"""
        crc = 0
        for byte in data:
            crc = crc ^ (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc = crc & 0xFFFF
        return crc
    def read_battery_voltage_shared(self):
        """Read battery voltage with shared serial access considerations"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.connect_serial()
            if not self.serial_conn:
                return None
        
        # Add delay before attempting communication to be respectful of ROS2 control
        time.sleep(0.1)
        
        for attempt in range(self.retry_attempts):
            try:
                with self.serial_lock:
                    # Wait between attempts
                    if attempt > 0:
                        time.sleep(0.2)
                    
                    # Clear input buffer before sending command
                    self.serial_conn.reset_input_buffer()
                    
                    # Command 24: Read Main Battery Voltage
                    command = [self.address, 24]  # READ_MAIN_BATT_VOLT = 24
                    
                    # Calculate and append CRC
                    crc = self.calculate_crc16(command)
                    command.extend([crc >> 8, crc & 0xFF])
                    
                    # Send command
                    self.serial_conn.write(bytes(command))
                    self.serial_conn.flush()  # Ensure data is sent
                    
                    # Read response (2 bytes for voltage + 2 bytes for CRC)
                    response = self.serial_conn.read(4)
                    
                    if len(response) != 4:
                        if attempt == self.retry_attempts - 1:
                            self.get_logger().debug(f'Incomplete response from roboclaw: got {len(response)} bytes, expected 4')
                        continue
                    
                    # Extract voltage (first 2 bytes, big-endian)
                    voltage_raw = struct.unpack('>H', response[:2])[0]
                    
                    # Verify CRC
                    expected_crc = struct.unpack('>H', response[2:4])[0]
                    calculated_crc = self.calculate_crc16(command[:2] + list(response[:2]))
                    
                    if expected_crc != calculated_crc:
                        if attempt == self.retry_attempts - 1:
                            self.get_logger().debug('CRC mismatch in roboclaw response')
                        continue
                    
                    # Convert to voltage (roboclaw returns voltage in 10ths of a volt)
                    voltage = voltage_raw / 10.0
                    
                    # Sanity check voltage reading
                    if 10.0 <= voltage <= 50.0:  # Reasonable voltage range
                        self.consecutive_errors = 0
                        return voltage
                    else:
                        if attempt == self.retry_attempts - 1:
                            self.get_logger().warning(f'Unreasonable voltage reading: {voltage}V')
                        continue
                        
            except serial.SerialTimeoutError:
                if attempt == self.retry_attempts - 1:
                    self.get_logger().debug('Timeout reading from roboclaw (shared mode)')
            except (serial.SerialException, OSError) as e:
                if attempt == self.retry_attempts - 1:
                    self.get_logger().warning(f'Serial error reading battery voltage: {e}')
                # Try to reconnect on serial errors
                self.connect_serial()
            except Exception as e:
                if attempt == self.retry_attempts - 1:
                    self.get_logger().error(f'Unexpected error reading battery voltage: {e}')
        
        # If we get here, all attempts failed
        self.consecutive_errors += 1
        self.error_count += 1
        return None
    def read_and_publish_battery_voltage(self):
        """Read battery voltage and publish to topics"""
        voltage = self.read_battery_voltage_shared()
        
        if voltage is not None:
            self.last_valid_voltage = voltage
            
            # Publish voltage
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_pub.publish(voltage_msg)
            
            # Publish diagnostics
            self.publish_diagnostics(voltage, healthy=True)
            
            # Log occasionally
            if hasattr(self, '_last_log_time'):
                if time.time() - self._last_log_time > 60.0:  # Log every minute
                    self.get_logger().info(f'Battery voltage: {voltage:.2f}V')
                    self._last_log_time = time.time()
            else:
                self._last_log_time = time.time()
                
        else:
            # Publish error diagnostics
            self.publish_diagnostics(self.last_valid_voltage, healthy=False)
            
            # Log errors occasionally to avoid spam
            if self.consecutive_errors % 50 == 1:  # Log every 50th consecutive error
                self.get_logger().warning(f'Failed to read battery voltage (consecutive errors: {self.consecutive_errors})')
    
    def publish_diagnostics(self, voltage, healthy=True):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        diag_status = DiagnosticStatus()
        diag_status.name = 'roboclaw_battery_shared'
        diag_status.hardware_id = f'roboclaw_address_{self.address}'
        
        if not healthy:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = f'Failed to read battery voltage (consecutive errors: {self.consecutive_errors})'
            if self.last_valid_voltage is not None:
                diag_status.message += f' - Last valid reading: {self.last_valid_voltage:.2f}V'
        elif voltage is None:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = 'No battery voltage reading available'
        elif voltage < self.min_voltage:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = f'Low battery voltage: {voltage:.2f}V'
        elif voltage > self.max_voltage:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = f'High battery voltage: {voltage:.2f}V'
        else:
            diag_status.level = DiagnosticStatus.OK
            diag_status.message = f'Battery voltage normal: {voltage:.2f}V'
        
        # Add voltage value
        if voltage is not None:
            voltage_kv = KeyValue()
            voltage_kv.key = 'voltage'
            voltage_kv.value = f'{voltage:.2f}'
            diag_status.values.append(voltage_kv)
            
            # Add battery percentage (rough estimate)
            voltage_range = self.max_voltage - self.min_voltage
            percentage = max(0, min(100, ((voltage - self.min_voltage) / voltage_range) * 100))
            percentage_kv = KeyValue()
            percentage_kv.key = 'percentage'
            percentage_kv.value = f'{percentage:.1f}'
            diag_status.values.append(percentage_kv)
        
        # Add error statistics
        error_count_kv = KeyValue()
        error_count_kv.key = 'total_errors'
        error_count_kv.value = str(self.error_count)
        diag_status.values.append(error_count_kv)
        
        consecutive_errors_kv = KeyValue()
        consecutive_errors_kv.key = 'consecutive_errors'
        consecutive_errors_kv.value = str(self.consecutive_errors)
        diag_status.values.append(consecutive_errors_kv)
        
        diag_array.status.append(diag_status)
        self.diagnostics_pub.publish(diag_array)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        battery_monitor = RoboclawBatteryMonitorShared()
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'battery_monitor' in locals():
            battery_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
