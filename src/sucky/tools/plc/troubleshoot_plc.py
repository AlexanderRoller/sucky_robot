"""
Comprehensive PLC troubleshooting script
Tests various connection parameters and provides detailed diagnostics
"""
from pylogix import PLC
import socket
import subprocess
import sys

def test_network_connectivity(ip_address):
    """Test basic network connectivity"""
    print(f"=== Network Connectivity Test ===")
    print(f"Testing ping to {ip_address}...")
    
    try:
        result = subprocess.run(['ping', '-c', '4', ip_address], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✓ Ping successful")
            return True
        else:
            print("✗ Ping failed")
            print(result.stderr)
            return False
    except subprocess.TimeoutExpired:
        print("✗ Ping timed out")
        return False
    except Exception as e:
        print(f"✗ Ping error: {e}")
        return False

def test_port_connectivity(ip_address, port=44818):
    """Test if the EtherNet/IP port is open"""
    print(f"\n=== Port Connectivity Test ===")
    print(f"Testing connection to {ip_address}:{port} (EtherNet/IP port)...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((ip_address, port))
        sock.close()
        
        if result == 0:
            print(f"✓ Port {port} is open")
            return True
        else:
            print(f"✗ Port {port} is closed or filtered")
            return False
    except Exception as e:
        print(f"✗ Port test error: {e}")
        return False

def test_plc_connection_variants(ip_address):
    """Test PLC connection with different parameters"""
    print(f"\n=== PLC Connection Tests ===")
    
    # Test different slot numbers (common values: 0, 1, 2)
    slots_to_test = [0, 1, 2]
    
    # Test different processor types
    processor_types = [None, 'CompactLogix', 'ControlLogix', 'Micro800', 'SLC', 'PLC5']
    
    for slot in slots_to_test:
        for proc_type in processor_types:
            print(f"\nTesting Slot: {slot}, Processor: {proc_type or 'Auto-detect'}")
            
            try:
                with PLC() as comm:
                    comm.IPAddress = ip_address
                    comm.ProcessorSlot = slot
                    comm.SocketTimeout = 10.0
                    
                    if proc_type:
                        # Note: pylogix doesn't have a direct ProcessorType property
                        # but we can try different approaches
                        pass
                    
                    # Try to read system information
                    ret = comm.GetPLCTime()
                    print(f"  GetPLCTime result: {ret.Status}")
                    
                    if ret.Status == 'Success':
                        print(f"  ✓ SUCCESS! PLC Time: {ret.Value}")
                        return True, slot, proc_type
                    
                    # Try reading a basic tag
                    ret2 = comm.Read('$ConnectionType')  # System tag
                    print(f"  System tag read result: {ret2.Status}")
                    
                    # Try discovering tags
                    ret3 = comm.GetTagList()
                    print(f"  Tag discovery result: {ret3.Status}")
                    if ret3.Status == 'Success' and ret3.Value:
                        print(f"  Found {len(ret3.Value)} tags")
                        
            except Exception as e:
                print(f"  Exception: {e}")
    
    return False, None, None

def test_alternative_ports():
    """Test other common industrial protocol ports"""
    print(f"\n=== Alternative Port Tests ===")
    
    common_ports = {
        44818: "EtherNet/IP",
        102: "S7 Communication",
        502: "Modbus TCP",
        20000: "AB Ethernet/IP",
        2222: "EtherNet/IP explicit",
        1470: "Allen-Bradley DF1"
    }
    
    ip_address = '10.41.235.6'
    
    for port, protocol in common_ports.items():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((ip_address, port))
            sock.close()
            
            if result == 0:
                print(f"✓ Port {port} ({protocol}) is open")
            else:
                print(f"✗ Port {port} ({protocol}) is closed")
        except Exception as e:
            print(f"✗ Port {port} test error: {e}")

def main():
    ip_address = '10.41.235.6'
    
    print("PLC Connection Troubleshooting Tool")
    print("=" * 40)
    
    # Test 1: Network connectivity
    ping_success = test_network_connectivity(ip_address)
    
    # Test 2: Port connectivity  
    port_success = test_port_connectivity(ip_address)
    
    # Test 3: Alternative ports
    test_alternative_ports()
    
    # Test 4: PLC connection variants
    if ping_success:
        success, working_slot, working_type = test_plc_connection_variants(ip_address)
        
        if success:
            print(f"\n🎉 SUCCESS! Working configuration:")
            print(f"   IP: {ip_address}")
            print(f"   Slot: {working_slot}")
            print(f"   Processor Type: {working_type or 'Auto-detect'}")
        else:
            print(f"\n❌ No working configuration found")
            print(f"\nAdditional troubleshooting steps:")
            print(f"1. Check PLC manual for correct slot number")
            print(f"2. Verify PLC is in RUN mode")
            print(f"3. Check if PLC supports EtherNet/IP")
            print(f"4. Verify network subnet and routing")
            print(f"5. Check for VPN or firewall interference")
            print(f"6. Try using RSLinx or other AB software to test connection")
    else:
        print(f"\n❌ Basic network connectivity failed")
        print(f"Check network configuration and PLC power/status")

if __name__ == "__main__":
    main()
