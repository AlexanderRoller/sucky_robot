"""
Set the PLC clock - Production version

Sets the ControlLogix PLC clock to the same time as your computer
"""
from pylogix import PLC
import datetime

def set_plc_time(ip_address='10.41.235.6', slot=0, timeout=10.0):
    """
    Set the PLC time to match the system time
    
    Args:
        ip_address: PLC IP address
        slot: Processor slot (typically 0 for ControlLogix)
        timeout: Connection timeout in seconds
    
    Returns:
        True if successful, False otherwise
    """
    with PLC() as comm:
        comm.IPAddress = ip_address
        comm.ProcessorSlot = slot
        comm.SocketTimeout = timeout
        
        current_time = datetime.datetime.now()
        print(f"Setting PLC time to: {current_time}")
        
        ret = comm.SetPLCTime()
        
        if ret.Status == 'Success':
            print("PLC time successfully set!")
            
            # Verify by reading back
            verify_ret = comm.GetPLCTime()
            if verify_ret.Status == 'Success' and verify_ret.Value:
                print(f"Verified PLC time: {verify_ret.Value}")
                return True
            else:
                print("Could not verify time setting")
                return False
        else:
            print(f"Failed to set PLC time: {ret.Status}")
            return False

if __name__ == "__main__":
    success = set_plc_time()
    if not success:
        print("Time setting failed")
