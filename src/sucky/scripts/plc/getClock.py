"""
Get the PLC time - Production version

Returns datetime.datetime type from ControlLogix PLC
"""
from pylogix import PLC

def get_plc_time(ip_address='10.41.235.6', slot=0, timeout=10.0):
    """
    Get the current time from the PLC
    
    Args:
        ip_address: PLC IP address
        slot: Processor slot (typically 0 for ControlLogix)
        timeout: Connection timeout in seconds
    
    Returns:
        datetime.datetime object or None if failed
    """
    with PLC() as comm:
        comm.IPAddress = ip_address
        comm.ProcessorSlot = slot
        comm.SocketTimeout = timeout
        
        ret = comm.GetPLCTime()
        
        if ret.Status == 'Success' and ret.Value is not None:
            return ret.Value
        else:
            print(f"Error getting PLC time: {ret.Status}")
            return None

if __name__ == "__main__":
    # Get PLC time
    plc_time = get_plc_time()
    
    if plc_time:
        print(f"PLC Time: {plc_time}")
        print(f"Year: {plc_time.year}")
        print(f"Month: {plc_time.month}")
        print(f"Day: {plc_time.day}")
        print(f"Hour: {plc_time.hour}")
        print(f"Minute: {plc_time.minute}")
        print(f"Second: {plc_time.second}")
        print(f"Microsecond: {plc_time.microsecond}")
    else:
        print("Failed to retrieve PLC time")
