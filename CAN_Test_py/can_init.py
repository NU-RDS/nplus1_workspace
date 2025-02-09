import can
import struct
import time

def setup_can():
    bus = can.Bus(
        interface='slcan',
        channel='/dev/ttyACM0',
        bitrate=250000,
        ttyBaudrate=115200
    )
    print("Bus created successfully")
    return bus

def send_odrive_command(bus, node_id, command_id, data=None):
    """Send a command to ODrive and print detailed debug info"""
    arbitration_id = (node_id << 5 | command_id)
    msg = can.Message(
        arbitration_id=arbitration_id,
        data=data if data else [0] * 8,
        is_extended_id=False
    )
    print(f"Sending message: ID={hex(arbitration_id)}, data={[hex(b) for b in msg.data]}")
    bus.send(msg)

def main():
    try:
        bus = setup_can()
        node_id = 0
        
        # 1. Clear any pending messages
        print("\nClearing message queue...")
        while bus.recv(timeout=0):
            pass
        
        # 2. Request ODrive state
        print("\nRequesting axis state...")
        send_odrive_command(bus, node_id, 0x07, struct.pack('<I', 1))  # Request AXIS_STATE_IDLE
        
        # 3. Wait for heartbeat messages
        print("\nWaiting for responses...")
        start_time = time.time()
        while time.time() - start_time < 5:  # Listen for 5 seconds
            msg = bus.recv(timeout=0.1)
            if msg is not None:
                print(f"Received message:")
                print(f"  ID: {hex(msg.arbitration_id)}")
                print(f"  Data: {[hex(b) for b in msg.data]}")
                if msg.arbitration_id == (node_id << 5 | 0x01):  # Heartbeat
                    error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                    print(f"  Decoded: Error={error}, State={state}, Result={result}")
            else:
                print(".", end='', flush=True)
        
        print("\n\nTest complete")
        bus.shutdown()
        
    except Exception as e:
        print(f"Error: {e}")
        try:
            bus.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()