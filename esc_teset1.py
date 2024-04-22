import time
from pymavlink import mavutil

# Function to request ESC telemetry data
def request_esc_telemetry():
    # Connect to the autopilot (change the connection string as needed)
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

    # Wait for heartbeat from the autopilot
    master.wait_heartbeat()

    # Send a command to request ESC telemetry data
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command to request message
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4,  # Message ID for ESC telemetry data
        0,  # Param 1 (0 for request)
        0,  # Param 2 (0 for request)
        0,  # Param 3 (0 for request)
        0,  # Param 4 (0 for request)
        0,  # Param 5 (0 for request)
        0,  # Param 6 (0 for request)
        0   # Param 7 (0 for request)
    )

    # Wait for the response
    while True:
        # Wait for the next message
        msg = master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=True, timeout=1.0)
        if msg:
            # ESC telemetry data received, print it
            print("Temperature:", msg.temperature)
            print("Voltage:", msg.voltage)
            print("Current:", msg.current)
            print("Total current:", msg.totalcurrent)
            print("RPM:", msg.rpm)
            print("Count:", msg.count)

# Main function
def main():
    while True:
        request_esc_telemetry()
        time.sleep(1)  # Wait for 1 second before requesting telemetry data again

# Entry point of the script
if __name__ == "__main__":
    main()
