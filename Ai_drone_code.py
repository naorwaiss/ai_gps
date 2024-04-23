import time
from pymavlink import mavutil
import csv

class AI_drone_naor:
    # 'udp:127.0.0.1:14550' - sitl string
    #'/dev/ttyAMA0' -real string -- baudrate - 57600
    def __init__(self, connection_string='/dev/ttyAMA0', baudrate=57600, csv_file_path='/home/drone/flight_data/flight_data.csv'):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
        self.satellite_count = None
        self.hdop = None
        self.csv_file_path = csv_file_path

    def wait_heartbeat(self):
        self.master.wait_heartbeat()

    def check_gps_status(self):
        while True:
            msg_GPS = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg_GPS is not None:
                self.satellite_count = msg_GPS.satellites_visible
                self.hdop = msg_GPS.eph / 100.0  # Convert to meters

                if self.satellite_count > 7 and self.hdop < 5:
                    return True  # Conditions met, return True
                else:
                    print(f"Satellite count: {self.satellite_count}, HDOP: {self.hdop}")
                    time.sleep(3)

    def request_esc_telemetry(self):
        # Send a command to request ESC telemetry data
        self.master.mav.command_long_send(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command to request message
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4,  # Message ID for ESC telemetry data
            0,  # Param 1 (0 for request)
            0,  # Param 2 (0 for request)
            0,  # Param 3 (0 for request)
            0,  # Param 4 (0 for request)
            0,  # Param 5 (0 for request)
            0,  # Param 6 (0 for request)
            0  # Param 7 (0 for request)
        )

    def receive_esc_telemetry(self):
        # Wait for the next message
        msg = self.master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=True, timeout=1.0)
        if msg:
            # ESC telemetry data received, return it
            return {
                "Temperature": msg.temperature,
                "Voltage": msg.voltage,
                "Current": msg.current,
                "Total current": msg.totalcurrent,
                "RPM": msg.rpm
            }
        else:
            return None




    def compass_telemetry(self):
        msg_compass = self.master.recv_match(type='COMPASS', blocking=True)


    def run(self):
        self.wait_heartbeat() # Connect to the drone

        # Check the GPS status - not take data until got all the GPS
        gps_status = self.check_gps_status()
        if not gps_status:
            print("GPS status is not satisfactory. Exiting.")
            return

        # Open the CSV file in write mode
        with open(self.csv_file_path, mode='w', newline='') as file:
            # Create a CSV writer object
            csv_writer = csv.writer(file)

            # Write the header row
            csv_writer.writerow(["Temperature", "Voltage", "Current", "Total current", "RPM"])

            while True:
                self.request_esc_telemetry()
                telemetry_data = self.receive_esc_telemetry()
                if telemetry_data:
                    # Write telemetry data to the CSV file
                    csv_writer.writerow(telemetry_data.values())
                    print("ESC Telemetry Data written to CSV file.")
               # time.sleep(1)  # Wait for 1 second before requesting telemetry data again

if __name__ == "__main__":
    app = AI_drone_naor(csv_file_path='/home/drone/flight_data/flight_data.csv')  # Create an instance of AI_drone_naor
    app.run()  # Call the run method of AI_drone_naor
