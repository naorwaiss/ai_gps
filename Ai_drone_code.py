import time
from pymavlink import mavutil
import csv
import os
import datetime
import queue




class AI_drone_naor:
    def __init__(self, connection_string='/dev/ttyS0', baudrate=57600, csv_file_directory='/home/linaro/flight_data/',
                 output_queue=None):
        self.output_queue = output_queue
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
        self.satellite_count = None
        self.hdop = None
        self.csv_file_directory = csv_file_directory

        self.buffer_size = 30  # for example
        self.data_matrix = None
        self.received_data = []
        self.delta_t = 0

        # Generate CSV filename based on current date and time
        current_datetime = datetime.datetime.now()
        filename = f"flight_data_{current_datetime.strftime('%d-%m-%Y_%H-%M-%S')}.csv"
        self.csv_file_path = os.path.join(self.csv_file_directory, filename)




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
        """
        Receive esc telemetry - the esc telemetry run on 10 Hz
        :return:
        """
        # Wait for the next message
        msg_telemetry = self.master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=True, timeout=1.0)
        if msg_telemetry:
            # ESC telemetry data received, return it
            return {
                "Temperature": msg_telemetry.temperature,
                "Voltage": msg_telemetry.voltage,
                "RPM": msg_telemetry.rpm
            }
        else:
            return None

    def request_compass_telemetry(self):
        # Send a command to request compass telemetry data
        self.master.mav.command_long_send(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command to request message
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_COMPASS,  # Message ID for compass telemetry data
            0,  # Param 1 (0 for request)
            0,  # Param 2 (0 for request)
            0,  # Param 3 (0 for request)
            0,  # Param 4 (0 for request)
            0,  # Param 5 (0 for request)
            0,  # Param 6 (0 for request)
            0  # Param 7 (0 for request)
        )

    def receive_compass_telemetry(self):
        """
        GPS function - disable
        data scaling and filtering - disable
        Receive compass telemetry
        :return: Compass telemetry data
        """
        msg_compass = self.master.recv_match(type='COMPASS', blocking=True)
        if msg_compass is not None:
            # Extract compass data
            compass_id = msg_compass.compid
            current_compass_heading = msg_compass.heading
            return {
                "Heading": current_compass_heading
            }
        else:
            return None

    def request_imu_reading(self):
        """
        GPS function - disable
        data scaling and filtering - disable
        time 10hz
        Imu reading
        :return:
        """
        self.master.mav.request_data_stream_send(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # Request RAW_IMU data stream
            10,  # Request at 10 Hz
            1  # Start/stop stream (1=start, 0=stop)
        )

    def receive_raw_imu_data(self):
        """
        Receive RAW_IMU data
        :return: RAW_IMU data
        """
        msg_raw_imu = self.master.recv_match(type='RAW_IMU', blocking=True)
        if msg_raw_imu is not None:
            return {
                "Time_usec": msg_raw_imu.time_usec,
                "Xacc": msg_raw_imu.xacc,
                "Yacc": msg_raw_imu.yacc,
                "Zacc": msg_raw_imu.zacc,
                "Xgyro": msg_raw_imu.xgyro,
                "Ygyro": msg_raw_imu.ygyro,
                "Zgyro": msg_raw_imu.zgyro,
                "Xmag": msg_raw_imu.xmag,
                "Ymag": msg_raw_imu.ymag,
                "Zmag": msg_raw_imu.zmag
            }
        else:
            return None

    def recive_gps_heading(self):
        return

    def run(self):
        self.wait_heartbeat()

        with open(self.csv_file_path, mode='w', newline='') as file:
            csv_writer = csv.writer(file)
            csv_writer.writerow(
                ["ET1", "ET2", "ET3", "ET4", "EV1", "EV2", "EV3", "EV4", "RPM1", "RPM2", "RPM3", "RPM4",
                 "Xacc", "Yacc", "Zacc", "gyroX", "gyroY", "gyroZ", "magX", "magY", "magZ",
                 "cmaera_x","camera_y","dt_hz"])  # Add "time" to the header

            while True:
                # Initialize a new buffer for each iteration
                data_buffer = []
                cam_movment = self.output_queue.get()

                # Collect telemetry data and fill the buffer
                start_time = time.time()  # Record start time for the iteration
                for i in range(self.buffer_size):
                    self.request_esc_telemetry()
                    self.request_imu_reading()
                    esc_telemetry_data = self.receive_esc_telemetry()
                    imu_reading = self.receive_raw_imu_data()

                    if esc_telemetry_data is not None and imu_reading is not None:
                        data_buffer.append([
                            esc_telemetry_data["Temperature"][0], esc_telemetry_data["Temperature"][1],
                            esc_telemetry_data["Temperature"][2], esc_telemetry_data["Temperature"][3],
                            esc_telemetry_data["Voltage"][0], esc_telemetry_data["Voltage"][1],
                            esc_telemetry_data["Voltage"][2], esc_telemetry_data["Voltage"][3],
                            esc_telemetry_data["RPM"][0], esc_telemetry_data["RPM"][1],
                            esc_telemetry_data["RPM"][2], esc_telemetry_data["RPM"][3],
                            imu_reading["Xacc"], imu_reading["Yacc"], imu_reading["Zacc"],
                            imu_reading["Xgyro"], imu_reading["Ygyro"], imu_reading["Zgyro"],
                            imu_reading["Xmag"], imu_reading["Ymag"], imu_reading["Zmag"],
                            cam_movment[0],cam_movment[1],
                            1/(time.time() - start_time)  # Add delta_t (time since start) to the row
                        ])
                        print(f"you are in the {i} iteration")

                # Write the buffer to the CSV file
                for row in data_buffer:
                    csv_writer.writerow(row)


if __name__ == "__main__":
    output_queue = queue.Queue()  # Initialize the output queue
    app = AI_drone_naor(output_queue=output_queue)
    app.run()
