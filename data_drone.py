from pymavlink import mavutil

class drone_setup():
    def __init__(self):
        super().__init__()
        self.connection_string = '/dev/ttyUSB0'

drone = drone_setup()



# Establish connection
master = mavutil.mavlink_connection(drone.connection_string)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))



#check the gps work



#check the altitude of the drone - start take data from 1.5 m


#check if the drone is aitburn





while True:
    try:
        # Wait for a message from the device
        msg = master.recv_match()

        if msg:
            # Print received message
            print("Message ID:", msg.get_msgId())
            print("Message:", msg)
    except KeyboardInterrupt:
        print("Exiting...")
        break
