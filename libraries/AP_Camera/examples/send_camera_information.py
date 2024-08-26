import threading
import time
from pymavlink import mavutil
import argparse

class CameraTrackingScript:
    def __init__(self, ip, port, sysid, compid):
        self.ip = ip
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.connection = None

    def connect_to_mavlink(self):
        self.connection = mavutil.mavlink_connection(f'udp:{self.ip}:{self.port}', source_system=self.sysid)
        print("Searching Vehicle")
        while not self.connection.probably_vehicle_heartbeat(self.connection.wait_heartbeat()):
            print(".", end="")
        print("\nFound Vehicle")
        self.connection.wait_heartbeat()
        print("Heartbeat received from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
        self.connection.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_UNINIT,
            mavlink_version=3
        )

    def send_camera_information(self):
        self.connection.mav.camera_information_send(
            int(time.time() * 1000) & 0xFFFFFFFF,          # time_boot_ms
            b"CameraVendor123" + b'\0' * (32 - len("CameraVendor123")),  # vendor_name
            b"CameraModel123" + b'\0' * (32 - len("CameraModel123")),    # model_name
            (1 << 24) | (0 << 16) | (0 << 8) | 1,          # firmware_version
            float('nan'),                                  # focal_length
            float('nan'),                                  # sensor_size_h
            float('nan'),                                  # sensor_size_v
            640,                                           # resolution_h
            480,                                           # resolution_v
            0,                                             # lens_id
            4095,                                          # flags
            0,                                             # cam_definition_version
            b"",                                           # cam_definition_uri
            0                                              # gimbal_device_id
        )
        print("Camera information message sent")

    def handle_camera_track_point(self, msg):
        print("Received MAV_CMD_CAMERA_TRACK_POINT command.")
        param1 = msg.param1
        param2 = msg.param2
        print(f"Tracking point parameters: param1={param1}, param2={param2}")

    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_UNINIT,
                mavlink_version=3
            )
            time.sleep(1)

    def run(self):
        self.connect_to_mavlink()
        self.send_camera_information()

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        while True:
            msg = self.connection.recv_match(type='COMMAND_LONG', blocking=True)
            if msg and msg.get_type() == 'COMMAND_LONG' and msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                if msg.target_system == self.sysid:
                    self.handle_camera_track_point(msg)
                else:
                    print("Received but not for us")

def main():
    parser = argparse.ArgumentParser(description="A script to demonstrate command-line arguments for sysid and compid.")
    parser.add_argument('--sysid', type=int, help='System ID', required=True)
    parser.add_argument('--compid', type=int, help='Component ID', required=True)
    args = parser.parse_args()

    ip = "127.0.0.1"
    port = 14570

    script = CameraTrackingScript(ip, port, args.sysid, args.compid)
    script.run()

if __name__ == "__main__":
    main()
