"""
Required Installations:
pip install opencv-python-headless pymavlink ultralytics
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
"""

import cv2
import threading
from pymavlink import mavutil
import math 
from ultralytics import YOLO
import argparse

class DroneGimbalControl:
    def __init__(self, video_source=None):
        # Set default video source if not provided
        if video_source is None:
            video_source = 'udpsrc port=5600 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink'

        self.master = mavutil.mavlink_connection('127.0.0.1:14560')
        self.master.wait_heartbeat()
        print("Heartbeat from the system (system %u component %u)" % 
             (self.master.target_system, self.master.target_component))

        # Store relevant parameters as object state
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        self.flags = 0
        self.gimbal_device_id = 0

        self.cap = cv2.VideoCapture(video_source, cv2.CAP_GSTREAMER)
        self.model = YOLO("yolov8n.pt")
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.center_y = 0
        self.center_x = 0

    def send_gimbal_manager_pitch_yaw_angles(self, pitch, yaw, pitch_rate, yaw_rate):
        msg = self.master.mav.gimbal_manager_set_pitchyaw_encode(
            self.target_system,
            self.target_component,
            self.flags,
            self.gimbal_device_id,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate
        )
        self.master.mav.send(msg)

    def send_command(self):
        while True:
            centre_x_copy = int(self.center_x)
            centre_y_copy = int(self.center_y)
            if (centre_x_copy == 0 and centre_y_copy == 0):
                diff_x = 0
                diff_y = 0
            else:
                diff_x = (centre_x_copy - (640 / 2)) / 2
                diff_y = -(centre_y_copy - (480 / 2)) / 2

            self.send_gimbal_manager_pitch_yaw_angles(float("NaN"), float("NaN"), math.radians(diff_y), math.radians(diff_x))

    def update_frame(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame

    def detect_faces(self):
        final_list = []
        while True:
            if self.latest_frame is not None:
                with self.frame_lock:
                    frame = self.latest_frame

                results = self.model.track(frame, stream=True)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        classf = int(box.cls)
                        name = self.model.names[classf]
                        if name != "person" or box.conf < 0.7:
                            continue
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        final_list = (x1, y1, x2, y2)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

                faces = [final_list]
                if len(faces) == 0:
                    self.center_x = 640 / 2
                    self.center_y = 480 / 2
                    continue

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (w, h), (255, 0, 0), 2)
                    self.center_x = (x + w) // 2
                    self.center_y = (y + h) // 2
                    cv2.circle(frame, (self.center_x, self.center_y), 3, (0, 255, 0), -1)

    def run(self):
        update_thread = threading.Thread(target=self.update_frame)
        update_thread.start()

        detect_thread = threading.Thread(target=self.detect_faces)
        detect_thread.start()

        control_thread = threading.Thread(target=self.send_command)
        control_thread.start()

        while True:
            with self.frame_lock:
                if self.latest_frame is not None:
                    cv2.imshow('Frame', self.latest_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Drone Gimbal Control with YOLOv8.')
parser.add_argument('--video-source', type=str, help='GStreamer pipeline or other video source')
args = parser.parse_args()

if __name__ == "__main__":
    # Pass the argument if provided; otherwise, the default will be used
    drone_control = DroneGimbalControl(args.video_source)
    drone_control.run()