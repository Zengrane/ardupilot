import time
import numpy as np
import random
import threading
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters

# Camera class definition. Represents orientation of camera relative to drone
class Camera:
    def __init__(self, mounting_yaw_deg: float, mounting_pitch_deg: float, mounting_roll_deg: float):
        self.mounting_yaw_deg = mounting_yaw_deg
        self.mounting_pitch_deg = mounting_pitch_deg
        self.mounting_roll_deg = mounting_roll_deg
        self.Rc = np.eye(4)  # Initialize a 4x4 identity matrix
        self.setCameraOrientation(np.radians(mounting_roll_deg), np.radians(mounting_pitch_deg), np.radians(mounting_yaw_deg))

    def rotationMatrix(self, roll, pitch, yaw):
        """Calculate rotation matrix from roll, pitch, and yaw angles in radians."""
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def setCameraOrientation(self, roll, pitch, yaw):
        self.mounting_yaw_deg = np.degrees(yaw)  # Store yaw in degrees
        self.mounting_pitch_deg = np.degrees(pitch)  # Store pitch in degrees
        self.mounting_roll_deg = np.degrees(roll)  # Store roll in degrees

        # Calculate the rotation matrix
        rotation = self.rotationMatrix(roll, pitch, yaw)

        # Set the top-left 3x3 part of self.Rc
        self.Rc[:3, :3] = rotation
        self.Rc_i = np.linalg.inv(self.Rc)  # Inverse of the rotation matrix

# Function to visualize camera gimbal position (matplotlib)
def visualize_camera_gimbal(camera, position, target_positions):
    fig, ax = plt.subplots()

    # Drone position
    ax.plot(position.lon, position.lat, 'bo', label='Drone Position')

    # Define camera FOV
    fov_distance = 100  # Distance for the field of view in meters
    angle_offset = 45  # Angle offset for FOV visualization
    fov_angle = 2 * angle_offset  # FOV angle based on angle offset

    # Calculate FOV corners based on camera angles
    yaw_rad = np.radians(camera.mounting_yaw_deg)

    # Camera FOV corners in local ENU coordinates 
    corners = [
        (fov_distance * np.cos(yaw_rad + np.radians(-angle_offset)), 
         fov_distance * np.sin(yaw_rad + np.radians(-angle_offset))),
        (fov_distance * np.cos(yaw_rad + np.radians(angle_offset)), 
         fov_distance * np.sin(yaw_rad + np.radians(angle_offset))),
    ]

    # Plot the FOV
    for corner in corners:
        ax.plot([position.lon, position.lon + corner[0]], 
                [position.lat, position.lat + corner[1]], 'r--')

    # Plot target positions
    for target_position in target_positions:
        ax.plot(target_position.lon, target_position.lat, 'ro', label='Target Position')

    ax.set_title("Camera Gimbal Position")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()

# Function to get camera orientation from user input
def get_camera_orientation():
    try:
        yaw = float(input("Enter yaw angle (degrees): "))
        pitch = float(input("Enter pitch angle (degrees): "))
        roll = float(input("Enter roll angle (degrees): "))
        return yaw, pitch, roll
    except ValueError:
        print("Invalid input. Please enter numerical values.")
        return get_camera_orientation()

# Checks if a target is within the camera FOV 
def is_within_fov(camera, drone_position, target_position):
    # Calculate drone's yaw in radians
    drone_yaw_rad = np.radians(camera.mounting_yaw_deg)

    # Convert drone and target positions to ENU coordinates
    target_dE, target_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                             target_position.lat, target_position.lon)
    
    # Represents the direction from the drone to the target
    angle_to_target = np.arctan2(target_dN, target_dE)

    # How far off the target is from the direction the drone is facing
    angle_diff = np.degrees(angle_to_target - drone_yaw_rad)
    angle_diff = (angle_diff + 360) % 360  # Normalize to [0, 360)

    angle_offset = 45  # Adjust this to your desired angle offset
    fov_angle = 2 * angle_offset  # FOV angle is twice the angle offset

    return angle_diff <= fov_angle / 2

# Number of drones
n_agents = 1

# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seekers = [seeker0]

# Takeoff location and home base coordinates
takeoff_location = LocationGlobalRelative(-35.36487698, 149.17000667, 100)
home_lat_lon = np.array([-35.36341649, 149.16525123])

# Assign home location to each drone
for seeker in seekers:
    seeker.home_lat_lon = home_lat_lon

# Camera initialization with user input
cam_yaw, cam_pitch, cam_roll = get_camera_orientation()
camera = Camera(cam_yaw, cam_pitch, cam_roll)

# Visualize the camera gimbal position
position = LocationGlobalRelative(-35.36487698, 149.17000667, 100)  # Use the takeoff location
visualize_camera_gimbal(camera, position, [])

# Takeoff function
def takeoff_drone(seeker):
    while not seeker.vehicle.armed:
        seeker.arm_and_takeoff()
        seeker.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        
    seeker.vehicle.simple_goto(takeoff_location)
    while seeker.vehicle.location.global_relative_frame.alt < 90:
        time.sleep(1)
    print(f"UAV has reached target altitude: {seeker.vehicle.location.global_relative_frame.alt}")

for seeker in seekers:
    takeoff_drone(seeker)

print("All drones taken off")

# Generate potential waypoints for the drone to navigate to
def generate_waypoints(current_location, n_samples=100, n_best=10, mutation=0.01):
    best_waypoints = []

    for _ in range(n_samples):
        new_lat = random.uniform(-35.36415709, -35.36190691)
        new_lon = random.uniform(149.16397893, 149.16706865)
        waypoint = LocationGlobalRelative(new_lat, new_lon, 100)
        cost = np.random.rand()  
        best_waypoints.append((waypoint, cost))
    
    best_waypoints.sort(key=lambda x: x[1])
    best_waypoints = best_waypoints[:n_best]
    
    new_waypoints = []
    for waypoint, _ in best_waypoints:
        new_lat = waypoint.lat + (random.uniform(-mutation, mutation))
        new_lon = waypoint.lon + (random.uniform(-mutation, mutation))
        
        new_lat = max(-35.36415709, min(-35.36190691, new_lat))
        new_lon = max(149.16397893, min(149.16706865, new_lon))

        new_waypoints.append(LocationGlobalRelative(new_lat, new_lon, 100))
    
    return new_waypoints

# Update the target's location within the NAI
def update_target_location():
    global target1_lat, target1_lon, target2_lat, target2_lon
    while True:
        with target_lock:
            # Target 1
            target1_lat += random.uniform(-0.00005, 0.00005)
            target1_lon += random.uniform(-0.00005, 0.00005)
            print(f"Updated Target 1 location: ({target1_lat}, {target1_lon})")

            # Target 2
            target2_lat += random.uniform(-0.00005, 0.00005)
            target2_lon += random.uniform(-0.00005, 0.00005)
            print(f"Updated Target 2 location: ({target2_lat}, {target2_lon})")

        time.sleep(8)  

# Detection and target logic
target_lock = threading.Lock()
target1_lat = -35.36163967
target1_lon = 149.16501711
target2_lat = -35.36200000  
target2_lon = 149.16600000
detection_range = 100.0

def detect_targets(seeker):
    target1_detected = False
    target2_detected = False
    
    while True:
        current_location = seeker.vehicle.location.global_relative_frame
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 current_location.lat, current_location.lon)
        drone_pos_NED = np.array([dN, dE, -current_location.alt])
        
        with target_lock:
            # Check target 1
            target1_dE, target1_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                       target1_lat, target1_lon)
            target1_pos_NED = np.array([target1_dN, target1_dE, -100])
            distance_to_target1 = get_distance_meters(drone_pos_NED, target1_pos_NED)

            # Check target 2
            target2_dE, target2_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                       target2_lat, target2_lon)
            target2_pos_NED = np.array([target2_dN, target2_dE, -100])
            distance_to_target2 = get_distance_meters(drone_pos_NED, target2_pos_NED)

        # Check for target detection. Checks that the target is within the detection range and within fov
        if distance_to_target1 <= detection_range and is_within_fov(camera, current_location, LocationGlobalRelative(target1_lat, target1_lon, 0)):
            if not target1_detected:
                print(f"Target 1 detected by UAV at distance: {distance_to_target1:.2f} meters")
                target1_detected = True
            else:
                print(f"Distance to Target 1: {distance_to_target1:.2f} meters")
        else:
            if target1_detected:
                print("Target 1 out of range or out of FOV.")
                target1_detected = False

        if distance_to_target2 <= detection_range and is_within_fov(camera, current_location, LocationGlobalRelative(target2_lat, target2_lon, 0)):
            if not target2_detected:
                print(f"Target 2 detected by UAV at distance: {distance_to_target2:.2f} meters")
                target2_detected = True
            else:
                print(f"Distance to Target 2: {distance_to_target2:.2f} meters")
        else:
            if target2_detected:
                print("Target 2 out of range or out of FOV.")
                target2_detected = False

        time.sleep(1)

# Start target location update in a separate thread
target_update_thread = threading.Thread(target=update_target_location)
target_update_thread.daemon = True
target_update_thread.start()

# Start target detection in a separate thread
for seeker in seekers:
    detection_thread = threading.Thread(target=detect_targets, args=(seeker,))
    detection_thread.daemon = True
    detection_thread.start()

# Main loop for waypoint navigation
while True:
    for seeker in seekers:
        current_location = seeker.vehicle.location.global_relative_frame
        waypoints = generate_waypoints(current_location)

        # Create a list of current target positions
        target_positions = [
            LocationGlobalRelative(target1_lat, target1_lon, 0),
            LocationGlobalRelative(target2_lat, target2_lon, 0)
        ]

        # Visualize the camera gimbal and target positions
        visualize_camera_gimbal(camera, current_location, target_positions)

        for waypoint in waypoints:
            seeker.vehicle.simple_goto(waypoint)

            while True:
                drone_location = seeker.vehicle.location.global_relative_frame
                dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                         drone_location.lat, drone_location.lon)
                agent_pos_NED = np.array([dN, dE, -drone_location.alt])
                waypoint_dE, waypoint_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                            waypoint.lat, waypoint.lon)
                waypoint_pos_NED = np.array([waypoint_dN, waypoint_dE, -waypoint.alt])
                distance_to_waypoint = get_distance_meters(agent_pos_NED, waypoint_pos_NED)

                if distance_to_waypoint <= 100.:
                    break

                time.sleep(1)

    time.sleep(1)
