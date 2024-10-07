import time
import numpy as np
import random
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters

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

# Setting NAI (Area that each drone should search for targets)
lat_min = -35.36415709 
lat_max = -35.36190691
lon_min = 149.16397893
lon_max = 149.16706865

# Initial Target Location
target1_lat = -35.36163967
target1_lon = 149.16501711

target2_lat = -35.36200000  
target2_lon = 149.16600000

# Detection range in meters
detection_range = 100.0
target_lock = threading.Lock()

# Focus parameter: (0.0 = 100% coverage, 1.0 = 100% target focus)
focus_on_targets_ratio = None

while focus_on_targets_ratio is None:
    try:
        user_input = input("Enter focus ratio (0.0 for 100% coverage, 1.0 for 100% target focus): ")
        focus_on_targets_ratio = float(user_input)

        if focus_on_targets_ratio < 0.0 or focus_on_targets_ratio > 1.0:
            print("Please enter a value between 0.0 and 1.0.")
            focus_on_targets_ratio = None

    except ValueError:
        print("Invalid input. Please enter a numerical value between 0.0 and 1.0.")

# Initialize detection states
target1_detected = False
target2_detected = False

# Update the target's location within the NAI
def update_target_location():
    global target1_lat, target1_lon, target2_lat, target2_lon
    while True:
        # Acquires the lock to ensure only one thread can modify target coordinates
        with target_lock:
            # Target 1
            target1_lat += random.uniform(-0.00005, 0.00005)
            target1_lon += random.uniform(-0.00005, 0.00005)
            target1_lat = max(lat_min, min(lat_max, target1_lat))
            target1_lon = max(lon_min, min(lon_max, target1_lon))
            print(f"Updated Target 1 location: ({target1_lat}, {target1_lon})")

            # Target 2
            target2_lat += random.uniform(-0.00005, 0.00005)
            target2_lon += random.uniform(-0.00005, 0.00005)
            target2_lat = max(lat_min, min(lat_max, target2_lat))
            target2_lon = max(lon_min, min(lon_max, target2_lon))
            print(f"Updated Target 2 location: ({target2_lat}, {target2_lon})")

        time.sleep(8)  

def detect_targets(seeker):
    global target1_detected, target2_detected
    while True:
        current_location = seeker.vehicle.location.global_relative_frame
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 current_location.lat, current_location.lon)
        drone_pos_NED = np.array([dN, dE, -current_location.alt])
        
        # Acquires lock 
        with target_lock:
            # Calculate distance to target 1
            target1_dE, target1_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                       target1_lat, target1_lon)
            target1_pos_NED = np.array([target1_dN, target1_dE, -100])
            distance_to_target1 = get_distance_meters(drone_pos_NED, target1_pos_NED)

            # Calculate distance to target 2
            target2_dE, target2_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                       target2_lat, target2_lon)
            target2_pos_NED = np.array([target2_dN, target2_dE, -100])
            distance_to_target2 = get_distance_meters(drone_pos_NED, target2_pos_NED)

        # Check if target 1 is within range 
        if distance_to_target1 <= detection_range:
            if not target1_detected:
                print(f"Target 1 detected by UAV at distance: {distance_to_target1:.2f} meters")
                target1_detected = True  # Update state to detected
            else:
                print(f"Distance to Target 1: {distance_to_target1:.2f} meters")  # Continuous distance update
        else:
            if target1_detected:
                print("Target 1 out of range.")
                target1_detected = False  # Update state to not detected

        # Check if target 2 is within range 
        if distance_to_target2 <= detection_range:
            if not target2_detected:
                print(f"Target 2 detected by UAV at distance: {distance_to_target2:.2f} meters")
                target2_detected = True  # Update state to detected
            else:
                print(f"Distance to Target 2: {distance_to_target2:.2f} meters")  # Continuous distance update
        else:
            if target2_detected:
                print("Target 2 out of range.")
                target2_detected = False  # Update state to not detected

        time.sleep(1)

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

# Generates potential waypoints for the drone to navigate to
def generate_waypoints(current_location, n_samples=100, n_best=10, mutation=0.01):
    best_waypoints = []

    # Loops to generate random waypoints
    for _ in range(n_samples):
        new_lat = random.uniform(lat_min, lat_max)
        new_lon = random.uniform(lon_min, lon_max)
        waypoint = LocationGlobalRelative(new_lat, new_lon, 100)
        # Randomised cost
        cost = np.random.rand()  
        best_waypoints.append((waypoint, cost))
    
    best_waypoints.sort(key=lambda x: x[1])
    best_waypoints = best_waypoints[:n_best]
    
    new_waypoints = []
    for waypoint, _ in best_waypoints:
        new_lat = waypoint.lat + (random.uniform(-0.0001, 0.0001))
        new_lon = waypoint.lon + (random.uniform(-0.0001, 0.0001))
        
        new_lat = max(lat_min, min(lat_max, new_lat))
        new_lon = max(lon_min, min(lon_max, new_lon))

        # Sort the waypoints based on their costs in ascending order
        new_waypoints.append(LocationGlobalRelative(new_lat, new_lon, 100))
    
    return new_waypoints

# Start target location update in a separate thread
target_update_thread = threading.Thread(target=update_target_location)
target_update_thread.daemon = True  # Allow thread to exit when main program exits
target_update_thread.start()

# Start target detection in a separate thread
for seeker in seekers:
    detection_thread = threading.Thread(target=detect_targets, args=(seeker,))
    detection_thread.daemon = True  # Allow thread to exit when main program exits
    detection_thread.start()

# Main loop for waypoint navigation
while True:
    for seeker in seekers:
        current_location = seeker.vehicle.location.global_relative_frame
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 current_location.lat, current_location.lon)
        drone_pos_NED = np.array([dN, dE, -current_location.alt])
        
        # Checks if any target is detected
        with target_lock:
            target_detected = any([target1_detected, target2_detected])

        # Dynamic focus decision based on user-defined ratio (Determines whether drones should focus on target or cover area)
        if target_detected and random.random() < focus_on_targets_ratio:  # Ratio for focusing (Checks if a target was detected and if a random number is less than the ratio)
            # Focus on detected targets if both conditions met
            print("Focusing on detected targets...")
            # Stores distances to detected targets
            target_dists = []
            # Calculates east and north coordinates of target 1 and converts them to NED
            if target1_detected:
                target1_dE, target1_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                           target1_lat, target1_lon)
                target1_pos_NED = np.array([target1_dN, target1_dE, -100])
                distance_to_target1 = get_distance_meters(drone_pos_NED, target1_pos_NED)
                target_dists.append((distance_to_target1, target1_lat, target1_lon))

            # Calculates east and north coordinates of target 2 and converts them to NED
            if target2_detected:
                target2_dE, target2_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                           target2_lat, target2_lon)
                target2_pos_NED = np.array([target2_dN, target2_dE, -100])
                distance_to_target2 = get_distance_meters(drone_pos_NED, target2_pos_NED)
                target_dists.append((distance_to_target2, target2_lat, target2_lon))

            # Determine closest target, and command drone to go to the closest waypoint of that target
            closest_target = min(target_dists, key=lambda x: x[0]) if target_dists else None
            if closest_target:
                closest_lat, closest_lon = closest_target[1], closest_target[2]
                closest_waypoint = LocationGlobalRelative(closest_lat, closest_lon, 100)
                seeker.vehicle.simple_goto(closest_waypoint)

        # If the previous condition is not met make the drone focus on covering area
        else:
            # Focus on area coverage
            print("Covering the area...")
            waypoints = generate_waypoints(current_location)
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
