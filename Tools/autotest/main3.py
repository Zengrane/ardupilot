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
lat_min = -35.36190691
lat_max = -35.36415709 
lon_min = 149.16397893
lon_max = 149.16706865

# Initial Target Location
target_lat = -35.36284990 
target_lon = 149.16244184

# Detection range in meters
detection_range = 100.0
target_detected = False
was_target_detected = False  # Track the previous detection state
target_lock = threading.Lock()

def update_target_location():
    global target_lat, target_lon
    while True:
        with target_lock:
            target_lat += random.uniform(-0.00005, 0.00005)
            target_lon += random.uniform(-0.00005, 0.00005)
            target_lat = max(lat_min, min(lat_max, target_lat))
            target_lon = max(lon_min, min(lon_max, target_lon))
            print(f"Updated target location: ({target_lat}, {target_lon})")
        time.sleep(3)  # Adjust update frequency here

def detect_target(seeker):
    global target_detected, was_target_detected
    while True:
        current_location = seeker.vehicle.location.global_relative_frame
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 current_location.lat, current_location.lon)
        drone_pos_NED = np.array([dN, dE, -current_location.alt])
        
        with target_lock:
            target_dE, target_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                     target_lat, target_lon)
            target_pos_NED = np.array([target_dN, target_dE, -100])  # Assuming target altitude is 100 meters

        distance_to_target = get_distance_meters(drone_pos_NED, target_pos_NED)

        if distance_to_target <= detection_range:
            target_detected = True
            if not was_target_detected:
                print(f"Target detected by UAV at distance: {distance_to_target:.2f} meters")
            else:
                print(f"Distance to target: {distance_to_target:.2f} meters")  # Print distance continuously
            
            was_target_detected = True  # Update state to detected
        else:
            if was_target_detected:
                print("Target out of range.")
            target_detected = False
            was_target_detected = False  # Update state to not detected

        time.sleep(1)  # Short delay to avoid busy-waiting

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

def generate_waypoints(current_location, n_samples=100, n_best=10, mutation=0.01):
    best_waypoints = []

    for _ in range(n_samples):
        new_lat = random.uniform(lat_min, lat_max)
        new_lon = random.uniform(lon_min, lon_max)
        waypoint = LocationGlobalRelative(new_lat, new_lon, 100)
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

        new_waypoints.append(LocationGlobalRelative(new_lat, new_lon, 100))
    
    return new_waypoints

# Start target location update in a separate thread
target_update_thread = threading.Thread(target=update_target_location)
target_update_thread.daemon = True  # Allow thread to exit when main program exits
target_update_thread.start()

# Start target detection in a separate thread
for seeker in seekers:
    detection_thread = threading.Thread(target=detect_target, args=(seeker,))
    detection_thread.daemon = True  # Allow thread to exit when main program exits
    detection_thread.start()

# Main loop for waypoint navigation
while True:
    for seeker in seekers:
        current_location = seeker.vehicle.location.global_relative_frame
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
