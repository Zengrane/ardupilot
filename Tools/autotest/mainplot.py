import time
import numpy as np
import random
import threading
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters

# Drone's takeoff location
takeoff_location = LocationGlobalRelative(-35.36487698, 149.17000667, 100)

# Define the home base coordinates
home_lat_lon = np.array([-35.36341649, 149.16525123])

# Plotting the graph
xpoints = np.array([-50, 150])
ypoints = np.array([-50, 150])

# Convert drone's takeoff location to east-north coordinates
dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                          takeoff_location.lat, takeoff_location.lon)

# Scatter the starting location on the plot
plt.scatter(dN, dE, color='blue', label='Drone Starting Location')
plt.text(dN, dE, ' Start', fontsize=12, verticalalignment='bottom')

# Add legend and show the plot
plt.legend()
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.title('Drone Path Visualization')
plt.grid()
plt.show()

# Number of drones
n_agents = 1

# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seekers = [seeker0]

# Takeoff location and home base coordinates
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

# Update the target's location within the NAI
def update_target_location():
    global target1_lat, target1_lon, target2_lat, target2_lon
    while True:
        # Acquires the lock to ensure only one thread can modify target coordinates
        with target_lock:
            # Target 1
            # Updates coordinates by adding a small random value
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
    global target_detected, was_target_detected
    target1_detected = False  # State for target 1
    target2_detected = False  # State for target 2
    
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
