import time
import numpy as np
import random
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north

# Number of drones and targets
n_agents = 3
n_targets = 3

# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seeker1 = seekerAgent("127.0.0.1:14561", 1)
seeker2 = seekerAgent("127.0.0.1:14571", 2)
seekers = [seeker0, seeker1, seeker2]

# Takeoff location and home base coordinates
takeoff_location = LocationGlobalRelative(-35.36025747, 149.16462578, 100)
home_lat_lon = np.array([-35.36500000, 149.16500000])

# Assign home location to each drone
for seeker in seekers:
    seeker.home_lat_lon = home_lat_lon

# Setting NAI (Area that each drone should search for targets)
lat_min = -35.36500000
lat_max = -35.35500000
lon_min = 149.16000000
lon_max = 149.17000

# Initial Target Locations
targets = [
    (-35.36284990, 149.16244184),
    (-35.36384990, 149.16144184),
    (-35.36484990, 149.16344184)
]

# Detection range in meters
detection_range = 100.0
target_detected = [False] * n_targets
was_target_detected = [False] * n_targets
target_lock = threading.Lock()

# Flags to track the target state
is_tracking = [False] * n_agents
tracking_target = [None] * n_agents

# Initialize exploration map
exploration_map = {}

def initialize_exploration_map():
    for lat in np.linspace(lat_min, lat_max, num=10):
        for lon in np.linspace(lon_min, lon_max, num=10):
            exploration_map[(lat, lon)] = False

initialize_exploration_map()

def update_target_locations():
    global targets
    while True:
        with target_lock:
            new_targets = []
            for lat, lon in targets:
                new_lat = lat + random.uniform(-0.000005, 0.000005)
                new_lon = lon + random.uniform(-0.000005, 0.000005)
                
                new_lat = max(lat_min, min(lat_max, new_lat))
                new_lon = max(lon_min, min(lon_max, new_lon))
                
                new_targets.append((new_lat, new_lon))
            
            targets = new_targets
            for idx, (new_lat, new_lon) in enumerate(targets):
                print(f"Updated target {idx} location: ({new_lat}, {new_lon})")
        
        time.sleep(3)

def detect_targets(seeker):
    global target_detected, was_target_detected
    seeker_id = seeker.SYSID
    while True:
        current_location = seeker.vehicle.location.global_relative_frame
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 current_location.lat, current_location.lon)
        
        with target_lock:
            distances_to_targets = [calculate_distance((current_location.lat, current_location.lon), target) for target in targets]
        
        for i, distance in enumerate(distances_to_targets):
            if distance <= detection_range:
                target_detected[i] = True
                if not was_target_detected[i]:
                    print(f"Target {i} detected by seeker {seeker_id} at distance: {distance:.2f} meters")
                    is_tracking[seeker_id] = True
                    tracking_target[seeker_id] = i  # Start tracking this target
                else:
                    print(f"Distance to target {i}: {distance:.2f} meters")
                was_target_detected[i] = True
            else:
                if was_target_detected[i]:
                    print(f"Target {i} out of range.")
                target_detected[i] = False
                was_target_detected[i] = False

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

def calculate_distance(point1, point2):
    lat1, lon1 = point1
    lat2, lon2 = point2

    R = 6371000  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)

    a = np.sin(delta_phi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    distance = R * c  # Distance in meters
    return distance

def unexplored_cost(lat, lon):
    return 10 if not exploration_map.get((lat, lon), False) else 0 

# Cost function to evaluate actions
def cost_function(drone, targets):
    drone_location = drone.vehicle.location.global_relative_frame
    drone_lat_lon = (drone_location.lat, drone_location.lon)

    distances_to_targets = [calculate_distance(drone_lat_lon, target) for target in targets]
    min_distance_to_target = min(distances_to_targets)
    
    unexplored_penalty = unexplored_cost(drone_location.lat, drone_location.lon)

    return min_distance_to_target + unexplored_penalty

def generate_waypoints(current_location, seeker, n_samples=100, n_best=10, mutation=0.01):
    best_waypoints = []

    for _ in range(n_samples):
        new_lat = random.uniform(lat_min, lat_max)
        new_lon = random.uniform(lon_min, lon_max)
        waypoint = LocationGlobalRelative(new_lat, new_lon, 100)
        cost = cost_function(seeker, targets)  
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

# Start target location updates in a separate thread
target_update_thread = threading.Thread(target=update_target_locations)
target_update_thread.daemon = True
target_update_thread.start()

# Start target detection in a separate thread
for seeker in seekers:
    detection_thread = threading.Thread(target=detect_targets, args=(seeker,))
    detection_thread.daemon = True
    detection_thread.start()

# Main loop for waypoint navigation
def navigate_waypoints(seeker):
    while True:
        current_location = seeker.vehicle.location.global_relative_frame
        
        # Check if there is a target to track
        seeker_id = seeker.SYSID
        if is_tracking[seeker_id]:
            target_index = tracking_target[seeker_id]
            target_location = targets[target_index]
            seeker.vehicle.simple_goto(LocationGlobalRelative(target_location[0], target_location[1], 100))
            print(f"Seeker {seeker_id} tracking target {target_index} at {target_location}")
            
            # Allow the drone to track the target for a certain amount of time
            time.sleep(5)  # Adjust this value to change tracking duration
            
            # Reset tracking if the target is lost
            if not target_detected[target_index]:
                is_tracking[seeker_id] = False
                tracking_target[seeker_id] = None
                print(f"Seeker {seeker_id} lost track of target {target_index}, switching to exploration.")
        
        else:
            # Generate waypoints for exploration if no target is being tracked
            waypoints = generate_waypoints(current_location, seeker)
            for waypoint in waypoints:
                seeker.vehicle.simple_goto(waypoint)
                print(f"Seeker {seeker_id} exploring waypoint at {waypoint.lat}, {waypoint.lon}")
                time.sleep(5)  # Adjust the delay as needed

# Start navigation threads for each seeker
for seeker in seekers:
    navigation_thread = threading.Thread(target=navigate_waypoints, args=(seeker,))
    navigation_thread.daemon = True
    navigation_thread.start()

# Keep the main thread alive
while True:
    time.sleep(1)
