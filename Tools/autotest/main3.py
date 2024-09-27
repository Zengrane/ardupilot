import time
import numpy as np
import random
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
lat_min = -35.36640660
lat_max = -35.35299608
lon_min = 149.15338018 
lon_max = 149.18100582


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
    """Generate new waypoints using CEM within a defined search area."""
    best_waypoints = []

    for _ in range(n_samples):
        # Generate random waypoints within the defined bounding box
        new_lat = random.uniform(lat_min, lat_max)
        new_lon = random.uniform(lon_min, lon_max)

        waypoint = LocationGlobalRelative(new_lat, new_lon, 100)

        # Calculate a cost (TO DO)
        cost = np.random.rand()  
        
        best_waypoints.append((waypoint, cost))
    
    # Sort by cost and keep the best ones
    best_waypoints.sort(key=lambda x: x[1])
    best_waypoints = best_waypoints[:n_best]
    
    # Generate new waypoints around the best waypoints
    new_waypoints = []
    for waypoint, _ in best_waypoints:
        new_lat = waypoint.lat + (random.uniform(-0.0001, 0.0001))
        new_lon = waypoint.lon + (random.uniform(-0.0001, 0.0001))
        
        # Ensure the new waypoints are still within the bounding box
        new_lat = max(lat_min, min(lat_max, new_lat))
        new_lon = max(lon_min, min(lon_max, new_lon))

        new_waypoints.append(LocationGlobalRelative(new_lat, new_lon, 100))
    
    return new_waypoints


# Main loop for waypoint navigation
while True:
    for idx in range(n_agents):
        current_location = seekers[idx].vehicle.location.global_relative_frame

        # Generate waypoints using CEM
        waypoints = generate_waypoints(current_location)

        for waypoint in waypoints:
            seekers[idx].vehicle.simple_goto(waypoint)

            while True:
                # Check if the drone has reached the waypoint
                drone_location = seekers[idx].vehicle.location.global_relative_frame
                dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                         drone_location.lat, drone_location.lon)
                agent_pos_NED = np.array([dN, dE, -drone_location.alt])
                waypoint_dE, waypoint_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                            waypoint.lat, waypoint.lon)
                waypoint_pos_NED = np.array([waypoint_dN, waypoint_dE, -waypoint.alt])
                distance_to_waypoint = get_distance_meters(agent_pos_NED, waypoint_pos_NED)

                print(f"Distance to Waypoint for UAV {idx + 1}: {distance_to_waypoint:.2f} meters")

                # If the drone has reached the waypoint, break the loop
                if distance_to_waypoint <= 100.:
                    print(f"UAV {idx + 1} reached the waypoint.")
                    break

                time.sleep(1)  # Short delay to avoid busy-waiting

    time.sleep(1)  

