from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters
import random

# Number of drones
n_agents = 1

# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seekers = [seeker0]

# The coordinates and altitude where the drone will take off 
takeoff_location = LocationGlobalRelative(-35.36487698, 149.17000667, 100)

# Home base coordinates 
home_lat_lon = np.array([-35.36341649, 149.16525123])

# Assign home location to each drone
for idx in range(n_agents):
    seekers[idx].home_lat_lon = home_lat_lon

# Takeoff
def takeoff_drone(seeker):
    while not seeker.vehicle.armed:
        seeker.arm_and_takeoff()
        seeker.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        
    seeker.vehicle.simple_goto(takeoff_location)
    while seeker.vehicle.location.global_relative_frame.alt < 90:  # Wait until altitude is reached
        time.sleep(1)
    print(f"UAV has reached target altitude: {seeker.vehicle.location.global_relative_frame.alt}")

for seeker in seekers:
    takeoff_drone(seeker)

print("All drones taken off")

# Function to generate new waypoints
def generate_waypoint(current_location, distance=0.002):
    """Generate a new waypoint a certain distance away from the current location."""
    radius = distance  # About 222 meters at the equator
    new_lat = current_location.lat + (random.uniform(-1, 1) * radius)
    new_lon = current_location.lon + (random.uniform(-1, 1) * radius)
    return LocationGlobalRelative(new_lat, new_lon, 100)

# Initialize waypoint
current_waypoint = generate_waypoint(seekers[0].vehicle.location.global_relative_frame)

# Main loop for waypoint navigation
while True:
    for idx in range(n_agents):
        current_location = seekers[idx].vehicle.location.global_relative_frame
        
        # Command drone to go to the current waypoint
        seekers[idx].vehicle.simple_goto(current_waypoint)

        # Get current drone position
        drone_location = seekers[idx].vehicle.location.global_relative_frame
        
        # Calculate distance to waypoint
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 drone_location.lat, drone_location.lon)
        agent_pos_NED = np.array([dN, dE, -drone_location.alt])
        
        # Convert waypoint to NED coordinates
        waypoint_dE, waypoint_dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                    current_waypoint.lat, current_waypoint.lon)
        waypoint_pos_NED = np.array([waypoint_dN, waypoint_dE, -current_waypoint.alt])
        
        # Calculate distance to waypoint
        distance_to_waypoint = get_distance_meters(agent_pos_NED, waypoint_pos_NED)

        print(f"Distance to Waypoint for UAV {idx + 1}: {distance_to_waypoint:.2f} meters")

        # Check if the drone has reached the waypoint
        if distance_to_waypoint <= 100.:
            print(f"UAV {idx + 1} reached the waypoint, generating a new waypoint.")
            current_waypoint = generate_waypoint(drone_location)  # Generate a new waypoint

    time.sleep(1)  # Adjust the sleep time as needed
