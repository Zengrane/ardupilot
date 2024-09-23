from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit as dk
import time
import numpy as np
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters

n_agents = 1
# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seekers = [seeker0]

# The coordinates and altitude where the drone will take off 
takeoff_location = dk.LocationGlobalRelative(-35.36487698, 149.17000667, 100)

# Home base coordinates 
home_lat_lon = np.array([-35.36341649, 149.16525123])

# Define waypoints
waypoints = [
    dk.LocationGlobalRelative(-35.36486230, 149.16401189, 100),  # First waypoint
    dk.LocationGlobalRelative(-35.36307697, 149.15926171, 100),  # Second waypoint
    dk.LocationGlobalRelative(-35.35867523, 149.16073485, 100),  # Third waypoint
]

# Home location assigned to each drone
for idx in range(n_agents):
    seekers[idx].home_lat_lon = home_lat_lon

# Takeoff
takeoff_status = False
while not takeoff_status:
    for idx in range(n_agents):
        if not seekers[idx].vehicle.armed:
            seekers[idx].arm_and_takeoff()
            seekers[idx].mode = VehicleMode("GUIDED")
            time.sleep(1)
    
    for idx in range(n_agents):
        seekers[idx].vehicle.simple_goto(takeoff_location)
        if seekers[idx].vehicle.location.global_relative_frame.alt >= 90:  # 90% of 100
            print(f"UAV{idx + 1} reached target altitude")
            takeoff_status = True
        time.sleep(1)

print("All drones taken off")

# Track which waypoints have been reached
waypoint_index = 0
agents_done = [False] * n_agents

# Main loop for waypoint navigation
while waypoint_index < len(waypoints):
    for idx in range(n_agents):
        current_waypoint = waypoints[waypoint_index]
        
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
        if distance_to_waypoint <= 80. and not agents_done[idx]:
            agents_done[idx] = True
            print(f"UAV {idx + 1} reached Waypoint {waypoint_index + 1}")

    # Check if all agents are done with the current waypoint
    if all(agents_done):
        waypoint_index += 1  # Move to the next waypoint
        agents_done = [False] * n_agents  # Reset for next waypoint
        print(f"Moving to Waypoint {waypoint_index + 1 if waypoint_index < len(waypoints) else 'completed'}")

    time.sleep(1)

print("All waypoints reached")




"""
Test roll/pitch control: set_attitude command here (roll_angle and pitch_angle are in degrees)
"""
# seekers[idx].set_attitude(roll_angle=10., pitch_angle=1., thrust=0.5, duration=10)