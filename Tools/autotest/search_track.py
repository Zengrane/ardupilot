from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from dk_agent_plane import seekerAgent  # Adjust the import as necessary
from geolocation import get_distance_meters
import ray
import toml
from algorithms import NAI, CEMAgent  # Import your NAI and CEMAgent classes

# Load configuration
config_fname = "config.toml"

# Initialize Ray
ray.init()

# Load necessary components
nai = NAI.from_toml(config_fname)
agents = CEMAgent.from_toml(config_fname)

# Initialize the seeker agent
seeker0 = seekerAgent("127.0.0.1:14551", 0)  # Connect to your vehicle
seekers = [seeker0]

# Takeoff location and home location
takeoff_location = LocationGlobalRelative(-35.35898964, 149.16463160, 100)

# Takeoff all drones
while True:
    if not seekers[0].vehicle.armed:
        seekers[0].arm_and_takeoff()
        seekers[0].mode = VehicleMode("GUIDED")
        time.sleep(1)

    seekers[0].vehicle.simple_goto(takeoff_location)
    if seekers[0].vehicle.location.global_relative_frame.alt >= 90:
        print("Drone taken off")
        break

# Main control loop
while True:
    # Generate waypoints based on your search region
    plans = seekers[0].generate_plans(nai)  # Assuming this method returns waypoints

    for waypoint in plans:
        print("Navigating to waypoint:", waypoint)
        seekers[0].vehicle.simple_goto(LocationGlobalRelative(waypoint[0], waypoint[1], 100))  # Altitude can be adjusted

        # Wait until the drone reaches the waypoint
        while True:
            current_location = seekers[0].vehicle.location.global_relative_frame
            distance_to_waypoint = get_distance_meters(current_location, LocationGlobalRelative(waypoint[0], waypoint[1], 100))
            print("Distance to waypoint:", distance_to_waypoint)
            if distance_to_waypoint < 10:  # Adjust threshold as needed
                print("Reached waypoint:", waypoint)
                break
            time.sleep(1)

# Make sure to close connections properly
for seeker in seekers:
    seeker.vehicle.close()
