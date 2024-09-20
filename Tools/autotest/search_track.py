from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit as dk
import time
import numpy as np
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters
import ray
import toml
from algorithms import NAI, Target, CEMAgent, reward_fn  # Import your classes/functions

# Load configuration
config_fname = "/path/to/your/config.toml"

# Initialize Ray
ray.init()

# Load necessary components
nai = NAI.from_toml(config_fname)
targets = Target.from_toml(config_fname)
agents = CEMAgent.from_toml(config_fname)

# Initialize beliefs for targets
beliefs = [None for _ in targets]

n_agents = 1
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seekers = [seeker0]

# Takeoff location and home location
takeoff_location = dk.LocationGlobalRelative(-35.35898964, 149.16463160, 100)
home_lat_lon = np.array([-35.36341649, 149.16525123])
target_lat_lon = (-35.36290851, 149.15882125)

# Takeoff all drones
takeoff_status = False
while True:
    if takeoff_status:
        break

    for idx in range(n_agents):
        if not seekers[idx].vehicle.armed:
            seekers[idx].arm_and_takeoff()
            seekers[idx].mode = VehicleMode("GUIDED")
            time.sleep(1)

    for idx in range(n_agents):
        seekers[idx].vehicle.simple_goto(takeoff_location)
        if seekers[idx].vehicle.location.global_relative_frame.alt >= 90:
            takeoff_status = True
        else:
            takeoff_status = False
            break

print("All drones taken off")

# Main control loop
while True:
    # Update target location dynamically
    target_lat_lon = (
        target_lat_lon[0] + np.random.uniform(-0.0001, 0.0001),
        target_lat_lon[1] + np.random.uniform(-0.0001, 0.0001)
    )
    target_location = dk.LocationGlobalRelative(target_lat_lon[0], target_lat_lon[1], 100)

    # Generate plans for the agent based on the current target
    plans = seekers[0].generate_plans(target_location)  # Assuming this generates plans for the active drone

    # Compute rewards using the integrated reward function
    rewards = reward_fn([plans], nai, beliefs, seekers)

    # Update the agent based on rewards
    seekers[0].update_based_on_rewards(rewards[0])  # Only update the active drone

    # Move the active drone towards the target location
    seekers[0].vehicle.simple_goto(target_location)

    # Calculate distance to target
    dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                             seekers[0].vehicle.location.global_relative_frame.lat,
                             seekers[0].vehicle.location.global_relative_frame.lon)

    agent_pos_NED = np.array([dN, dE, -seekers[0].vehicle.location.global_relative_frame.alt])
    target_pos_NED = np.array([0, 0, -100.])  # Update this logic to reflect the actual target position in NED

    distance_to_target = get_distance_meters(agent_pos_NED, target_pos_NED)
    if distance_to_target <= 50.:
        print("Active drone has reached the target.")
        break

    time.sleep(5)  # Wait for the next update

# Make sure to close connections properly
for seeker in seekers:
    seeker.vehicle.close()
