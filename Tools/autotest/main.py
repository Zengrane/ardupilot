"""
@author: DDN
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit as dk
import time
import math
import cv2
from time import sleep
from pymavlink import mavutil
import numpy as np
from numpy import rad2deg as deg
import math
from dk_agent_plane import seekerAgent
from geolocation import calc_east_north, get_distance_meters, ned_to_azel
import signal
import sys
import csv

n_agents = 3
# Connect to the vehicles
seeker0 = seekerAgent("127.0.0.1:14551", 0)
seeker1 = seekerAgent("127.0.0.1:14561", 1)
seeker2 = seekerAgent("127.0.0.1:14571", 2)
seekers = [seeker0, seeker1, seeker2]

# Location where drone should take off and loiter
takeoff_location = dk.LocationGlobalRelative(-35.35898964, 149.16463160, 100)
# Home location of the drone
home_lat_lon = np.array([-35.36341649, 149.16525123])
# Defines home location of the drone representing a geographic location
# relative to the drone 
home_location = dk.LocationGlobalRelative(home_lat_lon[0], home_lat_lon[1], 100)

# Initial target location
target_lat_lon = (-35.36290851, 149.15882125) 
target_location = dk.LocationGlobalRelative(target_lat_lon[0], target_lat_lon[1], 100)

# Target movement speed
movement_speed = 0.0001  # Latitude/Longitude change per update
# How often target position is recalculated
update_interval = 5  # seconds
movement_direction = 0  # direction in degrees from north (0 is north, 90 is east, etc.)

# Convert direction to radians
direction_radians = math.radians(movement_direction)

# Convert speed to latitude/longitude change per update
lat_change_per_update = movement_speed * math.cos(direction_radians)
lon_change_per_update = movement_speed * math.sin(direction_radians)

# Assign home location to each seeker
for idx in range(n_agents):
    seekers[idx].home_lat_lon = home_lat_lon

# Takeoff all drones
takeoff_status = False
while True:
    if takeoff_status:
        break

    # Iterates over each drone
    for idx in range(n_agents):
        if not seekers[idx].vehicle.armed:  # check if takeoff occurred
            seekers[idx].arm_and_takeoff()
            seekers[idx].mode = VehicleMode("GUIDED")
            sleep(1)

    for idx in range(n_agents):
        seekers[idx].vehicle.simple_goto(takeoff_location)
        print(" Altitude: ", seekers[idx].vehicle.location.global_relative_frame.alt)
        if seekers[idx].vehicle.location.global_relative_frame.alt >= 100 * 0.9:
            print(f"UAV{idx + 1} Reached target altitude")
            takeoff_status = True
        else:
            takeoff_status = False
            break

    time.sleep(1)

print("all drones taken off")

# An array that tracks which drones have reached the target
agents_done = np.zeros(shape=n_agents, dtype=bool)

# Main loop to navigate the drones
while True:

    # Update the target location dynamically in a straight line
    target_lat_lon = (
        target_lat_lon[0] + lat_change_per_update,
        target_lat_lon[1] + lon_change_per_update
    )
    target_location = dk.LocationGlobalRelative(target_lat_lon[0], target_lat_lon[1], 100)
    print(f"Updated target location to: {target_lat_lon}")

    # Calculate east and north coordinates relative to the home location
    dE_target_from_home, dN_target_from_home = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                                               target_lat_lon[0], target_lat_lon[1])
    target_pos_NED_init = np.array([dN_target_from_home, dE_target_from_home, -100.])

    for idx in range(n_agents):
        dE, dN = calc_east_north(home_lat_lon[0], home_lat_lon[1],
                                 seekers[idx].vehicle.location.global_relative_frame.lat,
                                 seekers[idx].vehicle.location.global_relative_frame.lon)

        agent_pos_NED = np.array([dN, dE, -seekers[idx].vehicle.location.global_relative_frame.alt])
        target_pos_NED = target_pos_NED_init

        seekers[idx].vehicle.simple_goto(target_location)

        distance_to_target = get_distance_meters(agent_pos_NED, target_pos_NED)
        if distance_to_target <= 50.:
            if not agents_done[idx]:
                agents_done[idx] = True
                break

    if all(agents_done):
        break

    time.sleep(update_interval)
