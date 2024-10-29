# Project
Multi-Drone Target Tracking and Exploration

# Description
This project involves a multi-drone system that performs target tracking and area exploration in a simulated environment using the ArduPlane SITL simulator and DroneKit. Each drone is capable of detecting targets within a set range and prioritizes target tracking when a target is within range. If no targets are detected, drones explore a defined search area.

# Functionality
Target Detection: Drones detect targets within a defined detection range (100 meters). Each target’s position is updated at intervals to simulate movement.
Tracking Priority: When a target is detected, the drone prioritizes tracking the target. Tracking continues for a set duration before the drone switches to exploring if the target goes out of range.
Exploration Mode: If no targets are detected, the drones explore the specified search area (defined by lat_min, lat_max, lon_min, and lon_max).
Multi-threading: Threads are used to handle target location updates, target detection, and waypoint navigation concurrently for each drone.

# Steps
Several versions of the implementation are available to run

To launch a single agent tracking multiple targets with focus ratio, run:
launch_arduplane.sh 1
python mainup.py

To launch multiple agents with multiple targets, run:
launch_arduplane.sh 3
python main4.py



