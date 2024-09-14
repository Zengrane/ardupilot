#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 29 22:00:50 2022

Contains all the dronekit code required to control the agent

@author: Jia Ming Kok
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit as dk
import time
from time import sleep
from pymavlink import mavutil
import math
import numpy as np
# from pymavlink.dialects.v20.all import MAVLink_acceleration_vector_cmd_message


#########################################################################
################## Dronekit Commands ####################################
#########################################################################


# def acceleration_vector_cmd_encode(frame_id: int, xacc: float, yacc: float,
#                                    zacc: float) -> MAVLink_acceleration_vector_cmd_message:
#     """
#     Guidance acceleration vector commands.

#     frame_id                  : Config type. (type:uint8_t, values:MAV_FRAME)
#     xacc                      : X acceleration. [m/s/s] (type:float)
#     yacc                      : Y acceleration. [m/s/s] (type:float)
#     zacc                      : Z acceleration. [m/s/s] (type:float)

#     """
#     return MAVLink_acceleration_vector_cmd_message(frame_id, xacc, yacc, zacc)



class dk_agent_plane(object):
    def __init__(self, ip):
        self.ip = ip
        self.vehicle = dk.connect(self.ip)


    def arm_and_takeoff(self):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        vehicle = self.vehicle
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        # Plane should arm in AUTO or TAKEOFF
        vehicle.mode = VehicleMode("TAKEOFF")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        time.sleep(10)
        #    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        #
        #    # Wait until the vehicle reaches a safe height before processing the goto
        #    #  (otherwise the command after Vehicle.simple_takeoff will execute
        #    #   immediately).
        vehicle.mode = VehicleMode("GUIDED")

    def arm_and_takeoff_acro(self):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        vehicle = self.vehicle
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        # Plane should arm in AUTO or TAKEOFF
        vehicle.mode = VehicleMode("TAKEOFF")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        time.sleep(10)
        #    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        #
        #    # Wait until the vehicle reaches a safe height before processing the goto
        #    #  (otherwise the command after Vehicle.simple_takeoff will execute
        #    #   immediately).
        vehicle.mode = VehicleMode("GUIDED")

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0, pitch_rate=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                             thrust=0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        vehicle = self.vehicle
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            thrust  # Thrust
        )
        # print('blah!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        vehicle.send_mavlink(msg)


    def send_acceleration_target(self, accel_fwd, accel_right, accel_down, frame=23):
        """Send custom acceleration command"""
        # self.vehicle.message_factory.acceleration_vector_cmd_send(
        #     frame,
        #     accel_fwd,
        #     accel_right,
        #     accel_down,
        #     force_mavlink1=False
        # )
        vehicle = self.vehicle
        # msg = vehicle.message_factory.acceleration_vector_cmd_encode(frame, accel_fwd, accel_right, accel_down)
        msg = acceleration_vector_cmd_encode(frame, accel_fwd, accel_right, accel_down)

        self.vehicle.send_mavlink(msg)


    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0, pitch_rate=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                     thrust=0.5, duration=0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.

        Input is in deg
        """
        self.send_attitude_target(roll_angle, pitch_angle, pitch_rate,
                                  yaw_angle, yaw_rate, False,
                                  thrust=thrust)

        # start = time.time()
        # while time.time() - start < duration:
        #     self.send_attitude_target(roll_angle, pitch_angle,
        #                               yaw_angle, yaw_rate, False,
        #                               thrust)
        #     time.sleep(0.1)
        # # Reset attitude, or it will persist for 1s more due to the timeout
        # self.send_attitude_target(0, 0,
        #                           0, 0, True,
        #                           thrust)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def increase_update_rate(self, msg_id, rate):
        vehicle = self.vehicle
        time_interval = int(1000000 / rate)
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            time_interval,
            0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        

class seekerAgent(dk_agent_plane):
    def __init__(self, ip, SYSID):
        super(seekerAgent, self).__init__(ip)
        self.ip = ip
        self.SYSID = SYSID
        self.target_NED = None
        self.home_lat_lon = None