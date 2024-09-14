#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 12:29:23 2023

Module for geolocation

@author: jmk
"""

import rotations
import numpy as np
from rotations import inv_mat
from MAVProxy.modules.lib import mp_util
import math
from math import sqrt, sin, cos, acos, atan2


def calc_geolocate_coeff(attitude, cam_orient):
    """
    The equation for Geolocation is
    P_obj - P_mav = L ( R_b^i * R_g^b * R_c^g * lc )
    
    
    This calculates (R_b^i * R_g^b * R_c^g)
    
    Inputs:
        attitude  = (roll,pitch,yaw) in degrees
        cam_orient = (alpha_az, alpha_el) in degrees
        cam_px = (px,py,f)

    Return:
        3x3 Matrix
    """

    R_v_b = rotations.calc_R_v_b(*attitude)
    R_b_g = rotations.calc_R_b_g(*cam_orient)
    R_g_c = rotations.R_g_c()

    return inv_mat(np.matmul(R_g_c, np.matmul(R_b_g, R_v_b)))


def calc_geolocate_coeff_in_v2(roll, cam_orient):
    """
    The equation for Geolocation in the vehicle 2 frame is
    P_obj - P_mav = L ( R_b^v2 * R_g^b * R_c^g * lc )
    
    
    This calculates (R_b^v2 * R_g^b * R_c^g)
    
    Inputs:
        roll  = roll in degrees
        cam_orient = (alpha_az, alpha_el) in degrees
        cam_px = (px,py,f)

    Return:
        3x3 Matrix
    """

    R_v2_b = rotations.calc_R_v2_b(roll)
    R_b_g = rotations.calc_R_b_g(*cam_orient)
    R_g_c = rotations.R_g_c()

    return inv_mat(np.matmul(R_g_c, np.matmul(R_b_g, R_v2_b)))


def calc_unit_l_c(cam_px):
    """
    Calc unit normal vector l_c = 1/sqrt(px^2+py^2+f^2)*(px,py,f)

    Parameters
    ----------
    cam_px : (px,py,f) in units of pixels
        f = (Max pixel width)/(2*tan(FoV/2))

    Returns
    -------
    3x1 vector

    """

    return cam_px / np.linalg.norm(cam_px)


def calc_target_bearing(attitude, cam_orient, cam_px):
    """
    Calculate the bearing to the target

    Parameters
    ----------
    attitude : (roll,pitch,yaw) in degrees
    cam_orient : (alpha_az, alpha_el) in degrees
    cam_px : (px,py,f) in units of pixels

    Returns
    -------
    target bearing in degrees (i.e. degrees from north) relative to north

    """

    pn_pe_pd = np.matmul(calc_geolocate_coeff(attitude, cam_orient),
                         calc_unit_l_c(cam_px))

    bearing = np.rad2deg(np.arctan2(pn_pe_pd[1], pn_pe_pd[0]))

    if bearing < 0:
        bearing = 360. + bearing

    return bearing


def calc_target_bearing_in_v2(roll, cam_orient, cam_px):
    """
    Calculate the bearing to the target in the vehicle-2 frame
    

    Parameters
    ----------
    roll : roll in degrees
    cam_orient : (alpha_az, alpha_el) in degrees
    cam_px : (px,py,f) in units of pixels

    Returns
    -------
    target bearing in degrees (i.e. degrees from north) in the vehicle-2 frame

    """

    pn_pe_pd = np.matmul(calc_geolocate_coeff_in_v2(roll, cam_orient),
                         calc_unit_l_c(cam_px))

    bearing = np.rad2deg(np.arctan2(pn_pe_pd[1], pn_pe_pd[0]))

    if bearing < 0:
        bearing = 360. + bearing

    return bearing


def calculate_location(attitude, cam_orient, cam_px, altitude):
    """
    Perform the geolocation here. Returns the relative pposition 
    between the target and the UAV

    Parameters
    ----------
    attitude : (roll,pitch,yaw) in degrees
    cam_orient : (alpha_az, alpha_el) in degrees
    cam_px : (px,py,f) in units of pixels
    altitude : altitude in m

    Returns
    -------
    3 x 1 vector (dP_n, dP_e, dAlt)

    """

    tmp = np.matmul(calc_geolocate_coeff(attitude, cam_orient),
                    calc_unit_l_c(cam_px))

    loc = tmp * altitude / tmp[2]

    return loc


def calc_inertial_to_pixel_coeff(attitude, cam_orient):
    """
    Calculate the coefficient  to transform position to a pixel
    l_c_hat = L*R_g^c * R_b^g * R_i^b * deltaP 
    Coeff = R_g^c * R_b^g * R_i^b

    Parameters
    ----------
    Inputs:
        attitude  = (roll,pitch,yaw) in degrees
        cam_orient = (alpha_az, alpha_el) in degrees

    Returns
    -------
        3x3 Matrix

    """
    R_v_b = rotations.calc_R_v_b(*attitude)
    R_b_g = rotations.calc_R_b_g(*cam_orient)
    R_g_c = rotations.R_g_c()

    return np.matmul(R_g_c, np.matmul(R_b_g, R_v_b))


def calc_pixel(attitude, cam_orient, target_ned, f):
    """
    Calculate the pixel based on the location of the target from the UAV

    Parameters
    ----------
    attitude : (roll,pitch,yaw) in degrees
    cam_orient : (alpha_az, alpha_el) in degrees
    target_ned : (pn , pe , pd)

    Returns
    -------
    (px, py, f)

    """

    coeff = calc_inertial_to_pixel_coeff(attitude, cam_orient)
    l_c_hat = np.matmul(coeff, target_ned) / np.linalg.norm(target_ned)
    F = f / l_c_hat[2]

    px = int(l_c_hat[0] * F), int(l_c_hat[1] * F), int(f)

    return px


def calc_east_north(lat1, lon1, lat2, lon2):
    dist = mp_util.gps_distance(lat1, lon1, lat2, lon2)
    bearing = mp_util.gps_bearing(lat1, lon1, lat2, lon2)
    dN = dist * math.cos(np.radians(bearing))
    dE = dist * math.sin(np.radians(bearing))

    return dE, dN


def calc_GPS_EN(lat, lon, east, north):
    """
    :param lat: latitude
    :param lon: longitude
    :param east: in m
    :param north: in m
    :return: new lat lon in degrees
    """

    return mp_util.gps_offset(lat, lon, east, north)


def get_distance_meters(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def ned_to_azel(ned):
    # ned is a tuple containing the velocity vector components in NED format
    north, east, down = ned
    azimuth = math.atan2(east, north)
    elevation = math.atan2(-1 * down, math.sqrt(north ** 2 + east ** 2))
    return math.degrees(azimuth), math.degrees(elevation)


def dot(u, v):
    return sum([coord1 * coord2 for coord1, coord2 in zip(u, v)])


def length(v):
    return sqrt(sum([coord ** 2 for coord in v]))


def get_lead_angle(uav_position, target_position, uav_velocity):
    relative_position = target_position - uav_position
    sigma = acos(dot(relative_position, uav_velocity) / (length(relative_position) * length(uav_velocity)))
    return math.degrees(sigma)  # in degrees


# %%

if __name__ == '__main__':
    attitude = 0, 0, 0
    cam_orient = 0, -15.
    cam_fov_xy = 62.2, 48.8
    Wp, Hp = 3280, 2464
    f = Wp / (2 * np.tan(np.deg2rad(cam_fov_xy[0]) / 2))

    target_ned = 200, 20, 100

    px = calc_pixel(attitude, cam_orient, target_ned, f)

    print(px)

    cam_px = px[0], px[1], f

    bearing = calc_target_bearing(attitude, cam_orient, cam_px)

    print(bearing)

    altitude = target_ned[2]
    loc = calculate_location(attitude, cam_orient, cam_px, altitude)

    print(loc)
