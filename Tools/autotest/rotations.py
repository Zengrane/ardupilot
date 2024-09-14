#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 14 21:55:01 2022

Algorithms based on the book "Small Unmanned Aircraft - Theory and Practice"

@author: Jia Ming Kok
"""


import numpy as np
from numpy import deg2rad as rad
from numpy import cos,sin
from numpy.linalg import inv



def calc_R_nv_b(roll, pitch, yaw):
    """
    Calculates rotation matrix for converting xyz in body axis to NED axis
    Input is roll, pitch and yaw of the aircraft in degrees       
    """
    phi = rad(roll)
    theta = rad(pitch)
    psi = rad(yaw)

    R_nv_b = np.array([[cos(theta)*cos(psi), sin(theta)*sin(phi)*cos(psi)-cos(phi)*sin(psi), sin(theta)*cos(phi)*cos(psi)+sin(phi)*sin(psi)],
                            [cos(theta)*sin(psi), sin(theta)*sin(phi)*sin(psi)+cos(phi)*cos(psi), sin(theta)*cos(phi)*sin(psi)-sin(phi)*cos(psi)],
                            [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)]])
    

    return R_nv_b


def calc_R_b_c(pan, tilt):
    """
    Input is pan and tilt of the camera in degrees
    """
    theta = rad(pan)
    phi = rad(tilt)

    R_b_c = np.array([[cos(theta)*cos(phi), -sin(theta), cos(theta)*sin(phi)],
          [sin(theta)*cos(phi), cos(theta), sin(theta)*sin(phi)],
          [-sin(phi), 0, cos(phi)]])

    return R_b_c


def calc_body_to_NED(x_b, y_b, z_b, roll, pitch, yaw):
    """    
    Converts from body to NED axis
    Input is roll, pitch and yaw of the aircraft in degrees, xyz in body axes in m   
    Returns xyz_NED
    """    
    R_nv_b = calc_R_nv_b(roll, pitch, yaw)
    X_b = np.array([x_b,y_b,z_b])    
    X_NED = np.matmul(R_nv_b, X_b)
    
    return X_NED



def calc_R_v2_b(roll):
    """
    The transformation from the vehicle-2 frame to the body frame
    p^b = R_v2^b * p^v2

    Parameters
    ----------
    roll : in degrees    
    pitch : in degrees        
    yaw : in degrees
        
    Returns
    -------
    Matrix R_{v2}^{b}

    """
    phi = rad(roll)
    
    R_v2_b = np.array([
        [ 1 , 0         , 0         ],
        [ 0 , cos(phi)  , sin(phi)  ],
        [ 0 , -sin(phi) , cos(phi)  ]
        ])
    
    return R_v2_b



def calc_R_v_b(roll, pitch, yaw):
    """
    The transformation from the vehicle frame to the body frame

    Parameters
    ----------
    roll : in degrees    
    pitch : in degrees        
    yaw : in degrees
        
    Returns
    -------
    Matrix R_{v}^{b}

    """
    
    phi = rad(roll)
    theta = rad(pitch)
    psi = rad(yaw)


    R_v_b = np.array([
        [cos(theta)*cos(psi) , cos(theta)*sin(psi) , -sin(theta)],
        [sin(theta)*sin(phi)*cos(psi) - cos(phi)*sin(psi) , sin(theta)*sin(phi)*sin(psi) + cos(phi)*cos(psi) , cos(theta)*sin(phi)],
        [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) , sin(theta)*cos(phi)*sin(psi) - sin(phi)*cos(psi) , cos(theta)*cos(phi)]
        ])
    

    return R_v_b



def calc_R_b_g(alpha_az, alpha_el):
    """
    Rotation from the body to the gimbal frame
    
    alpha_az: Gimbal azimuth angle (deg)
    alpha_el: Gimbal elevation angle (deg)
    
    Returns matrix R_{b}^{g}
    """
    az = rad(alpha_az)
    el = rad(alpha_el)
    
    R_b_g = np.array([[ cos(el)*cos(az) , cos(el)*sin(az)   , -sin(el)  ],
                      [ -sin(az)        , cos(az)           , 0         ],
                      [ sin(el)*cos(az) , sin(el)*sin(az)   , cos(el)   ]])


    return R_b_g

def R_g_c():
    """
    Rotation from the gimbal frame to the camera frame
    
    Returns
    -------
    Matrix R_{g}^{c}

    """
    
    R_g_c = np.array([[ 0 , 1 , 0 ],
                      [ 0 , 0 , 1 ],
                      [ 1 , 0 , 0 ]])
    
    return R_g_c
    
    
def inv_mat(mat):
    """
    Invert Matrix
    """

    return inv(mat)
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    