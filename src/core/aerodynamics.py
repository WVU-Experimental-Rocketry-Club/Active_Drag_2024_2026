"""
aerodynamics.py - Aerodynamic Coefficients & Drag Force Calculations
=====================================================================

Purpose:
    Handles all aerodynamic force calculations for the rocket simulation.
    Provides drag coefficient interpolation based on Mach number and airbrake
    deployment percentage, and computes total drag force.

Key Functions:
    - cd_interp(): Bilinear interpolation of drag coefficients from lookup tables
                   across Mach (0.2-2.0) and deployment (0%, 33%, 100%)
    - get_drag(): Calculates aerodynamic drag force using dynamic pressure and
                  drag coefficient: F_D = 0.5 * rho * V^2 * C_D * A

Dependencies:
    - numpy for array operations
    - atmosphere.atm_density() for air density at altitude

Data Format:
    - cd_array: 2D numpy array [mach_index, deploy_index] from drag coefficient tables
    - Input from Excel/CSV files in data/aero/ directory

Notes:
    - Currently uses fixed speed of sound (340 m/s)
    - TODO: Implement altitude-dependent Mach number calculation
"""

import numpy as np
import pandas as pd
from src.core.atmosphere import atm_density, atm_pressure, atm_temperature, speed_of_sound, atm_properties
import time as timer

# def cd_interp(cd_array, velocity, altitude, percent_deploy):
#     mach_num = velocity/speed_of_sound(altitude)
#     mach_pts = [0.2, 0.4, 0.6, 0.9, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
#     deploy_pts = [0, 33, 100] # airbrakes off, half, or all on
#     # Find closest mach point
#     for mach_val in mach_pts: 
#         if mach_val > mach_num:
#             i = mach_pts.index(mach_val) - 1
#             break
#         else:
#             i = len(mach_pts) - 1
#     # Find closest deploy val
#     for deploy_val in deploy_pts:
#         if deploy_val > percent_deploy:
#             j = deploy_pts.index(deploy_val) - 1
#             break
#         else:
#             j = len(deploy_pts) - 2
    
#     if (i == -1) or (i == len(mach_pts)-1): #if smallest or largest mach value
#         if i == -1:
#             i = 0
#         f1 = cd_array[i,j] #mach, closest smaller deploy value
#         f2 = cd_array[i,j+1] #mach, closest greater deploy value
#     else:
#         f1 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j]
#         f2 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j+1] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j+1]
#     # interpolate CD values
#     cd = (deploy_pts[j+1] - percent_deploy)/(deploy_pts[j+1] - deploy_pts[j])*f1 + (percent_deploy - deploy_pts[j])/(deploy_pts[j+1] - deploy_pts[j])*f2
#     return cd


# def get_drag(cd_array, velocity, percent_deploy, altitude, diameter):
#     V = velocity
#     A = np.pi*(diameter/2)**2
#     rho = atm_density(altitude)
#     cd = cd_interp(cd_array, velocity, altitude, percent_deploy)
#     drag = 1/2*rho*V**2*cd*A
#     return drag

def getTotalDrag(cd_array, velocity, percent_deploy, altitude, diameter, brakeFaceArea):
    pressure, temp, rho, c = atm_properties(altitude)
    gamma = 1.4  # Ratio of specific heats for air
    mach_num = velocity/c

    q = pressure * (1 + (((gamma - 1)/2) * mach_num**2))**(gamma/(gamma - 1)) - pressure # total pressure - pressure  # dynamic pressure

    # percent deploy is linear in projected area, so the flap angle is the arcsin.
    # The brake Cd fit is a function of flap angle (0.35 stowed to 1.15 at 90 deg,
    # derived from the 0.85 average for folding brakes in Michael Farha's thesis)
    percent_deploy = np.clip(percent_deploy, 0.0, 100.0)
    deploy_angle = np.degrees(np.arcsin(percent_deploy / 100.0))
    AdeployedBrakes = brakeFaceArea * (percent_deploy / 100.0)
    FbrakeDrag = q * AdeployedBrakes * (0.00889 * deploy_angle + 0.35)

    Arocket = np.pi*(diameter/2)**2
    Rocketdrag = np.interp(mach_num, cd_array['Mach'], cd_array['CD'])
    FrocketDrag = 0.5 * rho * velocity**2 * Rocketdrag * Arocket

    totalDrag = FbrakeDrag + FrocketDrag
    return totalDrag

def getBrakeDrag(cd_array, velocity, percent_deploy, altitude, brakeFaceArea):
    pressure, temp, rho, c = atm_properties(altitude)
    gamma = 1.4  # Ratio of specific heats for air
    mach_num = velocity/c

    q = pressure * (1 + (((gamma - 1)/2) * mach_num**2))**(gamma/(gamma - 1)) - pressure # total pressure - pressure  # dynamic pressure

    # same brake model as getTotalDrag: area linear in percent, Cd fit vs flap angle
    percent_deploy = np.clip(percent_deploy, 0.0, 100.0)
    deploy_angle = np.degrees(np.arcsin(percent_deploy / 100.0))
    AdeployedBrakes = brakeFaceArea * (percent_deploy / 100.0)
    FbrakeDrag = q * AdeployedBrakes * (0.00889 * deploy_angle + 0.35)

    return FbrakeDrag

def getRocketBodyDrag(cd_array, velocity, altitude, diameter):
    pressure, temp, rho, c = atm_properties(altitude)
    mach_num = velocity/c

    Arocket = np.pi*(diameter/2)**2
    Rocketdrag = np.interp(mach_num, cd_array['Mach'], cd_array['CD'])
    FrocketDrag = 0.5 * rho * velocity**2 * Rocketdrag * Arocket
    return FrocketDrag

def getBrakeCd(deployPercent, mach):
    a = 0