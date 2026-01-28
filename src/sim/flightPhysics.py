"""
flightPhysics.py - Core Physics Calculations
============================================

Purpose:
    Consolidates all physics calculations for rocket trajectory simulation.
    Computes forces (thrust, drag, gravity), accelerations, and derived quantities
    (Mach number, dynamic pressure, kinetic energy).

Key Functions:
    - get_acceleration(state, accel_consts, drag_args): 
        Computes net acceleration from all forces
        Returns: a = (F_thrust - F_drag) / m - g
        
    - get_drag(cd_array, velocity, percent_deploy, altitude, diameter):
        Calculates aerodynamic drag force (NOTE: Currently in aerodynamics.py)
        Formula: F_D = 0.5 * rho * V^2 * C_D * A
        
    - get_mass(time, total_mass, propellant_mass, burn_time):
        Returns current mass accounting for propellant depletion
        Linear decrease during burn: m(t) = m_total - (propellant_mass/burn_time)*t
        
    - get_thrust(time, thrust_curve, burn_time):
        Returns thrust at current time (future: interpolate from .eng file)
        Currently: Constant average thrust during burn, 0 after burnout

Physical Models:
    - Forces:
        * Thrust: F_T from motor (time-dependent from thrust curve)
        * Drag: F_D = 0.5 * rho * V^2 * C_D * A (velocity and altitude dependent)
        * Gravity: F_g = m * g (constant g = 9.80665 m/s²)
        
    - Equations of Motion:
        * Net force: F_net = F_thrust - F_drag - F_gravity
        * Acceleration: a = F_net / m
        * 1D kinematics: dh/dt = v, dv/dt = a
        
    - Mass Model:
        * Boost phase: m(t) = m_0 - (m_propellant / t_burn) * t
        * Coast phase: m = m_burnout (constant)

Dependencies:
    - aerodynamics.cd_interp() for drag coefficient
    - atmosphere.atm_density() for air density

Constants:
    - g = 9.80665 m/s² (standard gravity, Earth)
    - Future: Add Earth radius for gravity variation with altitude

Notes:
    - Assumes 1D vertical flight (no pitch, yaw, roll dynamics)
    - Uses simplified thrust model (TODO: implement thrust curve interpolation)
    - TODO: Add 6-DOF equations for full 3D trajectory
    - TODO: Implement variable gravity: g(h) = g_0 * (R_E / (R_E + h))^2
"""

import numpy as np
from src.core.atmosphere import atm_density
from src.core.aerodynamics import getTotalDrag

def get_acceleration(state, accel_consts, drag_args):
    altitude = state[0]
    velocity = state[1]
    mass = accel_consts[0]
    gravity = accel_consts[1]
    cd_array = drag_args[0]
    percent_deploy = drag_args[1]
    diameter = drag_args[2]
    brakefacearea = drag_args[3]
    drag = getTotalDrag(cd_array, velocity, percent_deploy, altitude, diameter, brakefacearea)
    acceleration = -drag/mass - gravity
    return acceleration

def get_acceleration_2d(state, accel_consts, drag_args):
    """
    Calculate 2D accelerations [ax, ay] from forces.
    
    Args:
        state: [altitude, vertical_velocity, horizontal_distance, horizontal_velocity]
        accel_consts: [mass, thrust, gravity]
        drag_args: [cd_array, percent_deploy, diameter, brakefacearea]
        
    Returns:
        [ax, ay]
    """
    altitude = state[0]
    vy = state[1]
    vx = state[3]
    
    mass = accel_consts[0]
    thrust = accel_consts[1]
    gravity = accel_consts[2]
    cd_array = drag_args[0]
    percent_deploy = drag_args[1]
    diameter = drag_args[2]
    brakefacearea = drag_args[3]
    # Total velocity magnitude
    v_total = np.sqrt(vx**2 + vy**2)
    
    # Avoid division by zero
    if v_total < 0.01:
        return [0.0, thrust / mass - gravity]
    
    # Drag force magnitude
    drag_force = getTotalDrag(cd_array, v_total, percent_deploy, altitude, diameter, brakefacearea)
    
    # Drag components (opposes velocity direction)
    # Negative because drag opposes motion
    drag_x = -drag_force * (vx / v_total)
    drag_y = -drag_force * (vy / v_total)
    
    # Accelerations
    ax = drag_x / mass
    ay = (thrust + drag_y) / mass - gravity
    
    return [ax, ay]


def get_acceleration_1d(state, accel_consts, drag_args):
    """
    Calculate 1D vertical acceleration (existing function, keep for compatibility).
    """
    altitude = state[0]
    vy = state[1]
    
    mass = accel_consts[0]
    thrust = accel_consts[1]
    gravity = accel_consts[2]
    cd_array = drag_args[0]
    percent_deploy = drag_args[1]
    diameter = drag_args[2]
    brakefacearea = drag_args[3]
    drag = getTotalDrag(cd_array, abs(vy), percent_deploy, altitude, diameter, brakefacearea)
    acceleration = (thrust - drag) / mass - gravity
    
    return acceleration