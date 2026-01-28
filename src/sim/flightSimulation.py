"""
flightSimulation.py - Main Trajectory Simulation Engine
=======================================================

Purpose:
    Orchestrates the complete rocket trajectory simulation from liftoff to landing.
    Integrates physics models, control algorithms, and state machine to produce
    time-series flight data. Loads configuration from JSON files and outputs results
    for analysis and visualization.

Key Functions:
    - run_simulation(): Main simulation loop
        - Initializes rocket parameters from config
        - Propagates trajectory using RK4 integration
        - Calls airbrake controller at control frequency (10 Hz)
        - Detects events (burnout, deployment, apogee)
        - Returns time-series arrays of flight data
        
    - runge_kutta(): Numerical integration (TODO: Move to ad_controller.py)
    
Simulation Flow:
    1. Load configuration (mass, motor, geometry, target apogee)
    2. Initialize state [altitude=0, velocity=0] at t=0
    3. Time-stepping loop (dt = 0.1s default):
        a. Update mass (propellant depletion during boost)
        b. Compute acceleration (thrust - drag - gravity) / mass
        c. Integrate state using RK4
        d. Check deployment conditions (post-burnout, velocity < 411 m/s)
        e. Call airbrake controller if active
        f. Log data (time, altitude, velocity, accel, deployment)
    4. Terminate at apogee or timeout
    5. Return trajectory data for plotting

Configuration Parameters (from JSON):
    - Rocket: mass, diameter, burnout_mass
    - Motor: total_impulse, burn_time (calculates avg thrust)
    - Active Drag: target_apogee, deployment_velocity_threshold
    - Simulation: timestep, max_time
    - File Paths: aero data (drag coefficients), motor thrust curves

Dependencies:
    - flightPhysics.get_acceleration() for force calculations
    - aerodynamics.get_drag() for drag force
    - ad_controller.deploy_brakes() for control commands
    - atmosphere.atm_density() for air density

Notes:
    - Currently uses simplified averaged thrust (TODO: load .eng thrust curves)
    - Assumes 1D vertical flight (no horizontal motion or wind)
    - TODO: Make fully config-driven instead of hardcoded parameters
    - TODO: Integrate with state machine instead of event flags
"""
import numpy as np
from src.flightSoftware.ad_controller import AirbrakeController
from src.sim.flightPhysics import *
import time as timer

# Propogates a rocket flight state forward by dt seconds using 4th order Runge Kutta
def runge_kutta(state, accel_consts, drag_args, dt):
    altitude = state[0]
    velocity = state[1]

    k1_velocity = state[1] # k1 = f(y0, t0)
    k1_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + 1/2*k1_velocity*dt
    state[1] = velocity + 1/2*k1_acceleration*dt

    k2_velocity = state[1] # k2 = f(y0+(k1 * dt/2), t0+(dt/2))
    k2_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + 1/2*k2_velocity*dt
    state[1] = velocity + 1/2*k2_acceleration*dt

    k3_velocity = state[1] # k3 = f(y0+(k2 * dt/2), t0+(dt/2))
    k3_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + 1/2 * k3_velocity*dt
    state[1] = velocity + 1/2 * k3_acceleration*dt

    k4_velocity = state[1] # k4 = f(y0+(k3*dt), t0+dt)
    k4_acceleration = get_acceleration(state, accel_consts, drag_args)

    # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    state[0] = altitude + 1/6*(k1_velocity + 2*k2_velocity + 2*k3_velocity + k4_velocity)*dt
    state[1] = velocity + 1/6*(k1_acceleration + 2*k2_acceleration + 2*k3_acceleration + k4_acceleration)*dt

    return state

def runge_kutta_2d(state, accel_consts, drag_args, dt):
    """
    Propagates 2D rocket state forward by dt seconds using 4th order Runge Kutta.
    
    State: [altitude, vy, horizontal_distance, vx]
    Derivatives: [vy, ay, vx, ax]
    
    Args:
        state: [y, vy, x, vx]
        accel_consts: [mass, thrust, gravity]
        drag_args: [cd_array, percent_deploy, diameter]
        dt: timestep
        
    Returns:
        Updated state [y, vy, x, vx]
    """
    y = state[0]
    vy = state[1]
    x = state[2]
    vx = state[3]
    
    # k1 = f(state_0, t_0)
    ax1, ay1 = get_acceleration_2d(state, accel_consts, drag_args)
    k1 = [vy, ay1, vx, ax1]
    
    # k2 = f(state_0 + k1*dt/2, t_0 + dt/2)
    state_k1 = [
        y + 0.5 * k1[0] * dt,
        vy + 0.5 * k1[1] * dt,
        x + 0.5 * k1[2] * dt,
        vx + 0.5 * k1[3] * dt
    ]
    ax2, ay2 = get_acceleration_2d(state_k1, accel_consts, drag_args)
    k2 = [state_k1[1], ay2, state_k1[3], ax2]
    
    # k3 = f(state_0 + k2*dt/2, t_0 + dt/2)
    state_k2 = [
        y + 0.5 * k2[0] * dt,
        vy + 0.5 * k2[1] * dt,
        x + 0.5 * k2[2] * dt,
        vx + 0.5 * k2[3] * dt
    ]
    ax3, ay3 = get_acceleration_2d(state_k2, accel_consts, drag_args)
    k3 = [state_k2[1], ay3, state_k2[3], ax3]
    
    # k4 = f(state_0 + k3*dt, t_0 + dt)
    state_k3 = [
        y + k3[0] * dt,
        vy + k3[1] * dt,
        x + k3[2] * dt,
        vx + k3[3] * dt
    ]
    ax4, ay4 = get_acceleration_2d(state_k3, accel_consts, drag_args)
    k4 = [state_k3[1], ay4, state_k3[3], ax4]
    
    # Combine: state_new = state_0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    y_new = y + (1/6) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) * dt
    vy_new = vy + (1/6) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) * dt
    x_new = x + (1/6) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) * dt
    vx_new = vx + (1/6) * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) * dt
    
    return [y_new, vy_new, x_new, vx_new]


def run_simulation(rocketConfig, cd_array, airbrakes_enabled):
    """
    Args:
        use_2d: If True, use 2D simulation; if False, use 1D
    """
    dt = rocketConfig["simulation_parameters"]["time_step"]
    target_apogee_AGL = rocketConfig["active_drag_system"]["target_apogee_AGL"]
    ground_level = rocketConfig["simulation_parameters"]["launch_altitude"]
    target_apogee = ground_level + target_apogee_AGL

    brakeFaceArea = rocketConfig["active_drag_system"]["brake_face_area"]
    
    airbrake_controller = AirbrakeController(target_apogee, rocketConfig)
    
    burnout_time = rocketConfig["simulation_parameters"]["burnout_time"]
    burnout_vy = rocketConfig["simulation_parameters"]["burnout_verticalVelocity"]
    burnout_vx = rocketConfig["simulation_parameters"]["burnout_horizontalVelocity"]
    burnout_altitude = rocketConfig["simulation_parameters"]["burnout_altitude"]
    burnout_acceleration = rocketConfig["simulation_parameters"]["burnout_acceleration"]
    
    burnout_mass = rocketConfig["mass_properties"]["burnout_mass"]
    diameter = rocketConfig["dimensions"]["diameter"]
    gravity = 9.80665
    
    n = 0
    time_array = [burnout_time]
    altitude = [burnout_altitude]
    vertical_velocity_array = [burnout_vy]
    horizontal_velocity_array = [burnout_vx]
    horizontal_position_array = [0]
    acceleration = [burnout_acceleration]
    percent_deploy = [0]
    deploy_angle = [0]
    brake_force_array = [0]
    
    accel_consts = [burnout_mass, 0, gravity]  # thrust = 0 after burnout
    drag_args = [cd_array, 0, diameter, brakeFaceArea]
    
    deployment_started = False
    
    ### Simulation loop
        # 2D simulation
    state = [burnout_altitude, burnout_vy, 0, burnout_vx]
    
    while state[1] >= 0:  # While vy >= 0
        state = runge_kutta_2d(state, accel_consts, drag_args, dt)
        
        n += 1
        time_array.append(n * dt + burnout_time)
        altitude.append(state[0])
        vertical_velocity_array.append(state[1])
        horizontal_position_array.append(state[2])
        horizontal_velocity_array.append(state[3])
        
        v_total = np.sqrt(state[1]**2 + state[3]**2)
        ax, ay = get_acceleration_2d(state, accel_consts, drag_args)
        a_total = np.sqrt(ax**2 + ay**2)
        acceleration.append(ay)
        
        # Control
        if airbrakes_enabled:
            # For controller, use vertical velocity state
            deploymentPercent, brakeForce = airbrake_controller.update(
                time_array[n], state, accel_consts, drag_args, dt
            )
        else:
            deploymentPercent = 0
            brakeForce = 0
        
        percent_deploy.append(deploymentPercent)
        deploy_angle.append(np.degrees(np.arcsin(deploymentPercent / 100.0)))  # Approximate angle
        brake_force_array.append(brakeForce)

        drag_args[1] = deploymentPercent
        
        if not deployment_started and deploymentPercent > 0:
            deployment_started = True
            print(f"\n\n--- Airbrake Deployment Initiated ---")
            print(f"Brake Deployment Velocity (vertical): {state[1]:.0f} m/s")
            print(f"Brake Deployment Velocity (horizontal): {state[3]:.0f} m/s")
            print(f"Brake Deployment Altitude: {state[0]:.0f} m")
            print(f"--------------------------------------\n\n")
    
    output = np.array([time_array, altitude, vertical_velocity_array, horizontal_velocity_array, horizontal_position_array, acceleration, percent_deploy, deploy_angle, brake_force_array])
    
    return output