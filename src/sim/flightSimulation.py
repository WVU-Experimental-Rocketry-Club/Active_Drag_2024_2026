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

def run_simulation(cd_array, target_apogee, dt):

    airbrake_controller = AirbrakeController(target_apogee, {'airbrakeMachThreshold': 1.45})
    ### Setup/initialization
    burnout_time = 9.901 # seconds
    burnout_verticalVelocity = 507.79 # m/s
    burnout_horizontalVelocity = 83.2 # m/s
    burnout_altitude = 4156.8624 # meters

    burnout_mass = 40.62 # kg
    diameter = 0.158 # m
    gravity = 9.80665 # m/s/s

    n = 0
    time = [burnout_time]
    altitude = [burnout_altitude]
    velocity = [burnout_verticalVelocity]
    acceleration = [0]
    percent_deploy = [0]
    accel_consts = [burnout_mass, gravity]
    drag_args = [cd_array, percent_deploy[0], diameter]

    # Event variables
    deploymentVelocity = 0
    deploymentAltitude = 0
    deployment_started = False

    ### Run to apogee
    while velocity[n] >= 0:
        
        ### Propogate state by dt
        state = [altitude[n], velocity[n]] #prev state alt/vel
        state = runge_kutta(state, accel_consts, drag_args, dt) # propogate to next altitude/velocity
        n = n + 1
        time.append(n*dt + burnout_time) 
        altitude.append(state[0]) #update to new runge kutta altitude
        velocity.append(state[1]) #update to new velocity
        acceleration.append(get_acceleration(state, accel_consts, drag_args))

        ### run active drag controller 

        deploymentPercent = airbrake_controller.update(time[n], state, accel_consts, drag_args, dt)
        percent_deploy.append(deploymentPercent)
        drag_args[1] = deploymentPercent

        if not deployment_started and deploymentPercent > 0:
            deployment_started = True
            deployment_velocity = velocity[n]
            deployment_altitude = altitude[n]
            print(f"Brake Deployment Velocity: {deployment_velocity:.0f} m/s")
            print(f"Brake Deployment Altitude: {deployment_altitude:.0f} m")
            print(f"Target Apogee: {target_apogee} m")

    output = np.array([time, altitude, velocity, acceleration, percent_deploy])
    return output

# base_results = run_simulation(cd_fb, 11000, dt)
# fb_results = run_simulation(cd_fb, target_apogee, dt)

# projected_apogee = base_results[1,-1]
# mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]