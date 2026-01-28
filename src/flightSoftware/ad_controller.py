"""
ad_controller.py - Active Drag (Airbrake) Controller
====================================================

Purpose:
    Implements the predictive control algorithm for active airbrake deployment.
    Continuously adjusts airbrake angle to achieve target apogee by computing
    predicted apogee and modulating deployment percentage.

Key Functions:
    - deploy_brakes(): Main control algorithm - predicts apogee via forward simulation,
                       compares to target, adjusts deployment percentage proportionally
    - runge_kutta(): RK4 numerical integrator for trajectory propagation
                     Integrates state [altitude, velocity] with derivatives [velocity, acceleration]

Control Strategy:
    - Predictive control: Simulates trajectory forward to apogee (velocity = 0)
    - Proportional adjustment: deployment_rate ∝ (predicted_apogee - target_apogee)
    - Rate limiting: 3-second full deployment time (smooths actuator motion)
    - Error threshold: ±5 meters (prevents oscillation near target)
    
Hardware Constraints:
    - Stepper motor on lead screw carriage
    - Magnetic encoder for position feedback
    - Limit switch for homing
    - Physical deployment range: 0-100%

Dependencies:
    - flightPhysics.get_acceleration() for force calculations
    - aerodynamics.get_drag() for drag force
    
Notes:
    - Currently operates in idealized simulation mode
    - TODO: Add GPS-only constraint mode for realistic sensor limitations
    - TODO: Implement alternative control strategies (PID, bang-bang)
"""

import numpy as np
from src.core.atmosphere import *
from src.core.aerodynamics import *
from src.sim.flightPhysics import *
import time as timer

class AirbrakeController:
    """
    Predictive controller for active airbrake system.
    
    Predicts apogee by forward-simulating trajectory and adjusts deployment
    percentage to achieve target altitude.
    
    Attributes:
        target_apogee (float): Desired apogee altitude (m)
        deployment_time (float): Time to fully deploy airbrakes (s)
        error_threshold (float): Acceptable error from target (m)
        current_deployment (float): Current deployment percentage (0-100)
        control_frequency (float): Controller update rate (Hz)
    """
    
    def __init__(self, target_apogee, config=None):
        """
        Initialize airbrake controller.
        
        Args:
            target_apogee (float): Target apogee altitude (m)
            config (dict, optional): Controller configuration
                - airbrakeMachThreshold: Maximum mach for deployment (default 2.0)
                - deployment_time: Time for full deployment (s), default 3.0
                - error_threshold: Acceptable error (m), default 5.0
                - control_frequency: Update rate (Hz), default 10.0
                - max_deployment: Maximum deployment percent, default 100.0
        """
        self.target_apogee = target_apogee
        
        # Controller parameters
        config = config or {}
        self.airbrakeMachThreshold = config.get('airbrakeMachThreshold', 2.0)
        self.deployment_time = config.get('deployment_time', 3.0)
        self.error_threshold = config.get('error_threshold', 5.0)
        self.control_frequency = config.get('control_frequency', 100.0)
        self.max_deployment = config.get('max_deployment', 100.0)
        
        # State
        self.current_deployment = 0.0
        self.predicted_apogee = 0.0
        self.last_update_time = 0.0
        
        # Calculate deployment rate (percent per second)
        self.deployment_rate = 100.0 / self.deployment_time
        
    def update(self, time, state, accel_consts, drag_args, dt=0.1):
        """
        Update controller and compute new deployment command.
        
        Args:
            time (float): Current simulation time (s)
            state (np.ndarray): Current state [altitude, velocity]
            accel_consts (list): Constants for acceleration calculation [mass, thrust, gravity]
            drag_args (list): Arguments for drag calculation [cd_array, percent_deploy, diameter]
            dt (float): Integration timestep for prediction (s)
            
        Returns:
            float: Updated deployment percentage (0-100)
        """
        # Check if enough time has passed for controller update
        if time - self.last_update_time < (1.0 / self.control_frequency):
            return self.current_deployment
        if self.target_apogee - state[0] > 1000:
            self.error_threshold = 25
            curr_target_apogee = self.target_apogee + 25
            prediction_dt = 1
        else:
            self.error_threshold = 5
            curr_target_apogee = self.target_apogee
            prediction_dt = dt
        # Predict apogee using forward simulation
        # time_counter = timer.time()
        self.predicted_apogee = self._predict_apogee(state, accel_consts, drag_args, prediction_dt)
        # lastloop_time = timer.time() - time_counter
        # print(f"Loop Time: {lastloop_time*1000:.2f} ms")
        # Calculate error
        error = self.predicted_apogee - curr_target_apogee
        
        # Adjust deployment if outside error threshold
        if abs(error) > self.error_threshold and state[1] < self.airbrakeMachThreshold * speed_of_sound(state[0]):
            # Proportional control: more error = faster deployment change
            # Positive error (too high) → increase deployment (more drag)
            # Negative error (too low) → decrease deployment (less drag)
            deployment_change = self.deployment_rate * (time - self.last_update_time)
            
            if error > 0:  # Predicted apogee too high
                self.current_deployment = min(self.current_deployment + deployment_change, 
                                             self.max_deployment)
            else:  # Predicted apogee too low
                self.current_deployment = max(self.current_deployment - deployment_change, 0.0)
        
        self.last_update_time = time
        return self.current_deployment
    
    def _predict_apogee(self, state, accel_consts, drag_args, dt):
        """
        Predict apogee by simulating forward to velocity = 0.
        
        Args:
            state (np.ndarray): Current state [altitude, velocity]
            accel_consts (list): [mass, thrust, gravity]
            drag_args (list): [cd_array, percent_deploy, diameter]
            dt (float): Integration timestep (s)
            
        Returns:
            float: Predicted apogee altitude (m)
        """
        # Make a copy to avoid modifying actual state
        pred_state = state.copy()
        
        # Update drag_args with current deployment
        pred_drag_args = drag_args.copy()
        pred_drag_args[1] = self.current_deployment
        
        # Simulate forward until velocity reaches zero (apogee)
        max_iterations = 10000  # Safety limit
        iteration = 0
        
        while pred_state[1] > 0 and iteration < max_iterations:
            pred_state = runge_kutta(pred_state, accel_consts, pred_drag_args, dt)
            iteration += 1
        
        return pred_state[0]  # Return predicted altitude
    
    def get_deployment(self):
        """Returns current deployment percentage"""
        return self.current_deployment
    
    def get_predicted_apogee(self):
        """Returns most recent apogee prediction"""
        return self.predicted_apogee
    
    def reset(self):
        """Reset controller to initial state"""
        self.current_deployment = 0.0
        self.predicted_apogee = 0.0
        self.last_update_time = 0.0


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