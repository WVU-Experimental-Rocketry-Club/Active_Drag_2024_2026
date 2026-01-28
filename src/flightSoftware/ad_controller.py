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
    
    def __init__(self, target_apogee, rocketConfig, config=None):
        """
        Initialize airbrake controller.
        
        Args:
            target_apogee (float): Target apogee altitude (m)
            rocketConfig (dict): Rocket configuration parameters
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
        self.airbrakeMachThreshold = rocketConfig["active_drag_system"]["deployment_conditions"]["maximum_mach"]
        self.deployment_time = config.get('deployment_time', 3.0)
        self.error_threshold = config.get('error_threshold', 5.0)
        self.control_frequency = config.get('control_frequency', 100.0)
        self.max_deployment = config.get('max_deployment', 100.0)
        self.kp = config.get('kp', .02)  # Proportional gain
        self.maxBrakeForece = rocketConfig["active_drag_system"]["max_brake_force"]  # Newtons
        
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
            return self.current_deployment, getBrakeDrag(drag_args[0], state[1], self.current_deployment, state[0], drag_args[3])
        if self.target_apogee - state[0] > 2000:
            self.error_threshold = 5
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
            deployment_change = self.deployment_rate * (time - self.last_update_time) * min(abs(error) * self.kp, 1.0)
            
            if error > 0:  # Predicted apogee too high
                #only deploy brakes if resulting drag is within max brake force
                if getBrakeDrag(drag_args[0], state[1], self.current_deployment + deployment_change, state[0], drag_args[3]) < self.maxBrakeForece:
                    self.current_deployment = min(self.current_deployment + deployment_change, self.max_deployment)
                if self.current_deployment > 100:
                    pass
            else:  # Predicted apogee too low
                self.current_deployment = max(self.current_deployment - deployment_change, 0.0)
        
        self.last_update_time = time
        return self.current_deployment, getBrakeDrag(drag_args[0], state[1], self.current_deployment, state[0], drag_args[3])
    
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
            pred_state = runge_kutta_2d(pred_state, accel_consts, pred_drag_args, dt)
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