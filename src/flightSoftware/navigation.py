"""
navigation.py - State Estimation & Position/Velocity Tracking
=============================================================

Purpose:
    Provides position and velocity estimates for the flight computer using GPS,
    barometer, and IMU sensor fusion. Simulates realistic sensor constraints including
    update rates, latency, and noise to test controller robustness.

Key Functions:
    - get_position(): Returns current [altitude, latitude, longitude] estimate
    - get_velocity(): Returns current [vertical_velocity, horizontal_velocity, heading]
    - update_sensors(): Fuses GPS (1 Hz), barometer (10 Hz), IMU (100 Hz) data
    - predict_apogee(): Uses current state to predict peak altitude (for controller)

Sensor Characteristics:
    Hardware:
    - UBLOX M10Q GPS: 1 Hz update, ±5m horizontal, ±10m vertical accuracy, 200ms latency
    - MPL3115A2 Barometer: 10 Hz capable, ±1.5m altitude precision, pressure → altitude conversion
    - BNO055 IMU: 100 Hz accelerometer, bias drift, noise (for vertical accel integration)
    
    Simulation Constraints:
    - GPS-only mode: Forces controller to use only GPS data (realistic flight limitation)
    - Sensor fusion mode: Combines all sensors using Kalman filter or complementary filter
    - Noise injection: Adds realistic measurement errors to test robustness

State Estimation Strategies:
    1. GPS-only: Direct position/velocity from GPS (sparse, delayed)
    2. Barometer + IMU: High-rate altitude estimate via sensor fusion
    3. Full Kalman: Optimal fusion of all sensors with uncertainty quantification

Dependencies:
    - atmosphere.atm_pressure() for barometer altitude conversion
    - external_interfaces for hardware sensor readings

Notes:
    - Critical for testing controller under realistic sensor constraints
    - TODO: Implement complementary filter (simple) vs Kalman filter (optimal)
    - TODO: Add GPS dropout simulation (signal loss scenarios)
"""

import numpy as np
from enum import Enum, auto


class NavigationMode(Enum):
    """Navigation sensor fusion modes"""
    GPS_ONLY = auto()          # Only GPS position/velocity (realistic flight constraint)
    BARO_IMU = auto()          # Barometer + IMU fusion (high rate, no GPS)
    FULL_FUSION = auto()       # All sensors with Kalman filter
    SIMULATION_TRUTH = auto()  # Perfect state from simulation (testing only)


class NavigationSystem:
    """
    State estimation system for position and velocity tracking.
    
    Attributes:
        mode (NavigationMode): Current sensor fusion mode
        position (np.ndarray): [altitude, latitude, longitude] in [m, deg, deg]
        velocity (np.ndarray): [vertical_vel, north_vel, east_vel] in m/s
        acceleration (np.ndarray): [vertical_accel, north_accel, east_accel] in m/s²
        state_covariance (np.ndarray): State uncertainty (for Kalman filter)
        last_gps_time (float): Timestamp of last GPS update
        last_baro_time (float): Timestamp of last barometer update
        last_imu_time (float): Timestamp of last IMU update
    """
    
    def __init__(self, mode=NavigationMode.FULL_FUSION, sensor_interfaces=None):
        """
        Initialize navigation system.
        
        Args:
            mode (NavigationMode): Sensor fusion mode
            sensor_interfaces (dict): Dictionary of sensor interface objects
                {'gps': GPSInterface, 'barometer': BarometerInterface, 'imu': IMUInterface}
        """
        self.mode = mode
        self.sensors = sensor_interfaces or {}
        
        # State vectors
        self.position = np.array([0.0, 0.0, 0.0])  # [alt, lat, lon]
        self.velocity = np.array([0.0, 0.0, 0.0])  # [v_z, v_n, v_e]
        self.acceleration = np.array([0.0, 0.0, 0.0])  # [a_z, a_n, a_e]
        
        # Uncertainty estimates (for Kalman filter)
        self.state_covariance = np.eye(9) * 100.0  # Initial high uncertainty
        
        # Sensor timestamps
        self.last_gps_time = -1.0
        self.last_baro_time = -1.0
        self.last_imu_time = -1.0
        
        # Sensor update rates (Hz)
        self.gps_rate = 1.0
        self.baro_rate = 10.0
        self.imu_rate = 100.0
        
        # Biases and calibration
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.baro_offset = 0.0
        
    def update(self, time, true_state=None):
        """
        Update state estimate with latest sensor data.
        
        Args:
            time (float): Current time (s)
            true_state (dict, optional): True state for simulation mode
                {'altitude': float, 'velocity': float, 'acceleration': float}
                
        Returns:
            dict: Updated state estimate
                {'position': np.ndarray, 'velocity': np.ndarray, 'acceleration': np.ndarray}
        """
        if self.mode == NavigationMode.SIMULATION_TRUTH:
            # Use perfect state from simulation (testing only)
            if true_state:
                self.position[0] = true_state.get('altitude', 0.0)
                self.velocity[0] = true_state.get('velocity', 0.0)
                self.acceleration[0] = true_state.get('acceleration', 0.0)
                
        elif self.mode == NavigationMode.GPS_ONLY:
            # Only use GPS updates (sparse, realistic)
            if self._should_update_gps(time):
                self._update_from_gps(time)
                
        elif self.mode == NavigationMode.BARO_IMU:
            # Fuse barometer and IMU (no GPS)
            if self._should_update_imu(time):
                self._update_from_imu(time)
            if self._should_update_baro(time):
                self._update_from_baro(time)
                
        elif self.mode == NavigationMode.FULL_FUSION:
            # Full Kalman filter with all sensors
            if self._should_update_imu(time):
                self._kalman_predict(time)
                self._update_from_imu(time)
            if self._should_update_baro(time):
                self._kalman_update_baro(time)
            if self._should_update_gps(time):
                self._kalman_update_gps(time)
        
        return self.get_state()
    
    def get_state(self):
        """Returns current state estimate"""
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),
            'altitude': self.position[0],
            'vertical_velocity': self.velocity[0]
        }
    
    def get_position(self):
        """Returns current position [altitude, latitude, longitude]"""
        return self.position.copy()
    
    def get_velocity(self):
        """Returns current velocity [vertical, north, east]"""
        return self.velocity.copy()
    
    def get_altitude(self):
        """Returns current altitude estimate (m)"""
        return self.position[0]
    
    def get_vertical_velocity(self):
        """Returns current vertical velocity (m/s)"""
        return self.velocity[0]
    
    def predict_apogee(self, gravity=9.80665):
        """
        Predict apogee altitude using current state.
        Simple kinematic prediction: h_apogee = h + v²/(2g)
        
        Args:
            gravity (float): Gravitational acceleration (m/s²)
            
        Returns:
            float: Predicted apogee altitude (m)
        """
        h = self.position[0]
        v = self.velocity[0]
        
        if v <= 0:
            return h  # Already falling
        
        # Kinematic prediction (ignores drag, conservative)
        predicted_apogee = h + (v ** 2) / (2 * gravity)
        return predicted_apogee
    
    def _should_update_gps(self, time):
        """Check if GPS should update based on rate"""
        dt = 1.0 / self.gps_rate
        return (time - self.last_gps_time) >= dt
    
    def _should_update_baro(self, time):
        """Check if barometer should update based on rate"""
        dt = 1.0 / self.baro_rate
        return (time - self.last_baro_time) >= dt
    
    def _should_update_imu(self, time):
        """Check if IMU should update based on rate"""
        dt = 1.0 / self.imu_rate
        return (time - self.last_imu_time) >= dt
    
    def _update_from_gps(self, time):
        """Update state from GPS sensor"""
        if 'gps' in self.sensors:
            gps_data = self.sensors['gps'].read()
            self.position = gps_data['position']
            self.velocity = gps_data['velocity']
            self.last_gps_time = time
    
    def _update_from_baro(self, time):
        """Update altitude from barometer"""
        if 'barometer' in self.sensors:
            baro_data = self.sensors['barometer'].read()
            self.position[0] = baro_data['altitude'] - self.baro_offset
            self.last_baro_time = time
    
    def _update_from_imu(self, time):
        """Update acceleration from IMU"""
        if 'imu' in self.sensors:
            imu_data = self.sensors['imu'].read()
            self.acceleration = imu_data['acceleration'] - self.accel_bias
            self.last_imu_time = time
    
    def _kalman_predict(self, time):
        """Kalman filter prediction step (TODO: Implement full Kalman)"""
        pass
    
    def _kalman_update_baro(self, time):
        """Kalman filter barometer measurement update (TODO: Implement)"""
        pass
    
    def _kalman_update_gps(self, time):
        """Kalman filter GPS measurement update (TODO: Implement)"""
        pass
    
    def calibrate(self, ground_altitude=0.0):
        """
        Calibrate sensors on the launch pad.
        
        Args:
            ground_altitude (float): Known ground altitude (m)
        """
        if 'barometer' in self.sensors:
            baro_data = self.sensors['barometer'].read()
            self.baro_offset = baro_data['altitude'] - ground_altitude
        
        if 'imu' in self.sensors:
            # Measure IMU bias (should read -g on pad)
            samples = []
            for _ in range(100):
                imu_data = self.sensors['imu'].read()
                samples.append(imu_data['acceleration'])
            self.accel_bias = np.mean(samples, axis=0)
            self.accel_bias[0] += 9.80665  # Subtract expected -g
    
    def reset(self):
        """Reset navigation system to initial state"""
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.state_covariance = np.eye(9) * 100.0
        self.last_gps_time = -1.0
        self.last_baro_time = -1.0
        self.last_imu_time = -1.0