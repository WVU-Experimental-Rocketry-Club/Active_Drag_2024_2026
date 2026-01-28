"""
datalogger.py - Flight Data Recording
=====================================

Purpose:
    Records flight telemetry data to CSV files for post-flight analysis, plotting,
    and validation. Captures time-series data of all state variables, sensor readings,
    and control outputs throughout the flight.

Key Functions:
    - DataLogger class: Manages data buffers and file I/O
    - log_state(): Records [time, altitude, velocity, acceleration, deployment_pct]
    - log_sensors(): Records raw sensor readings (GPS, barometer, IMU)
    - log_control(): Records control commands and predicted apogee
    - write_to_file(): Flushes buffer to CSV file
    - close(): Finalizes logging and closes file

Data Format:
    CSV Output Columns:
    - time (s), altitude (m), velocity (m/s), acceleration (m/s²)
    - deployment_percent (%), predicted_apogee (m), target_apogee (m)
    - gps_lat (deg), gps_lon (deg), gps_alt (m), gps_velocity (m/s)
    - pressure (Pa), imu_accel_z (m/s²), temperature (K)
    
    File Naming: flight_data_YYYYMMDD_HHMMSS.csv

Usage Modes:
    - Simulation: Logs perfect state variables for validation
    - Hardware: Logs actual sensor readings for telemetry replay
    - Comparison: Logs both true state and sensor estimates side-by-side

Dependencies:
    - csv module for file writing
    - datetime for timestamps

Notes:
    - Buffer size configurable (default: write every 100 samples)
    - High-frequency logging (10-100 Hz) generates large files
    - TODO: Add binary logging format for efficiency
    - TODO: Implement real-time telemetry streaming (future: radio downlink)
"""

import csv
from datetime import datetime
from pathlib import Path


class DataLogger:
    """
    Flight data recording system for telemetry logging.
    
    Attributes:
        file_path (Path): Path to output CSV file
        headers (list): Column names for CSV
        buffer (list): Data buffer for batch writing
        buffer_size (int): Number of samples before flushing to disk
        file_handle: Open file handle
        csv_writer: CSV writer object
        is_logging (bool): Whether logging is active
    """
    
    def __init__(self, output_dir='data/flight_logs', buffer_size=100, filename=None):
        """
        Initialize data logger.
        
        Args:
            output_dir (str): Directory for output files
            buffer_size (int): Samples to buffer before writing
            filename (str, optional): Custom filename, otherwise auto-generated with timestamp
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"flight_data_{timestamp}.csv"
        
        self.file_path = self.output_dir / filename
        self.buffer_size = buffer_size
        self.buffer = []
        self.file_handle = None
        self.csv_writer = None
        self.is_logging = False
        
        # Define CSV headers
        self.headers = [
            'time', 'altitude', 'velocity', 'acceleration',
            'deployment_percent', 'predicted_apogee', 'target_apogee',
            'gps_altitude', 'gps_latitude', 'gps_longitude', 'gps_velocity',
            'pressure', 'temperature', 'imu_accel_x', 'imu_accel_y', 'imu_accel_z',
            'flight_phase', 'mass', 'drag_force', 'thrust'
        ]
    
    def start(self):
        """Open log file and begin recording"""
        if not self.is_logging:
            self.file_handle = open(self.file_path, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=self.headers)
            self.csv_writer.writeheader()
            self.is_logging = True
            self.buffer = []
    
    def stop(self):
        """Flush remaining data and close log file"""
        if self.is_logging:
            self._flush_buffer()
            if self.file_handle:
                self.file_handle.close()
            self.is_logging = False
    
    def log_state(self, time, altitude, velocity, acceleration, deployment_percent=0.0):
        """
        Log basic flight state.
        
        Args:
            time (float): Simulation time (s)
            altitude (float): Altitude (m)
            velocity (float): Vertical velocity (m/s)
            acceleration (float): Vertical acceleration (m/s²)
            deployment_percent (float): Airbrake deployment percentage (0-100)
        """
        data = {
            'time': time,
            'altitude': altitude,
            'velocity': velocity,
            'acceleration': acceleration,
            'deployment_percent': deployment_percent
        }
        self._add_to_buffer(data)
    
    def log_full_state(self, time, state_dict):
        """
        Log complete flight state with all available data.
        
        Args:
            time (float): Simulation time (s)
            state_dict (dict): Dictionary containing all state variables
                Keys should match self.headers
        """
        data = {'time': time}
        data.update(state_dict)
        self._add_to_buffer(data)
    
    def log_control(self, time, predicted_apogee, target_apogee, deployment_percent):
        """
        Log control system data.
        
        Args:
            time (float): Simulation time (s)
            predicted_apogee (float): Controller's predicted apogee (m)
            target_apogee (float): Target apogee (m)
            deployment_percent (float): Commanded deployment (0-100)
        """
        data = {
            'time': time,
            'predicted_apogee': predicted_apogee,
            'target_apogee': target_apogee,
            'deployment_percent': deployment_percent
        }
        self._add_to_buffer(data)
    
    def log_sensors(self, time, sensor_data):
        """
        Log raw sensor readings.
        
        Args:
            time (float): Simulation time (s)
            sensor_data (dict): Dictionary of sensor readings
                {'gps': {...}, 'barometer': {...}, 'imu': {...}}
        """
        data = {'time': time}
        
        if 'gps' in sensor_data:
            gps = sensor_data['gps']
            data.update({
                'gps_altitude': gps.get('altitude'),
                'gps_latitude': gps.get('latitude'),
                'gps_longitude': gps.get('longitude'),
                'gps_velocity': gps.get('velocity')
            })
        
        if 'barometer' in sensor_data:
            baro = sensor_data['barometer']
            data.update({
                'pressure': baro.get('pressure'),
                'temperature': baro.get('temperature')
            })
        
        if 'imu' in sensor_data:
            imu = sensor_data['imu']
            data.update({
                'imu_accel_x': imu.get('accel_x'),
                'imu_accel_y': imu.get('accel_y'),
                'imu_accel_z': imu.get('accel_z')
            })
        
        self._add_to_buffer(data)
    
    def _add_to_buffer(self, data):
        """Add data to buffer and flush if needed"""
        if not self.is_logging:
            return
        
        # Ensure all headers exist with None defaults
        row = {header: data.get(header) for header in self.headers}
        self.buffer.append(row)
        
        if len(self.buffer) >= self.buffer_size:
            self._flush_buffer()
    
    def _flush_buffer(self):
        """Write buffer to file"""
        if self.csv_writer and self.buffer:
            self.csv_writer.writerows(self.buffer)
            self.file_handle.flush()
            self.buffer = []
    
    def get_file_path(self):
        """Returns path to log file"""
        return str(self.file_path)
    
    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()