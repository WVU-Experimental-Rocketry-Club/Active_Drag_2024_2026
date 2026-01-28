"""
external_interfaces.py - Hardware Sensor & Actuator Interfaces
==============================================================

Purpose:
    Provides abstraction layer for all hardware sensors and actuators. Enables
    code to run identically in simulation (mock sensors) and on actual flight
    hardware (Tiny 2350 microcontroller) without modification.

Hardware Components:
    
    Sensors:
    - UBLOX M10Q GPS: I2C/UART interface, NMEA parser, position/velocity/time
    - MPL3115A2 Barometer: I2C, pressure/temperature readings, altitude calculation
    - BNO055 IMU: I2C, 9-DOF (accel, gyro, magnetometer), sensor fusion
    
    Actuators:
    - Stepper Motor: Step/direction pins, controls airbrake carriage position
    - Magnetic Encoder: Quadrature encoder, position feedback for closed-loop control
    - Limit Switch: Digital input, carriage homing and end-stop detection

Key Classes:
    - SensorInterface (abstract): Base class defining sensor API
        - read(): Returns current measurement
        - calibrate(): Performs sensor calibration routine
        - is_ready(): Checks if new data available
        
    - GPSInterface: UBLOX M10Q wrapper
    - BarometerInterface: MPL3115A2 wrapper
    - IMUInterface: BNO055 wrapper
    - StepperInterface: Motor control with position tracking
    
    - MockSensor: Simulation mode - returns ideal data from simulation state
    - HardwareSensor: Flight mode - reads from actual I2C/GPIO peripherals

Microcontroller Platform:
    - Tiny 2350: RP2350 chip, dual-core ARM Cortex-M33, 264kB RAM
    - CircuitPython or MicroPython compatible
    - I2C buses for sensors, GPIO for motor control

Dependencies:
    - Simulation: Returns data from physics simulation (no hardware)
    - Hardware: Requires board-specific libraries (busio, digitalio, adafruit_gps, etc.)

Notes:
    - Same code runs in simulation and on hardware (dependency injection pattern)
    - TODO: Implement actual hardware drivers (currently simulation stubs)
    - TODO: Add error handling for sensor failures (I2C timeouts, bad data)
"""

from abc import ABC, abstractmethod
import numpy as np


# ============================================================================
# Abstract Base Classes
# ============================================================================

class SensorInterface(ABC):
    """Abstract base class for all sensors"""
    
    @abstractmethod
    def read(self):
        """Read current sensor measurement"""
        pass
    
    @abstractmethod
    def calibrate(self):
        """Perform sensor calibration"""
        pass
    
    @abstractmethod
    def is_ready(self):
        """Check if new data is available"""
        pass


class ActuatorInterface(ABC):
    """Abstract base class for all actuators"""
    
    @abstractmethod
    def command(self, value):
        """Send command to actuator"""
        pass
    
    @abstractmethod
    def get_position(self):
        """Get current actuator position"""
        pass


# ============================================================================
# Mock Sensors (Simulation Mode)
# ============================================================================

class MockGPS(SensorInterface):
    """Simulated GPS sensor - returns data from simulation state"""
    
    def __init__(self, noise_std={'position': 10.0, 'velocity': 0.5}):
        self.noise_std = noise_std
        self.sim_state = None
        self.last_update_time = -1.0
        self.update_rate = 1.0  # Hz
        
    def set_sim_state(self, time, altitude, velocity, latitude=0.0, longitude=0.0):
        """Update with simulation ground truth"""
        self.sim_state = {
            'time': time,
            'altitude': altitude,
            'velocity': velocity,
            'latitude': latitude,
            'longitude': longitude
        }
    
    def read(self):
        """Returns GPS data with realistic noise and update rate"""
        if self.sim_state is None:
            return None
        
        # Add realistic GPS noise
        altitude_noise = np.random.normal(0, self.noise_std['position'])
        velocity_noise = np.random.normal(0, self.noise_std['velocity'])
        
        return {
            'position': np.array([
                self.sim_state['altitude'] + altitude_noise,
                self.sim_state['latitude'],
                self.sim_state['longitude']
            ]),
            'velocity': np.array([
                self.sim_state['velocity'] + velocity_noise,
                0.0, 0.0
            ]),
            'altitude': self.sim_state['altitude'] + altitude_noise,
            'latitude': self.sim_state['latitude'],
            'longitude': self.sim_state['longitude'],
            'time': self.sim_state['time']
        }
    
    def calibrate(self):
        """GPS requires no calibration"""
        pass
    
    def is_ready(self):
        """GPS updates at 1 Hz"""
        if self.sim_state is None:
            return False
        dt = self.sim_state['time'] - self.last_update_time
        return dt >= (1.0 / self.update_rate)


class MockBarometer(SensorInterface):
    """Simulated barometer - converts altitude to pressure"""
    
    def __init__(self, noise_std=1.5):
        self.noise_std = noise_std  # meters
        self.sim_altitude = 0.0
        self.sim_time = 0.0
        self.update_rate = 10.0  # Hz
        
    def set_sim_state(self, time, altitude):
        """Update with simulation altitude"""
        self.sim_time = time
        self.sim_altitude = altitude
    
    def read(self):
        """Returns altitude and pressure with noise"""
        altitude_noise = np.random.normal(0, self.noise_std)
        noisy_altitude = self.sim_altitude + altitude_noise
        
        # Convert altitude to pressure (ISA model)
        pressure = 101325.0 * (1 - 2.25577e-5 * noisy_altitude) ** 5.25588
        temperature = 288.15 - 0.0065 * noisy_altitude
        
        return {
            'altitude': noisy_altitude,
            'pressure': pressure,
            'temperature': temperature
        }
    
    def calibrate(self):
        """Barometer calibration sets ground level offset"""
        pass
    
    def is_ready(self):
        """Barometer updates at 10 Hz"""
        return True


class MockIMU(SensorInterface):
    """Simulated IMU - 9-DOF sensor with noise and bias"""
    
    def __init__(self, noise_std={'accel': 0.1, 'gyro': 0.01}):
        self.noise_std = noise_std
        self.sim_accel = np.array([0.0, 0.0, -9.80665])
        self.sim_time = 0.0
        self.update_rate = 100.0  # Hz
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        
    def set_sim_state(self, time, acceleration):
        """Update with simulation acceleration"""
        self.sim_time = time
        self.sim_accel = np.array([0.0, 0.0, acceleration - 9.80665])
    
    def read(self):
        """Returns IMU data with noise and bias"""
        accel_noise = np.random.normal(0, self.noise_std['accel'], size=3)
        gyro_noise = np.random.normal(0, self.noise_std['gyro'], size=3)
        
        return {
            'acceleration': self.sim_accel + accel_noise + self.accel_bias,
            'accel_x': self.sim_accel[0] + accel_noise[0],
            'accel_y': self.sim_accel[1] + accel_noise[1],
            'accel_z': self.sim_accel[2] + accel_noise[2],
            'gyro': np.array([0.0, 0.0, 0.0]) + gyro_noise
        }
    
    def calibrate(self):
        """Measure and store accelerometer bias"""
        samples = [self.read()['acceleration'] for _ in range(100)]
        self.accel_bias = np.mean(samples, axis=0)
    
    def is_ready(self):
        """IMU updates at 100 Hz"""
        return True


class MockStepper(ActuatorInterface):
    """Simulated stepper motor for airbrake control"""
    
    def __init__(self, max_position=100.0, speed=33.33):
        """
        Args:
            max_position (float): Maximum deployment percentage
            speed (float): Deployment speed (%/second)
        """
        self.max_position = max_position
        self.speed = speed
        self.current_position = 0.0
        self.target_position = 0.0
        self.last_update_time = 0.0
        
    def command(self, target_percent):
        """
        Command airbrake deployment percentage.
        
        Args:
            target_percent (float): Desired deployment (0-100)
        """
        self.target_position = np.clip(target_percent, 0.0, self.max_position)
    
    def update(self, time):
        """Update stepper position based on speed limit"""
        dt = time - self.last_update_time
        if dt <= 0:
            return
        
        # Move toward target at max speed
        error = self.target_position - self.current_position
        max_change = self.speed * dt
        
        if abs(error) <= max_change:
            self.current_position = self.target_position
        else:
            self.current_position += np.sign(error) * max_change
        
        self.last_update_time = time
    
    def get_position(self):
        """Returns current deployment percentage"""
        return self.current_position
    
    def home(self):
        """Reset to zero position (limit switch)"""
        self.current_position = 0.0
        self.target_position = 0.0


# ============================================================================
# Hardware Sensors (Flight Mode - Stubs for now)
# ============================================================================

class HardwareGPS(SensorInterface):
    """UBLOX M10Q GPS interface - hardware implementation stub"""
    
    def __init__(self, i2c_bus=None, uart=None):
        self.i2c_bus = i2c_bus
        self.uart = uart
        # TODO: Initialize actual GPS module
        
    def read(self):
        """Read GPS data from hardware"""
        # TODO: Implement NMEA parsing or UBX protocol
        raise NotImplementedError("Hardware GPS not yet implemented")
    
    def calibrate(self):
        pass
    
    def is_ready(self):
        # TODO: Check for new NMEA sentence or UBX message
        return False


class HardwareBarometer(SensorInterface):
    """MPL3115A2 barometer interface - hardware implementation stub"""
    
    def __init__(self, i2c_bus=None):
        self.i2c_bus = i2c_bus
        # TODO: Initialize MPL3115A2 via I2C
        
    def read(self):
        """Read pressure/temperature from hardware"""
        # TODO: Implement I2C read from MPL3115A2 registers
        raise NotImplementedError("Hardware barometer not yet implemented")
    
    def calibrate(self):
        """Set sea-level reference pressure"""
        # TODO: Implement calibration
        pass
    
    def is_ready(self):
        # TODO: Check data ready bit in status register
        return False


class HardwareIMU(SensorInterface):
    """BNO055 IMU interface - hardware implementation stub"""
    
    def __init__(self, i2c_bus=None):
        self.i2c_bus = i2c_bus
        # TODO: Initialize BNO055 via I2C
        
    def read(self):
        """Read acceleration/gyro/magnetometer from hardware"""
        # TODO: Implement I2C read from BNO055 registers
        raise NotImplementedError("Hardware IMU not yet implemented")
    
    def calibrate(self):
        """Run BNO055 calibration routine"""
        # TODO: Implement calibration sequence
        pass
    
    def is_ready(self):
        return False


class HardwareStepper(ActuatorInterface):
    """Stepper motor with encoder - hardware implementation stub"""
    
    def __init__(self, step_pin=None, dir_pin=None, encoder_pins=None, limit_switch_pin=None):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.encoder_pins = encoder_pins
        self.limit_switch_pin = limit_switch_pin
        # TODO: Initialize GPIO pins
        
    def command(self, target_percent):
        """Command deployment percentage"""
        # TODO: Convert percent to steps and move motor
        raise NotImplementedError("Hardware stepper not yet implemented")
    
    def get_position(self):
        """Read encoder position"""
        # TODO: Read quadrature encoder
        raise NotImplementedError("Hardware stepper not yet implemented")
    
    def home(self):
        """Move to limit switch"""
        # TODO: Implement homing routine
        pass