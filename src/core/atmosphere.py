"""
atmosphere.py - Atmospheric Model & Properties
==============================================

Purpose:
    Provides atmospheric properties (density, temperature, pressure) as functions
    of altitude. Used for calculating aerodynamic forces and Mach number.

Key Functions:
    - atm_density(altitude): Returns air density in kg/m³ using ISA standard atmosphere
                             Formula: rho = rho_0 * (T_b - (h - h_b)*L_b / T_b)^((g*M)/(R*L_b) - 1)
    - atm_temperature(altitude): Returns temperature in Kelvin (for Mach calculation)
    - atm_pressure(altitude): Returns pressure in Pa (for sensor simulation)
    - speed_of_sound(altitude): Returns local speed of sound: a = sqrt(gamma * R * T)

Dependencies:
    - numpy for mathematical operations

Atmospheric Model:
    - ISA (International Standard Atmosphere) troposphere model
    - Valid from sea level to ~11 km
    - Constants: rho_0 = 1.2250 kg/m³, T_0 = 288.15 K, L_b = 0.0065 K/m
    
Future Enhancements:
    - Integration of weather balloon data from data/weather/ directory
    - Real atmospheric profiles vs. standard atmosphere
    - Wind modeling (speed and direction vs altitude)
"""
import numpy as np
import pandas as pd
import time as timer

_weather_data = None
_use_weather = False

def load_weather_data(weather_data):
    """
    Set weather data for atmospheric calculations.
    
    Args:
        weather_data: DataFrame or path to CSV file
    """
    global _weather_data, _use_weather
    
    if isinstance(weather_data, pd.DataFrame):
        _weather_data = weather_data
    else:
        _weather_data = pd.read_csv(weather_data)
    
    _use_weather = True
    print(f"Loaded weather data with {len(_weather_data)} altitude points")

def use_isa_model():
    """Switch back to ISA standard atmosphere"""
    global _use_weather
    _use_weather = False

def atm_pressure(altitude):
    if _use_weather and _weather_data is not None:
        return np.interp(altitude, _weather_data['geopotential height_m'], _weather_data['pressure_hPa']) * 100  # convert hPa to Pa
    else:
        # ISA model
        P0 = 101325  # Pa at sea level
        Tb = 288.15  # K at sea level   
        Lb = 0.0065  # K/m temperature lapse rate
        hb = 0       # m base altitude (sea level)
        g0 = 9.80665 # m/s² standard gravity
        R = 8.3144598# J/(mol·K) universal gas constant
        M = 0.0289644# kg/mol molar mass of Earth's air
        h = altitude
        T = Tb - (h - hb) * Lb
        P = P0 * (T / Tb) ** (g0 * M / (R * Lb))
        return P

def speed_of_sound(altitude):
    gamma = 1.4  # Ratio of specific heats for air
    R = 287.05   # Specific gas constant for dry air (J/(kg·K))
    T = atm_temperature(altitude)
    a = (gamma * R * T) ** 0.5
    return a

def atm_density(altitude):
    if _use_weather and _weather_data is not None:
        pressure = atm_pressure(altitude)  # in Pa
        temperature = atm_temperature(altitude)  # in K
        R_specific = 287.058  # J/(kg·K) for dry air
        rho = pressure / (R_specific * temperature)
        return rho
    else:
        # ISA model
        rho_b = 1.2250  # kg/m³ at sea level
        Tb = 288.15     # K at sea level
        Lb = 0.0065     # K/m temperature lapse rate
        hb = 0          # m base altitude (sea level)
        g0 = 9.80665    # m/s² standard gravity
        R = 8.3144598   # J/(mol·K) universal gas constant
        M = 0.0289644   # kg/mol molar mass of Earth's air

        h = altitude
        rho = rho_b * ((Tb - (h - hb) * Lb) / Tb) ** ((g0 * M) / (R * Lb) - 1)
        return rho
def atm_temperature(altitude):  
    if _use_weather and _weather_data is not None:
        return np.interp(altitude, _weather_data['geopotential height_m'], _weather_data['temperature_C']) + 273.15  # Convert °C to K
    else:
        # ISA model
        Tb = 288.15     # K at sea level
        Lb = 0.0065     # K/m temperature lapse rate
        hb = 0          # m base altitude (sea level)

        h = altitude
        T = Tb - (h - hb) * Lb
        return T