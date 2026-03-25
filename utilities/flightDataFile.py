# generate_aero_header.py
import numpy as np
import json
# 1. Load your data (Replace this with your actual CSV loading)
# Example: data = np.genfromtxt('cd_curve.csv', delimiter=',')
# Here we simulate some transonic drag rise data

config_path = 'configs/competition_rocket_2026.json'
with open(config_path, 'r') as f:
    config = json.load(f)

rocket_mass = config["mass_properties"]["burnout_mass"]  # kg
rocket_diameter = config["dimensions"]["diameter"]  # meters
rocket_reference_area = np.pi * (rocket_diameter / 2) ** 2  # m^2
brake_face_area = config["active_drag_system"]["brake_face_area"]  # m^2
target_apogee_AGL = config["active_drag_system"]["target_apogee_AGL"]  # meters
maximumDeploymentMach = config["active_drag_system"]["deployment_conditions"]["maximum_mach"]  # dimensionless

aero_file_path = config["file_paths"]["aero_file"]
weather_file_path = config["file_paths"]["weather_file"]

aero_data = np.genfromtxt(aero_file_path, delimiter=',', skip_header=1)
mach_mask = aero_data[:,0] <= 2.5
mach_points = aero_data[mach_mask, 0]
cd_points   = aero_data[mach_mask, 2]

weather_data = np.genfromtxt(weather_file_path, delimiter=',', skip_header=1)
alt_mask = weather_data[:,4] <= 15000
alt_points = weather_data[alt_mask, 4]
pressure_points = weather_data[alt_mask, 3]
temp_points = weather_data[alt_mask, 5]

# 2. Generate the C++ Header Content
header_content = f"""#pragma once
#include <cstddef>

// Auto-generated aerodynamic data
// Aero Points: {len(mach_points)}
// Balloon Points: {len(alt_points)}

static const size_t AERO_TABLE_SIZE = {len(mach_points)};
static const size_t BALLOON_TABLE_SIZE = {len(alt_points)};

static const float ROCKET_MASS = {rocket_mass}f; // kg
static const float ROCKET_DIAMETER = {rocket_diameter}f; // meters 
static const float BRAKE_FACE_AREA = {brake_face_area}f; // m^2
static const float ROCKET_REFERENCE_AREA = {rocket_reference_area}f; // m^2
static const float TARGET_APOGEE_AGL = {target_apogee_AGL}f; // meters
static const float MAXIMUM_DEPLOYMENT_MACH = {maximumDeploymentMach}f;

static const float AERO_MACH_LUT[] = {{
    {', '.join([f'{x:.2f}f' for x in mach_points])}
}};

static const float AERO_CD_LUT[] = {{
    {', '.join([f'{x:.4f}f' for x in cd_points])}
}};

static const float BALLOON_ALT_LUT[] = {{
    {', '.join([f'{x:.1f}f' for x in alt_points])}
}};

static const float BALLOON_PRESSURE_LUT[] = {{
    {', '.join([f'{x:.1f}f' for x in pressure_points])}
}};

static const float BALLOON_TEMP_LUT[] = {{
    {', '.join([f'{x:.1f}f' for x in temp_points])}
}};
"""

# 3. Write to file
with open("flightCodeSrc/mothmansRevenge/config_data.h", "w") as f:
    f.write(header_content)

print("Successfully generated config_data.h")