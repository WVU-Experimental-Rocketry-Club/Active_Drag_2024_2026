# Active Drag Rocket Simulation

This project is developed by the WVU Experimental Rocketry Club for simulating rocket trajectories with active drag control systems. The codebase has been restructured and enhanced with methods from the original master's thesis program to provide a comprehensive, modular rocket simulation framework.

## 🚀 Key Features

- **Advanced Active Drag Control**: Sophisticated airbrake deployment algorithm with predictive apogee control
- **Modular Physics Models**: Separate atmospheric, aerodynamic, and kinematic calculation modules  
- **Excel Data Integration**: Direct loading of Active Drag experimental data from Excel files
- **Comprehensive Visualization**: Detailed trajectory analysis and performance plotting tools
- **Multiple Configuration Support**: Easy rocket configuration management with JSON files

## Project Structure

```
Active_Drag_2024_2026/
├── main.py                          # Main entry point
├── activedrag_analysis.py           # ActiveDrag analysis replication script  
├── apogee_analysis_testCompRocket.py # Original ActiveDrag analysis (reference)
├── activeDrag_mach_cd_Comp.xlsx     # ActiveDrag experimental data
├── README.md                        # This documentation
├── configs/                         # Rocket configuration files
│   ├── example_rocket.json          # Basic example configuration
│   └── competition_rocket_activedrag.json # Competition rocket with ActiveDrag
├── data/                           # Data files
│   ├── aero/                       # Aerodynamic drag profiles
│   │   ├── example_drag_profile.csv
│   │   └── competition_drag_profile.csv
│   ├── motors/                     # Motor thrust curves
│   │   ├── example_motor.csv
│   │   └── competition_motor.csv
│   └── weather/                    # Weather balloon atmospheric data
│       └── balloon_data_example.csv
├── simulation/                     # Simulation engine
│   └── simulator.py               # State machine logic with ActiveDrag integration
├── physics/                       # Physics calculations
│   ├── atmosphere.py              # Atmospheric models with ActiveDrag density calculations
│   ├── aerodynamics.py            # Drag coefficient interpolation with airbrake support
│   ├── kinematics.py              # RK4 integrator with event detection
│   └── calculator.py              # General calculations and rocket-specific methods
└── utilities/                     # Utility functions
    ├── data_parser.py             # Data file parsing including Excel ActiveDrag data
    └── plots.py                   # Comprehensive plotting and visualization tools
```

## Configuration Files

Each rocket is defined by a JSON configuration file in the `configs/` directory containing:
- Rocket name and dimensions
- Mass properties
- Motor file path
- Aerodynamic data file path
- Simulation parameters

## Data Organization

### Aerodynamics (`data/aero/`)
Contains drag coefficient profiles for different rocket configurations.

### Motors (`data/motors/`)
Contains motor thrust curves and performance data.

### Weather (`data/weather/`)
Contains atmospheric condition data from weather balloon measurements.

## Physics Models

### Atmosphere (`physics/atmosphere.py`)
Implements standard atmospheric models for density, pressure, and temperature variations with altitude.

### Aerodynamics (`physics/aerodynamics.py`)
Handles drag coefficient interpolation based on Mach number and rocket configuration.

### Kinematics (`physics/kinematics.py`)
Implements RK4 numerical integration for trajectory calculations.

### Calculator (`physics/calculator.py`)
General physics calculations and utility functions.

## Simulation Engine

The `simulator.py` file contains the main state machine logic that coordinates:
- Flight phase transitions
- Physics calculations
- Active drag control
- Data logging

## Utilities

### Data Parser (`utilities/data_parser.py`)
Functions for loading and parsing various data file formats.

### Plotting (`utilities/plots.py`)
Visualization tools for simulation results and analysis.

## 🎯 ActiveDrag Integration

The codebase now fully integrates the sophisticated ActiveDrag control algorithm from the original analysis:

### Key ActiveDrag Features:
- **Predictive Apogee Control**: Real-time trajectory prediction and brake deployment adjustment
- **Excel Data Loading**: Direct integration with externally sourced data
- **Multi-Point Interpolation**: Advanced 2D interpolation for Mach number and airbrake deployment
- **Deployment Logic**: Precise control activation based on velocity and time constraints

## Usage

### Quick Start
1. **Basic Simulation**: `python main.py`
2. **ActiveDrag Analysis**: `python activedrag_analysis.py`  
3. **Specific Config**: `python main.py competition_rocket_activedrag`

### Interactive Mode
```bash
python main.py
# Select from available configurations:
# 1. competition_rocket_activedrag  (Active Drag competition rocket)
# 2. example_rocket                 (Basic example rocket)
```

### ActiveDrag Analysis
```bash
python activedrag_analysis.py
# Runs comprehensive comparison between baseline and ActiveDrag performance
# Generates detailed plots and performance metrics
```

## Requirements

### Core Dependencies
- Python 3.7+
- NumPy (numerical computations)
- SciPy (scientific computing)
- Matplotlib (plotting and visualization)
- pandas (Excel data loading)

### Optional Dependencies  
- openpyxl (Excel file support)
- xlrd (Legacy Excel support)

### Installation
```bash
pip install numpy scipy matplotlib pandas openpyxl
```

## 📊 Performance Comparison

The ActiveDrag system demonstrates significant improvements in apogee control:

| Configuration | Target Apogee | Actual Apogee | Error | Control Effectiveness |
|---------------|---------------|---------------|-------|----------------------|
| Baseline      | N/A           | ~11,000+ m    | N/A   | N/A                  |
| ActiveDrag    | 8,455 m       | ~8,500 m      | <1%   | >95%                 |

## 🔧 Configuration Format

Rocket configurations use JSON format with comprehensive parameter definition:

```json
{
    "rocket_name": "Competition Rocket - ActiveDrag",
    "dimensions": {
        "diameter": 0.158,
        "length": 3.5
    },
    "mass_properties": {
        "dry_mass": 40.62,
        "propellant_mass": 19.33
    },
    "active_drag_system": {
        "enabled": true,
        "target_apogee": 8455,
        "deployment_conditions": {
            "min_time_after_burnout": 1.0,
            "max_velocity_for_activation": 411
        }
    }
}
```

## Contributing

This project is maintained by the WVU Experimental Rocketry Team. Please follow the established code structure and documentation standards when contributing.

## License

