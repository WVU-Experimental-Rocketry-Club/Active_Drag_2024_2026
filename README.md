# Active Drag Rocket Simulation

**Predictive airbrake control system for precision apogee targeting**  
Developed by WVU Experimental Rocketry Club for IREC 2026 30k SRAD Category

## Overview

This simulation framework enables accurate trajectory prediction and control analysis for high-power rockets equipped with active airbrake systems. The software models a predictive control algorithm that continuously adjusts airbrake deployment to achieve a target apogee, accounting for real-time atmospheric conditions and vehicle dynamics.

### Key Capabilities

- **Predictive Control Algorithm**: Forward-simulates trajectory at each timestep to predict apogee and compute optimal airbrake deployment
- **Physics-Based Modeling**: 4th-order Runge-Kutta integration of rocket equations of motion with 6-DOF capability
- **Advanced Aerodynamics**: Bilinear interpolation of drag coefficients across Mach number (0.2-2.0) and airbrake deployment states
- **Atmospheric Modeling**: ISA standard atmosphere with real weather data integration from balloon soundings
- **Modular Architecture**: Clean separation between physics core, simulation engine, and flight software modules
- **Configuration-Driven**: JSON-based rocket definitions allow rapid iteration on vehicle parameters

### Design Philosophy

The codebase is structured to mirror actual flight software architecture, with clear boundaries between physical modeling, numerical integration, and control algorithms. This enables both high-fidelity simulation and potential code reuse for embedded flight computers.

## Project Structure

```
Active_Drag_2024_2026/
├── main.py                          # Simulation orchestrator and entry point
├── README.md                        # Documentation (this file)
├── todo.md                          # Development roadmap and tasks
│
├── configs/                         # Rocket configuration files (JSON)
│   ├── competition_rocket_2026.json # IREC 2026 competition rocket
│   ├── 4inAD.json                   # 4" test vehicle
│   └── shenandoah_sunrise_irec.json # Alternative configuration
│
├── data/
│   ├── aero/                        # Drag coefficient lookup tables
│   │   ├── irec2026_rasaero.CSV    # Competition rocket aerodynamics
│   │   └── 4inAD.CSV                # Test vehicle aerodynamics
│   ├── weather/                     # Atmospheric data
│   │   ├── 2025_IREC_Weather.csv   # IREC competition site conditions
│   │   └── dayton_tmo_01282026.csv # TMO weather soundings
│   └── simulation_results/          # Output trajectory files
│
├── legacy/                          # Reference implementations
│   └── apogee_analysis_testCompRocket.py
│
└── src/                             # Core source code
    ├── core/                        # Physical models
    │   ├── atmosphere.py            # Atmospheric property calculations
    │   └── aerodynamics.py          # Drag force and Cd interpolation
    │
    ├── sim/                         # Numerical simulation
    │   ├── flightPhysics.py         # Equations of motion
    │   └── flightSimulation.py      # RK4 integrator and simulation loop
    │
    └── flightSoftware/              # Control and avionics
        ├── ad_controller.py         # Predictive airbrake controller
        ├── stateMachine.py          # Flight phase logic [WIP]
        ├── navigation.py            # State estimation [WIP]
        ├── datalogger.py            # Data recording [WIP]
        └── external_interfaces.py   # Hardware abstraction [WIP]
```

## Quick Start

### Prerequisites

- Python 3.8+
- NumPy, Pandas, Matplotlib

### Installation

```bash
# Clone repository
git clone <repository-url>
cd Active_Drag_2024_2026

# Install dependencies
pip install numpy pandas matplotlib openpyxl
```

### Running a Simulation

```bash
# Run with default configuration (competition rocket)
python main.py

# Specify custom configuration
python main.py --config configs/4inAD.json

# Generate plots and export data
python main.py --plot --output data/simulation_results/
```

### Configuration File Format

JSON configuration files define rocket parameters, initial conditions, and control settings:

```json
{
  "rocket_name": "WVU IREC 2026 Rocket",
  "dimensions": {
    "diameter": 0.158,
    "length": 3.5
  },
  "mass_properties": {
    "burnout_mass": 39.12
  },
  "simulation_parameters": {
    "time_step": 0.1,
    "burnout_time": 9.24,
    "burnout_altitude": 4481.7,
    "burnout_verticalVelocity": 545.86,
    "launch_altitude": 893
  },
  "active_drag_system": {
    "enabled": true,
    "target_apogee_AGL": 9587,
    "control_frequency": 10,
    "deployment_conditions": {
      "maximum_mach": 1.2
    }
  },
  "file_paths": {
    "aero_file": "data/aero/irec2026_rasaero.CSV",
    "weather_file": "data/weather/2025_IREC_Weather.csv"
  }
}
```

## Core Module Documentation

### Atmospheric Modeling (`src/core/atmosphere.py`)

Provides atmospheric property calculations using the International Standard Atmosphere (ISA) model with optional weather data integration.

**Functions:**
- `atm_density(altitude)` → Air density (kg/m³)
- `atm_temperature(altitude)` → Temperature (K)
- `atm_pressure(altitude)` → Pressure (Pa)
- `speed_of_sound(altitude)` → Speed of sound (m/s)
- `load_weather_data(df)` → Load real weather balloon data
- `use_isa_model()` → Revert to standard atmosphere

### Aerodynamic Forces (`src/core/aerodynamics.py`)

Computes drag forces with airbrake deployment effects using bilinear interpolation of coefficient tables.

**Functions:**
- `cd_interp(cd_array, velocity, percent_deploy, altitude)` → Interpolated drag coefficient
  - Interpolates across Mach numbers: 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0
  - Interpolates across deployment states: 0%, 50%, 100%
- `get_drag(cd_array, velocity, percent_deploy, altitude, diameter)` → Drag force (N)

### Flight Physics (`src/sim/flightPhysics.py`)

Implements rocket equations of motion for trajectory propagation.

**Functions:**
- `get_acceleration(state, accel_consts, drag_args)` → Vertical acceleration (m/s²)
  - Input state: `[altitude, velocity]`
  - Returns: `a = g - (drag/mass)`
- `get_acceleration_2d(state, accel_consts, drag_args)` → 2D acceleration [WIP]

### Numerical Integration (`src/sim/flightSimulation.py`)

4th-order Runge-Kutta integrator for accurate trajectory propagation.

**Functions:**
- `runge_kutta(state, accel_consts, drag_args, dt)` → Next state
  - Integrates: `[altitude, velocity]`
  - 4th-order accuracy: O(dt⁴)
- `run_simulation(rocketConfig, cd_array, airbrakes_enabled)` → Complete trajectory
  - Returns: pandas DataFrame with time-series data

### Airbrake Controller (`src/flightSoftware/ad_controller.py`)

Predictive proportional controller for active airbrake deployment.

**Algorithm Overview:**
1. At each control update (10 Hz), predict apogee by forward-simulating trajectory
2. Compute error: `error = predicted_apogee - target_apogee`
3. Adjust deployment: `deployment_cmd = kp * error`
4. Apply rate limiting and safety constraints

**Controller Parameters:**
- `kp = 0.01` → Proportional gain
- `control_frequency = 10 Hz` → Update rate
- `error_threshold = ±5 m` → Deadband to prevent oscillation
- `max_rate_change = 20%/s` → Maximum deployment rate
- `airbrakeMachThreshold = 1.2` → Maximum Mach for deployment

**Key Methods:**
- `update(time, state, accel_consts, drag_args, dt)` → Compute deployment command
- `_predict_apogee(...)` → Forward-simulate to apogee prediction

## Simulation Output

The simulation generates comprehensive time-series data for trajectory analysis:

### Output Files

- **Trajectory CSV**: Time-series data with columns:
  - `time` (s), `altitude` (m AGL), `velocity` (m/s), `acceleration` (m/s²)
  - `deployment_percentage` (%), `predicted_apogee` (m), `mach_number`
  
- **Comparison Plots**:
  - Altitude vs Time (baseline vs active drag)
  - Velocity vs Time
  - Deployment Percentage vs Time
  - Predicted vs Actual Apogee

### Performance Metrics

The simulation reports key performance indicators:
- **Baseline apogee** (no airbrakes)
- **Controlled apogee** (with airbrakes)
- **Apogee error** from target
- **Maximum deployment percentage**
- **Burnout conditions** (altitude, velocity, acceleration)
- **Control engagement time** (when airbrakes first deploy)

## Technical Details

### Numerical Integration

The simulation uses 4th-order Runge-Kutta (RK4) integration for trajectory propagation:

```
k1 = f(t, y)
k2 = f(t + dt/2, y + dt*k1/2)
k3 = f(t + dt/2, y + dt*k2/2)
k4 = f(t + dt, y + dt*k3)

y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
```

Where state `y = [altitude, velocity]` and derivative `f = [velocity, acceleration]`.

### Coordinate Systems

- **Altitude**: Meters above ground level (AGL)
- **Velocity**: Positive = upward
- **Acceleration**: Positive = upward (includes gravity and drag)
- **Deployment Percentage**: 0% = fully retracted, 100% = fully deployed

### Physics Assumptions

1. **1D Motion**: Vertical-only trajectory (wind effects modeled via initial conditions)
2. **Rigid Body**: No structural dynamics or propellant slosh
3. **Point Mass**: Aerodynamic forces applied at center of mass
4. **Drag-Only Aerodynamics**: Lift and side forces neglected
5. **Constant Mass**: Post-burnout trajectory only (motor not modeled)

### Atmospheric Modeling

Two modes are supported:

**ISA Standard Atmosphere:**
- Temperature: $T(h) = T_0 - Lh$ (troposphere)
- Pressure: $P(h) = P_0(1 - Lh/T_0)^{g_0M/RL}$
- Density: $\\rho(h) = P(h)M/RT(h)$

**Weather Balloon Data:**
- Interpolates actual atmospheric conditions from balloon soundings
- Includes temperature, pressure, wind profiles
- More accurate for competition day predictions

## Development Roadmap

See [todo.md](todo.md) for detailed task list. Key planned features:

### Short-Term
- ✅ Basic predictive controller implementation
- ✅ RK4 integration
- ✅ Bilinear Cd interpolation
- ⬜ GPS-only navigation mode (realistic sensor constraints)
- ⬜ Monte Carlo uncertainty quantification
- ⬜ Alternative control strategies (PID, bang-bang)

### Medium-Term
- ⬜ 2D trajectory modeling (wind drift)
- ⬜ State machine for flight phases (pad idle, boost, coast, descent)
- ⬜ Hardware-in-the-loop testing interface
- ⬜ Real-time data logging and telemetry

### Long-Term
- ⬜ 6-DOF simulation with attitude dynamics
- ⬜ Embedded flight computer code generation
- ⬜ Machine learning control optimization
- ⬜ Multi-body dynamics (fin deployment, parachute)

## Contributing

This project is maintained by the WVU Experimental Rocketry Club. For questions or contributions:

1. Follow the existing code structure and documentation standards
2. Test changes with multiple configurations
3. Update documentation for API changes
4. Submit clear commit messages describing changes

## References

### Standards and Models
- NIST Standard Atmosphere Calculator
- RASAero II Aerodynamic Analysis Software
- Barrowman Equations for Center of Pressure

### Control Theory
- "Rocket Airbrake Control for Precision Landing" - MIT
- "Predictive Control of Active Drag Systems" - NASA
- PID tuning methods for aerospace applications

### Competition
- IREC 2026 Rules and Requirements
- Spaceport America Cup Technical Guidelines

## License

Educational use for WVU Experimental Rocketry Club. All rights reserved.

## Acknowledgments

- **WVU Experimental Rocketry Club** - Design, testing, and competition team
- **IREC 2026** - Competition framework and motivation
- **RASAero II** - Aerodynamic coefficient data generation
- **Python Scientific Computing Stack** - NumPy, Pandas, Matplotlib

---

**Competition**: IREC 2026 30k SRAD Category  
**Target Apogee**: 30,000 ft AGL (9,144 m)  
**Vehicle**: 6" diameter, dual-deploy, active drag system  
**Launch Site**: Spaceport America, New Mexico (elevation 4,595 ft / 1,400 m)