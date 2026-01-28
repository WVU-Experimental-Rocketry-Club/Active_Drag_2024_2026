# Active Drag Rocket Simulation

This project is developed by the WVU Experimental Rocketry Club for the IREC 2026 30k SRAD Category. The simulation framework provides a comprehensive, modular approach to rocket trajectory prediction with active airbrake control using predictive apogee targeting.

## 🚀 Key Features

- **Predictive Airbrake Control**: Proportional controller that adjusts airbrake deployment based on real-time apogee predictions
- **Modular Physics Architecture**: Separation of concerns with distinct physics, control, and simulation modules
- **Advanced Aerodynamic Modeling**: Bilinear interpolation of drag coefficients across Mach numbers (0.2-2.0) and airbrake deployment states
- **Standard Atmosphere Integration**: ISA atmospheric model with optional real-world weather data integration
- **Configuration-Driven Simulation**: JSON-based rocket parameter definitions with flexible simulation settings
- **RK4 Numerical Integration**: 4th-order Runge-Kutta integration for accurate trajectory propagation

## Project Structure

```
Active_Drag_2024_2026/
├── main.py                          # Entry point with SimulationRunner class
├── README.md                        # This documentation
├── todo.md                          # Development roadmap
│
├── configs/                         # Rocket configuration files (JSON)
│   ├── competition_rocket_activedrag.json
│   └── example_rocket.json
│
├── data/                            # Data files and simulation outputs
│   ├── aero/                        # Drag coefficient lookup tables
│   ├── motors/                      # Motor thrust curves
│   ├── weather/                     # Atmospheric conditions (balloon data)
│   └── simulation_results/          # Output trajectory CSV files
│
├── legacy/                          # Original reference implementations
│   ├── apogee_analysis_testCompRocket.py
│   └── activeDrag_mach_cd_Comp.xlsx
│
├── src/                             # Main source code (modular architecture)
│   ├── core/                        # Physics core modules
│   │   ├── __pycache__/
│   │   ├── atmosphere.py            # ISA atmospheric model + weather integration
│   │   └── aerodynamics.py          # Drag force calculations & Cd interpolation
│   │
│   ├── sim/                         # Flight simulation engine
│   │   ├── __pycache__/
│   │   ├── flightSimulation.py      # RK4 integrator & main simulation loop
│   │   └── flightPhysics.py         # Force & acceleration calculations
│   │
│   └── flightSoftware/              # Control algorithms & flight systems
│       ├── __pycache__/
│       ├── ad_controller.py         # Proportional airbrake controller
│       ├── stateMachine.py          # Flight phase state machine [TODO]
│       ├── datalogger.py            # Flight data logging [TODO]
│       ├── navigation.py            # Navigation calculations [TODO]
│       └── external_interfaces.py   # Hardware interfaces [TODO]
│
└── utilities/                       # Utility functions (placeholder for future)
```

## Core Modules

### `src/core/atmosphere.py`
Atmospheric property calculations supporting ISA standard atmosphere model:
- `atm_density(altitude)` - Air density at altitude (kg/m³)
- `atm_temperature(altitude)` - Temperature (K)
- `atm_pressure(altitude)` - Pressure (Pa)
- `speed_of_sound(altitude)` - Speed of sound (m/s)

Supports optional integration of real weather balloon data for enhanced accuracy.

### `src/core/aerodynamics.py`
Drag force calculations with airbrake deployment effects:
- `cd_interp(cd_array, velocity, percent_deploy)` - Bilinear interpolation of drag coefficients across:
  - Mach numbers: 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0
  - Deployment states: 0%, 33% (half), 100% (full)
- `get_drag(cd_array, velocity, percent_deploy, altitude, diameter)` - Drag force in Newtons

### `src/sim/flightPhysics.py`
Newton's laws for rocket motion:
- `get_acceleration(state, accel_consts, drag_args)` - Vertical acceleration calculation
- `get_acceleration_2d(state, accel_consts, drag_args)` - [Future] 2D acceleration with horizontal drag

### `src/sim/flightSimulation.py`
Main simulation engine with RK4 numerical integration:
- `runge_kutta(state, accel_consts, drag_args, dt)` - 4th-order Runge-Kutta integrator for [altitude, velocity]
- `runge_kutta_2d(state, accel_consts, drag_args, dt)` - [Future] 2D integration for [altitude, vy, horizontal_dist, vx]
- `run_simulation(rocketConfig, cd_array, airbrakes_enabled)` - Main simulation loop

### `src/flightSoftware/ad_controller.py`
Proportional airbrake controller:
- **Algorithm**: Predicts apogee by forward simulation, adjusts deployment proportionally to error
- **Key Parameters**:
  - `kp` (proportional gain): Controls responsiveness (default 0.01)
  - `control_frequency`: Update rate in Hz (default 10 Hz)
  - `error_threshold`: Acceptable apogee error in meters (default ±5m)
  - `max_rate_change`: Maximum deployment rate in %/s (default 20%/s)
  - `airbrakeMachThreshold`: Maximum Mach for deployment (default 1.45)

- **Key Methods**:
  - `update(time, state, accel_consts, drag_args, dt)` - Computes new deployment percentage
  - `_predict_apogee(state, accel_consts, drag_args, dt)` - Simulates trajectory to apogee

## Configuration Format

Rockets are configured via JSON files in `configs/` with the following structure:

```json
{
    "rocket_name": "Competition Rocket - Active Drag",
    "dimensions": {
        "diameter": 0.158
    },
    "mass_properties": {
        "burnout_mass": 40.62
    },
    "simulation_parameters": {
        "time_step": 0.01,
        "launch_altitude": 0,
        "burnout_time": 9.901,
        "burnout_altitude": 4156.8624,
        "burnout_verticalVelocity": 507.79,
        "burnout_horizontalVelocity": 83.2
    },
    "active_drag_system": {
        "target_apogee_AGL": 8000,
        "controller_config": {
            "kp": 0.01,
            "control_frequency": 10.0,
            "error_threshold": 5.0,
            "max_rate_change": 20.0,
            "airbrakeMachThreshold": 1.45
        }
    }
}
```

## Usage

### Running Simulations

```python
# In main.py
from src.sim.flightSimulation import run_simulation
from main import SimulationRunner

# Create a simulation runner
runner = SimulationRunner('configs/competition_rocket_activedrag.json')

# Load configuration and aerodynamic data
runner.load_config()
cd_array = runner.load_aero_data()

# Run both baseline and active drag simulations
baseline_results = runner.run_baseline_simulation()
active_drag_results = runner.run_active_drag_simulation()

# Generate comparison plots
runner.plot_results(show=True)
runner.print_summary()
```

### Simulation Output

`run_simulation()` returns a numpy array with shape `(5, N)` where N is number of timesteps:
```python
output = [time, altitude, velocity, acceleration, percent_deploy]
```

For 2D simulations (future): `[time, altitude, vy, vx, horizontal_distance, acceleration, percent_deploy]`

### Example: Custom Simulation

```python
import json
from src.sim.flightSimulation import run_simulation
import pandas as pd
import numpy as np

# Load configuration
with open('configs/competition_rocket_activedrag.json', 'r') as f:
    config = json.load(f)

# Load aerodynamic data
cd_fb = pd.read_excel('legacy/activeDrag_mach_cd_Comp.xlsx', 
                      skiprows=8, nrows=11, usecols=['D', 'H', 'L'])
cd_array = np.array([cd_fb['D'], cd_fb['H'], cd_fb['L']]).transpose()

# Run simulation with airbrakes enabled
results = run_simulation(config, cd_array, airbrakes_enabled=True)
time, altitude, velocity, acceleration, deployment = results

# Find apogee
apogee = np.max(altitude)
apogee_time = time[np.argmax(altitude)]
print(f"Apogee: {apogee:.1f} m at t={apogee_time:.2f} s")
```

## Physics Models & Algorithms

### Vertical Motion (1D Simulation)

**State vector**: `[altitude, vertical_velocity]`

**Kinematic equations**:
$$\frac{dh}{dt} = v_y$$
$$\frac{dv_y}{dt} = \frac{F_{thrust} - F_{drag}}{m} - g$$

Where:
- Drag: $F_{drag} = \frac{1}{2}\rho v^2 C_D A$ 
- Mach number: $M = \frac{v}{a(h)}$ where $a$ is speed of sound at altitude

### Airbrake Control (Proportional)

**Control law**:
$$u(t) = u(t-\Delta t) + k_p \cdot (h_{pred} - h_{target})$$

Where:
- $u$ = deployment percentage (0-100%)
- $k_p$ = proportional gain (0.01 default)
- $h_{pred}$ = apogee prediction via forward simulation
- Rate-limited to prevent oscillation: $|du/dt| \leq 20\%/s$ (configurable)

### Drag Coefficient Interpolation

Bilinear interpolation of $C_D$ across Mach and deployment states:

Given tables at Mach points $\{0.2, 0.4, ..., 2.0\}$ and deployment $\{0\%, 33\%, 100\%\}$:
$$C_D(M, \delta) = \text{interp2d}(M, \delta, CD\_table)$$

## Performance Metrics

The current implementation achieves:
- **Apogee Targeting Accuracy**: ±50-200m depending on controller tuning
- **Control Response Time**: ~3-5 seconds for full deployment
- **Computational Efficiency**: ~10-50 ms per timestep (dt=0.01s)

## Requirements

### Core Dependencies
- **Python 3.8+**
- **NumPy** - Numerical computations
- **Pandas** - Data handling and Excel reading
- **Matplotlib** - Plotting and visualization
- **openpyxl** - Excel file support

### Installation

```bash
# Clone repository
git clone https://github.com/WVU-Experimental-Rocketry-Club/Active_Drag_2024_2026.git
cd Active_Drag_2024_2026

# Install dependencies
pip install numpy pandas matplotlib openpyxl
```

## Development Status

### ✅ Implemented
- [x] 1D vertical trajectory simulation (RK4 integration)
- [x] Proportional airbrake controller with apogee prediction
- [x] Drag coefficient bilinear interpolation (Mach & deployment)
- [x] ISA atmospheric model
- [x] Configuration-driven simulation
- [x] SimulationRunner with baseline/active-drag comparison
- [x] Controller rate limiting and deadband logic

### 🔄 In Progress / Future
- [ ] 2D trajectory simulation (horizontal + vertical motion)
- [ ] State machine for flight phases (boost, coast, drogue, main)
- [ ] Real weather data integration (weather balloon files)
- [ ] Wind modeling
- [ ] PID controller alternative
- [ ] Monte Carlo uncertainty analysis
- [ ] Sensor simulation (GPS, accelerometer noise)
- [ ] Hardware-in-loop testing interface
- [ ] Comprehensive test suite

## Troubleshooting

### Simulation Hangs
- **Cause**: Airbrake controller prediction loop taking too long
- **Solution**: Reduce `control_frequency` or increase prediction `dt`

### Large Apogee Errors
- **Cause**: `kp` gain too small or too large
- **Solution**: Tune proportional gain (start at 0.01, adjust ±50%)

### NaN/Inf in Results
- **Cause**: Drag coefficient interpolation out of Mach range
- **Solution**: Ensure Mach stays within 0.2-2.0 or extrapolate Cd table

## References

- Original Active Drag analysis: `legacy/apogee_analysis_testCompRocket.py`
- Excel drag data: `legacy/activeDrag_mach_cd_Comp.xlsx`
- Physics reference: RK4 integration + drag equation (Raymer, "Aircraft Design")

## Contributing

Please follow these guidelines when contributing:
1. Maintain modular structure (physics, control, simulation separate)
2. Add docstrings to all functions
3. Update README for new features
4. Test on example configuration before committing
5. Reference issue numbers in commits

## License

[License information to be added]

## Contact

WVU Experimental Rocketry Club - IREC 2026