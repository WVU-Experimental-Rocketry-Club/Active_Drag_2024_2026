# Active Drag 2024-2026

WVU Experimental Rocketry Club airbrake project. This repo holds two things:

1. **A Python simulator** that predicts apogee and airbrake behavior on your desktop,
   so you can size brakes, pick targets, and tune the controller before a flight.
2. **The flight code** (`flightCodeSrc/`) that actually runs on the rocket.

The two never talk to each other directly. The Python sim is for design and analysis
on the ground. The flight computer runs a C++ version of the same physics, and it gets
its settings and lookup tables from a generated header file (see
[Getting settings onto the rocket](#getting-settings-onto-the-rocket) below).

## Running the sim

You need Python 3.11+ installed. From the repo folder:

```
pip install -r requirements.txt
python main.py
```

It'll list the available rocket configs and ask you to pick one. Or skip the menu:

```
python main.py --config configs/competition_rocket_2026.json
```

You get an apogee summary printed in the terminal and a window of comparison plots
(altitude, velocity, acceleration, brake deployment, brake force) for the rocket with
and without airbrakes. `--no-plot` skips the plots.

### How the sim works

The sim starts at **motor burnout**, not liftoff - the brakes never deploy under
power, so there's no reason to sim the boost. Burnout conditions (altitude, velocity,
time) come from RASAero and live in the config file. From there it:

1. Integrates the 2D trajectory (vertical + horizontal) to apogee with RK4
2. Gets body drag from the RASAero CD-vs-Mach table and brake drag from
   compressible dynamic pressure times the deployed face area
3. Gets air density from a real weather balloon sounding for the launch site
   instead of the standard atmosphere
4. Runs the airbrake controller at 10 Hz: predict apogee by simulating ahead to
   zero vertical velocity, compare against the target, and step the deployment
   up or down

To sim a new rocket, copy the closest config in `configs/` and update the numbers.
`configs/README.md` explains every key and where to get the values.

## What's where

```
main.py                     entry point, run this
configs/                    one JSON per rocket+motor combo (see configs/README.md)
data/aero/                  RASAero aero plot exports (Mach vs CD)
data/flight_sims/           RASAero flight sim exports (for the burnout importer)
data/weather/               weather balloon soundings, named by date-station
data/simulation_results/    sim output (not tracked in git)
src/core/                   atmosphere + drag calculations
src/sim/                    RK4 integration and the main sim loop
src/flightSoftware/         airbrake controller (the algorithm the rocket runs)
flightCodeSrc/              the actual onboard code, see below
utilities/flightDataFile.py generates config_data.h for the flight code
utilities/import_rasaero.py fills a config's burnout state from a RASAero export
legacy/                     the original thesis-based script everything came from
Post Flight Data/           logs from real flights
```

## Flight code

Each folder in `flightCodeSrc/` is one generation of the onboard code:

| Folder | Status |
|--------|--------|
| `mmrAirbrake/` | **Current.** Flown at IREC 2026. ODrive-driven brakes. |
| `mothmansRevenge/` | Flown at the Kansas test launch, March 2026. Stepper-driven. |
| `AirbrakeController/`, `AirbrakeController_1/` | Early development versions. |
| `m10q_qwiic_check/` | GPS bring-up test sketch. |
| `testBuzzer/` | Buzzer test sketch. |

The flight computer navigates on GPS alone (u-blox, 10 Hz) and runs the same
predict-apogee-then-adjust loop as the Python controller. It logs to onboard flash;
after a flight, connect over serial and send `d` to dump the log, `e` to erase it.

`computations.cpp` in each folder is the C++ port of the sim physics. There's a
`desktop.cpp` + Makefile in each one for building and testing the physics on a
computer without flashing hardware (needs g++ - on Windows install MSYS2/MinGW,
or use WSL).

## Getting settings onto the rocket

The flight code can't read JSON or CSV files, so everything it needs gets baked into
a header at build time:

```
configs/*.json  +  data/aero/*.CSV  +  data/weather/*.csv
                        |
        python utilities/flightDataFile.py
                        |
        flightCodeSrc/mmrAirbrake/config_data.h
                        |
        compile + flash in Arduino IDE
```

`config_data.h` is generated - don't edit it by hand, your changes will get wiped the
next time someone runs the script. Change the JSON config instead, rerun the script,
and reflash. Before a launch, make sure the weather file in the config is a fresh
sounding for the launch site.

## Contributing

- Work off `dev`, PR into `main`.
- Don't commit build output (`.o`, `.pyc`, compiled binaries) - the gitignore
  should catch it.
- If you change what the sim reads from configs, update `REQUIRED_CONFIG_KEYS`
  in `main.py` and `configs/README.md` to match.
- Tag the flight code the day it flies so we always know exactly what was on the
  rocket.
