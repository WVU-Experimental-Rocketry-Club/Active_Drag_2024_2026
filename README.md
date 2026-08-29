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
and without airbrakes. After you close the plot window it asks whether to save - saying
yes writes the plots, both trajectories as CSVs, and a summary of the config and
results into `data/simulation_results/`, all timestamped. `--no-plot` skips the plots.

That's the whole story for a rocket that already has a config. To sim a new rocket,
follow the setup below.

## Setting up a new rocket

Everything the sim knows about a rocket comes from RASAero and a weather sounding,
tied together by a config file. In order:

1. **Model the rocket in RASAero** and run its flight simulation with your motor.

2. **Export the aero plot data** (the CD vs Mach table) to a CSV in `data/aero/`.

3. **Export the flight simulation data** (the time-series output) to a CSV in
   `data/flight_sims/`. Name both exports by rocket and motor so they're
   identifiable later.

4. **Get a weather sounding** for the launch site into `data/weather/`, named by
   date and station. The sim needs the `geopotential height_m`, `pressure_hPa`,
   and `temperature_C` columns - it interpolates the real atmosphere from these
   instead of using the standard atmosphere.

5. **Make the config**: copy the closest existing file in `configs/`, then set the
   rocket name, diameter, launch site altitude, the file paths from steps 2-4, and
   the airbrake settings (target apogee, brake area, force limit, deployment
   limits). `configs/README.md` explains every key and its units.

6. **Fill in the burnout state** - don't type these numbers by hand:

   ```
   python utilities/import_rasaero.py data/flight_sims/your_export.CSV configs/your_config.json
   ```

   The sim starts at motor burnout (the brakes never deploy under power, so the
   boost isn't simmed). The importer finds burnout in the RASAero export, converts
   to metric, and writes the burnout altitude/velocity/acceleration/mass into the
   config.

7. **Run it**: `python main.py --config configs/your_config.json`. Sanity check
   the baseline apogee against RASAero's projected apogee (the importer prints it) -
   they should land within a few percent of each other. If they're way off, the
   usual suspects are a wrong launch_altitude or mismatched aero/flight exports.

### How the sim works

From the burnout state in the config, the sim:

1. Integrates the 2D trajectory (vertical + horizontal) to apogee with RK4
2. Gets body drag from the RASAero CD-vs-Mach table and brake drag from
   compressible dynamic pressure times the deployed face area (percent deploy is
   linear in projected area, the same model the flight code runs)
3. Gets air density from the weather sounding for the launch site
4. Runs the airbrake controller: predict apogee by simulating ahead to zero
   vertical velocity, compare against the target, and step the deployment up
   or down

It runs twice - once with the brakes locked shut for a baseline, once with the
controller active - and plots both.

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
and reflash. The script reads the config named in `config_path` at the top of
`flightDataFile.py`, so point that at your config first.

Before a launch: get a fresh sounding for the launch site into the config's
weather file, rerun `flightDataFile.py`, recompile, and reflash - otherwise the
rocket flies with whatever atmosphere was baked in last time.

## Contributing

Working on the code itself? Read [DEVELOPERS.md](DEVELOPERS.md) first - it covers
how the sim flows, the physics model, and how to change things without breaking
the sim/flight-code agreement.

- **Branch off `main`, PR back into `main`. Don't commit directly to `main`.**
- Don't commit build output (`.o`, `.pyc`, compiled binaries) - the gitignore
  should catch it.
- If you change what the sim reads from configs, update `REQUIRED_CONFIG_KEYS`
  in `main.py` and `configs/README.md` to match.
- Tag the flight code the day it flies so we always know exactly what was on the
  rocket.
