# Developer's Guide

This is the guide for people working ON the code rather than just running it. The
README covers how to run a sim and set up a new rocket - read that first. This
document covers how everything fits together, how a sim run actually flows through
the code, the physics model and its assumptions, and how to make changes without
breaking things you didn't know existed.

This guide **can** and **should** evolve over time, both as the code gets improved
and as the team learns the best ways to work together on this software. The whole
codebase started out as a single person working on it, so there will be many things
that need to change as more and more people get their hands on it. 

The biggest piece of advice I can give is to constantly remember that all of this
software will be run in the future by people who are only marginally familiar
with its origins and will not be well versed in the best principles for 
writing and maintaining code, so make **everything** more robust and better documented
than it feels like it needs to be. 

## The big picture

There are two programs in this repo that must agree with each other:

1. **The Python simulator** (`main.py` + `src/`) - runs on your desktop. Used to
   size brakes, pick targets, tune the controller, and predict flights before
   they happen.
2. **The flight code** (`flightCodeSrc/mmrAirbrake/`) - C++ on the rocket's
   flight computer. Runs the same predict-apogee-and-adjust loop in real time
   using GPS.

They never communicate at runtime. The link between them is a code generation
step: `utilities/flightDataFile.py` reads a config JSON plus its aero and weather
CSVs and writes `flightCodeSrc/mmrAirbrake/config_data.h`, which gets compiled
into the flight code. Same settings, same lookup tables, two implementations.

**VERY IMPORTANT: if you change the physics or the controller in
one program, change it in the other (or write down why not).** The whole value of
the simulator is that it predicts what the flight computer will do. Every time
the two drift apart, sim results stop meaning anything. This has happened
before and will happen again. Before every flight, try to have multiple people
verify that the flight code agrees with the simulator.

## File Architecture

```
main.py                       CLI, config loading/validation, runs both sims, plots, saves results
configs/*.json                one rocket+motor combo each (configs/README.md has every key)
src/
  core/atmosphere.py          air properties vs altitude, from weather sounding or ISA
  core/aerodynamics.py        body drag + brake drag forces
  sim/flightPhysics.py        acceleration from forces (1D and 2D)
  sim/flightSimulation.py     RK4 integrators + the main sim loop
  flightSoftware/
    ad_controller.py          the airbrake controller (the part that flies, in python form)
    stateMachine.py           NOT WIRED IN - scaffold for a flight phase state machine
    navigation.py             NOT WIRED IN - scaffold for sensor fusion
    datalogger.py             NOT WIRED IN - scaffold for logging
    external_interfaces.py    NOT WIRED IN - scaffold for mock/hardware sensors
utilities/
  flightDataFile.py           generates config_data.h for the flight code
  import_rasaero.py           fills a config's burnout state from a RASAero export
flightCodeSrc/
  mmrAirbrake/                CURRENT flight code (IREC 2026, ODrive actuator)
  mothmansRevenge/            as flown at Kansas 03/2026 (stepper) - kept as a record
  AirbrakeController*/        early development versions
legacy/                       the original thesis-based single-file script
data/flight_sims/             RASAero flight exports (importer input)
Post Flight Data/             real logs from actual flights
```

A warning about the scaffold files: `stateMachine.py`, `navigation.py`,
`datalogger.py`, and `external_interfaces.py` have detailed docstrings and
plausible-looking classes, but nothing imports them. They describe features that 
ought to be implemented, but don't do anything yet. Don't assume something works because a
docstring says it does - a few of the older docstrings describe features that
were never built. When in doubt, trust the code, then the git history, then the
comments. Tangentially related: make sure to update the comment blocks as you work
so that they actually tend to represent the code underneath.

## How a sim run flows:

Running the command:
`python main.py --config configs/competition_rocket_2026.json` starts this process:

```
main()
  pick_config()                     if no --config given
  SimulationRunner(config, outdir)
    load_config_file()              utf-8-sig read + friendly JSON errors
    check_config()                  every required key present, data files exist
  run_full_analysis()
    load_aero_data()                once: CD table -> numpy dict, weather -> atmosphere module
    run_simulation(..., False)      baseline, brakes locked shut
    run_simulation(..., True)       active drag run
    print_summary()                 run info + apogee numbers, via run_summary()
    plot_results()                  2x3 figure, then offers to save everything
```

`run_simulation()` in `src/sim/flightSimulation.py` is the actual flight sim:

```
state = [burnout_altitude, burnout_vy, 0, burnout_vx]      # starts at burnout, not liftoff
while vertical velocity >= 0:                              # i.e. until apogee
    state = runge_kutta_2d(state, ...)                     # one 0.1 s step
    if airbrakes_enabled:
        deployment, brake_force = controller.update(...)   # may change percent_deploy
    log everything into parallel arrays
return np.array([time, alt, vy, vx, x, accel, deploy_pct, deploy_angle, brake_force])
```

The sim starts at burnout because the brakes never deploy under power. All the
burnout numbers come from a RASAero export via `import_rasaero.py`. Just follow
the comments in that script to run it correctly.

**The results array** (returned by `run_simulation`, also the CSV columns
when you save a run): 0 time, 1 altitude, 2 vertical velocity, 3 horizontal
velocity, 4 horizontal position, 5 acceleration, 6 deploy percent, 7 deploy
angle, 8 brake force. If you add a row, update `plot_results`, `save_results`,
and this list.

### One controller tick

`AirbrakeController.update()` in `ad_controller.py`, called every sim step:

1. Rate limit check - skip if called faster than `control_frequency`
2. **Predict apogee**: copy the current state and run the same RK4 forward,
   brakes frozen at the current deployment, until vertical velocity hits zero.
   This nested simulation inside every controller tick is why the sim used to
   take a full minute to run (see the performance section), eince each step of
   the outer simulation wrapper triggered a full inner loop integration to apogee.
3. `error = predicted apogee - target`
4. **Arming latch**: brakes stay stowed until a prediction at `min_deployment`
   shows the rocket would STILL overshoot. This prevents the system from operating 
   at very low deployment angles where the control authority isn't modeled as well
   and is difficult to guarantee the physical system has the resolution to control.
5. **Gates**: no deployment change above `maximum_mach`, and no increase that
   would push predicted brake drag past `max_brake_force`. These factors
   should be changed in the config file based on the confidence in hardware
   components and confidence in supersonic/high subsonic modeling
6. **Step**: deployment moves toward the target at the actuator's real rate,
   `100 / full_deploy_time` percent per second, one tick's worth per update

The flight code's control loop (`flightCoast()` in `mmrAirbrake.ino`) is the
same idea but simpler: it currently has the mach gate but NOT the force gate or
the arming latch. That's one of probably many sim/flight divergences that still need
reconciling.

## The physics model

All internal units are metric (m, m/s, kg, N, Pa, K). Imperial appears only in
printouts and RASAero imports.

**Integration**: classic RK4, fixed 0.1 s timestep, state `[y, vy, x, vx]`.
2D point mass - drag acts opposite the velocity vector, gravity straight down,
constant g = 9.80665. No wind, no pitch dynamics, no thrust (post-burnout only).

**Body drag**: `0.5 * rho * v^2 * CD(mach) * A`. CD comes from the RASAero aero
export via `np.interp` on the Mach column. The table is the power-off CD curve at
RASAero's reference conditions, so Reynolds/altitude dependence of CD is not
modeled. Against RASAero's own trajectory this whole model lands within ~2.5% on
apogee.

**Brake drag**: `q * area * Cd(angle)` where
- `q` is compressible dynamic pressure (total minus static pressure, not the
  incompressible 0.5*rho*v^2 - matters near mach 1)
- deployment percent is linear in projected area (that's how the mechanism is
  defined), so the flap angle is `arcsin(percent/100)`
- `Cd(angle) = 0.00889 * angle_deg + 0.35`, a fit anchored on the 0.85 average
  for folding brakes from Michael Farha's thesis and also makes sense against 
  known Cd of a flat angled plate. The table in the thesis is good, but more CFD
  runs should be used at some point to further verify, as well as ideally a dozen
  or so supersonic flights to see how well it stacks up. The fit is a best guess
  for testing and is worth correcting.

This model lives in two places: `getTotalDrag()` in
`src/core/aerodynamics.py` and `get_total_drag()` in
`flightCodeSrc/mmrAirbrake/computations.cpp`. They should be identical for now,
but over time it would be good to use an actual interpolated LUT rather than a 
linearization of sketchy data. 

**Atmosphere**: `src/core/atmosphere.py` interpolates pressure and temperature
from a real weather balloon sounding, and derives density and speed of sound from
those. ISA formulas are the fallback if no weather file is loaded. The flight
code does the same thing with lookup tables baked into `config_data.h` (nearest
point rather than interpolated - close enough at 2500 table points). 

**Known simplifications**, roughly in order of how much they cost: no
Reynolds/altitude dependence of CD, constant gravity (~0.2-0.3% at altitude), no
wind, nearest-point LUTs on the flight side. None of these are currently worth
fixing before actually flying the thing and seeing what happens.

## Performance notes

The sim runs in seconds now, but it's easy to accidentally undo that. The important
path to remember: controller tick -> apogee prediction -> hundreds of RK4 steps -> 4
acceleration evals each -> drag -> atmosphere. Anything you add inside that chain
runs hundreds of thousands of times per sim. The rules that keep it fast:

- No pandas inside the loop. Tables get converted to plain numpy arrays once at
  load (`load_aero_data`, `load_weather_data`). `np.interp` against a pandas
  column re-extracts the whole array every call.
- One atmosphere lookup per drag call - `atm_properties()` returns pressure,
  temperature, density, and speed of sound together. Don't call the individual
  getters repeatedly for the same altitude.
- Load data files once. `load_aero_data` caches; don't add re-reads.

If it ever needs to be faster still (parameter sweeps, Monte Carlo), the next
moves are coarser prediction timesteps far from apogee, re-predicting only when
deployment changed, or Numba on the hot path. Don't bother doing any of that unless
it actually becomes necessary, but **DO** look into doing some Monte Carlo runs to be
much more resilient to things like burnout mass uncertainty, CD uncertainty, etc.

## Making changes without breaking things

**Branch workflow**: branch off `main`, PR back into `main`. Before opening the
PR, run at least the competition config and one 4in config and compare the apogee
summary against an unmodified run. There is no automated test suite yet, so
before/after runs ARE the test suite. Save the results (the y/n prompt after the
plots) if you want a record - saved summaries include the config and date.

Ideally, you should mostly branch off of and PR into a `dev` branch of some sort,
whether that's a generic dev branch, a personal dev branch, etc. Try to save `main`
for very well proven code. 

You should also not get in the habit of approving your own PRs if you're working as
a software team and not a solo developer. The software lead or a designated person
should take a moment to glance through all of the proposed PR changes to make sure 
everything will continue to work nicely. 

There are many other best practices you can adopt for the software team, so you 
should spend part of a meeting each semester going over how people should be working
collaboratively on this project. 

**Adding a config key**: three places or it doesn't exist - the config JSON
itself, `REQUIRED_CONFIG_KEYS` in `main.py` (with units in the description - this
is what generates the helpful error when someone's config is missing it), and
`configs/README.md`.

**Changing the physics**: change it in `src/core/` AND in
`flightCodeSrc/mmrAirbrake/computations.cpp`, note it in the PR, and remember the
rocket doesn't get the change until someone recompiles and reflashes. You can
syntax-check the C++ without hardware by running the 'verify' step in the Arduino
IDE instead of 'compile and upload'. You can also write an intermediate C++ program
that's designed to be compiled on your desktop to test out all of the software logic
and computations without including any GPIO/sensors. 

**Changing config values that fly**: rerun `utilities/flightDataFile.py` (point
`config_path` at the top of it to your config) to regenerate `config_data.h`,
recompile, reflash. Never edit `config_data.h` by hand.

**Flown code is history**: the day something flies, tag the commit
(`flown/kansas-2026` style) and don't touch that folder again. Post-flight logs go
in `Post Flight Data/`.

**Time and altitude conventions**: sim time is seconds since liftoff (it just
starts partway through, at `burnout_time`). Altitude inside the sim is meters
ASL; targets and RASAero exports are AGL; `launch_altitude` in the config is the
bridge. When numbers look ~900 m off, this is why.

## Good next projects

Roughly in order of value:
- **CD correction in flight**: feed the inner rk4 loop an inaccurate CD curve
  and have your software be able to calculate the real CD based on accelerometer 
  data. This will let a flight still provide meaningful apogee corrections if the
  general CD curve shape is right but the scalar value is wrong, which is one of the 
  things we're still uncertain of and have not had any successful flights to validate
  further guesses.
- **Flight replay validation**: script that seeds the sim from a real flight
  log's burnout state (Post Flight Data has Blue Raven, Featherweight, and
  onboard logs from three flights) and overlays predicted vs actual. This will
  make each test flight much more valuable because you can apply software changes
  and see how they stack up to what has actually flown. 
- **Hardware In The Loop**: I got a basic version working at one point, but you'll
  have to rebuild yours from scratch. The python sim should be able to run the "outer"
  RK4 loop, pass simulated "sensor" data to the actual hardware, and you should be able 
  to watch the hardware deploy the brakes in simulated real time. 
- **RTOS Flight Code**: Moving the flight computer code over to a scheduled RTOS
  architecture would be good for a whole host of reasons you can look into. FreeRTOS
  has support for the RP2040/RP2350s.
- **Sim/flight controller parity**: port the force gate and arming latch to
  `mmrAirbrake`, or make the sim optionally run a flight-accurate mode.
- **A real test suite**: even five pytest checks (ISA density at sea level, drag
  increases with deployment, competition config apogee within a known band)
  would catch most accidental breakage. Unit tests are VERY VERY important, 
  and I haven't gotten around to doing them. But they will let you catch a lot of
  the errors (like the scalar issue at the TMO AD test flight), and give you peace of
  mind that if the tests pass, your algorithm is probably still operating as expected. 
- **Boost phase**: simulate from liftoff using a thrust curve (thrustcurve.org
  has a free API) so the burnout import step disappears entirely.
- **Named data structures**: the `drag_args`/`accel_consts` positional lists and
  numbered results rows predate everything else and are the biggest readability
  wart left.

## Other low priority projects but could be cool:

- **Sounding auto-fetch**: pull the launch site sounding straight from the
  University of Wyoming archive by date and station ID instead of downloading
  CSVs by hand. Small script, removes a pre-launch chore.
- **Run comparison overlay**: point the sim at two saved result sets and overlay
  them on the same plots (`--compare`). Great for "did my change actually matter"
  and for settling tuning arguments with pictures.
- **Extra derived plots**: dynamic pressure, Mach vs time with the deployment
  gate marked, and energy-to-apogee. All computable from the existing results
  array, no new physics needed.
- **Parameter sweep mode**: loop one config key over a range (brake area, target
  apogee, mass) and plot apogee vs that parameter. Turns brake sizing into one
  command instead of an afternoon of editing configs.
- **Monte Carlo dispersion**: jitter burnout state, CD, and mass, run a few
  hundred sims (they only take seconds each now), and histogram the apogees.
  Gives you an error bar on the target instead of a single point estimate.
- **KML trajectory export**: write sim trajectories as KML for Google Earth,
  matching the Featherweight kml files already in Post Flight Data. Mostly fun
  now, actually useful for recovery planning once descent gets modeled.
- **Pre-flight card generator**: one command that renders a printable summary
  (config, sounding date, predicted apogee with dispersion, deployment profile)
  to bring to the pad.

