# Config Files

Each JSON file in here describes one rocket + motor + launch site combo. To sim a new
rocket, copy the closest existing config and change the numbers. `main.py` checks the
config when it loads and will tell you if anything is missing.

The sim starts at motor burnout, not liftoff. All the `burnout_*` values come from
RASAero. Don't type them in by hand - export the flight sim data from RASAero into
`data/flight_sims/` and run the importer, which finds burnout and fills them in
(along with `burnout_mass`), converting to metric:

```
python utilities/import_rasaero.py data/flight_sims/your_export.CSV configs/your_config.json
```

## Files

| File | Rocket |
|------|--------|
| `competition_rocket_2026.json` | 6" IREC 2026 competition rocket (flown at IREC, June 2026) |
| `shenandoah_sunrise_irec.json` | Shenandoah Sunrise IREC rocket |
| `4inAD_k1100.json` | 4" test rocket, K1100 motor |
| `4inAD_k2050.json` | 4" test rocket, K2050 motor |
| `4inAD_m2050.json` | 4" test rocket, M2050 motor (flown at Kansas, March 2026) |

## Keys

### dimensions
- `diameter` - body tube diameter, meters. Used for the rocket's reference area.
- `length`, `fin_span`, `nose_cone_length` - meters. Not currently used by the sim,
  just here for reference.

### mass_properties
- `burnout_mass` - kg, mass of the rocket after the motor burns out.

### file_paths
- `aero_file` - RASAero CSV export with the Mach vs CD table for this rocket.
- `weather_file` - weather balloon sounding CSV for the launch site
  (see `data/weather/`). The sim interpolates pressure/temp from this instead of
  the standard atmosphere.

### simulation_parameters
- `time_step` - seconds, RK4 integration timestep. 0.1 is fine.
- `launch_altitude` - meters ASL of the launch site ground level. This is what
  converts between AGL and ASL everywhere.
- `burnout_time` - seconds after liftoff, from RASAero.
- `burnout_altitude` - meters ASL, from RASAero.
- `burnout_verticalVelocity` / `burnout_horizontalVelocity` - m/s, from RASAero.
- `burnout_acceleration` - m/s^2 at burnout, from RASAero.
- `max_simulation_time` - seconds, not currently used.

### active_drag_system
- `target_apogee_AGL` - meters above ground level that the controller aims for.
  Note this is usually set a bit above the actual scoring target to account for
  the sim overpredicting drag (IREC 30k target is 9144 m AGL).
- `brake_face_area` - m^2, total frontal area of all brake faces at full deploy.
- `max_brake_force` - newtons, structural limit. Controller won't deploy further
  if predicted brake drag would exceed this.
- `max_deployment` / `min_deployment` - percent. The controller keeps the brakes
  stowed until deploying at `min_deployment` would still overshoot the target.
- `full_deploy_time` - seconds for the actuator to go 0 to 100%.
- `control_frequency` - Hz, how often the controller recomputes.
- `deployment_conditions.maximum_mach` - brakes stay stowed above this Mach.
- `deployment_conditions.min_time_after_burnout` - seconds, not currently used.

## Brake area reference

Frontal areas for the different brake face options, if we swap hardware again:

| Faces | Area (m^2) |
|-------|------------|
| 2x3 | 0.015483871 |
| 5x3 | 0.0232258 |
| IREC 2026 as-flown | 0.01899918781 |

Old alternate values that used to sit in the configs as `*2` keys: IREC official
target 9144 m AGL, max brake force 1112.06 N (250 lbf).

## Getting settings onto the rocket

The flight computer doesn't read these JSON files. `utilities/flightDataFile.py`
reads a config plus its aero/weather CSVs and generates
`flightCodeSrc/mmrAirbrake/config_data.h`, which gets compiled into the flight
code. If you change a config and want it on the rocket, rerun that script and
reflash. See the main README.
