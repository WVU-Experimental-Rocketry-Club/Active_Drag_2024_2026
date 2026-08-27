# RASAero Flight Sim Exports

Drop RASAero flight simulation CSV exports in here (Flight Simulation -> run the
sim -> export the flight data). These are the time-series outputs (time, altitude,
velocity, etc), not the aero CD tables - those go in `data/aero/`.

`utilities/import_rasaero.py` reads one of these, finds the burnout point, and
fills in the `simulation_parameters` block of a config file so the burnout numbers
never have to be typed in by hand.

Name files by rocket and motor so it's obvious what they came from, e.g.
`irec2026_M3400.csv`.
