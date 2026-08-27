"""
import_rasaero.py - Pull burnout conditions out of a RASAero flight sim export
==============================================================================

Reads a RASAero flight simulation CSV (from data/flight_sims/), finds the burnout
point, and writes the burnout state into a config file's simulation_parameters
block. Also updates burnout_mass from the Weight column. This replaces reading the
numbers off the RASAero screen and typing them into the config by hand.

Usage:
    python utilities/import_rasaero.py data/flight_sims/mmr_irec2026_finalPreflight.CSV configs/competition_rocket_2026.json

Notes:
    - RASAero exports in imperial (ft, lb, ft/s^2), configs are metric
    - RASAero altitude is AGL from the pad, configs store burnout_altitude ASL,
      so the config's launch_altitude gets added on
    - Burnout is the first row where thrust returns to zero after ignition
    - Single stage flights only
"""

import sys
import json
import argparse
import pandas as pd

FT_TO_M = 0.3048
LB_TO_KG = 0.45359237


def find_burnout(flight_data):
    """Returns the first row where thrust has returned to zero after ignition"""
    thrust = flight_data["Thrust (lb)"]
    ignition_index = thrust[thrust > 0].index[0]
    burned_out = thrust[thrust.index > ignition_index] == 0
    if not burned_out.any():
        print("Never found burnout - thrust is nonzero to the end of the file")
        sys.exit(1)
    return flight_data.loc[burned_out.idxmax()]


def main():
    parser = argparse.ArgumentParser(
        description="Fill a config's burnout state from a RASAero flight sim export"
    )
    parser.add_argument("flight_csv", help="RASAero flight simulation CSV export")
    parser.add_argument("config", help="Config JSON file to update")
    args = parser.parse_args()

    flight_data = pd.read_csv(args.flight_csv)

    expected_cols = ["Time (sec)", "Thrust (lb)", "Weight (lb)", "Accel-V (ft/sec^2)",
                     "Vel-V (ft/sec)", "Vel-H (ft/sec)", "Altitude (ft)"]
    missing = [c for c in expected_cols if c not in flight_data.columns]
    if missing:
        print("{} doesn't look like a RASAero flight sim export.".format(args.flight_csv))
        print("Missing columns: {}".format(", ".join(missing)))
        print("(export the flight simulation data, not the aero plot CSV)")
        sys.exit(1)

    with open(args.config, "r", encoding="utf-8-sig") as f:
        config = json.load(f)

    launch_altitude = config["simulation_parameters"]["launch_altitude"]

    burnout = find_burnout(flight_data)

    updates = {
        "burnout_time": round(float(burnout["Time (sec)"]), 2),
        "burnout_altitude": round(float(burnout["Altitude (ft)"]) * FT_TO_M + launch_altitude, 1),
        "burnout_verticalVelocity": round(float(burnout["Vel-V (ft/sec)"]) * FT_TO_M, 2),
        "burnout_horizontalVelocity": round(float(burnout["Vel-H (ft/sec)"]) * FT_TO_M, 2),
        "burnout_acceleration": round(float(burnout["Accel-V (ft/sec^2)"]) * FT_TO_M, 2),
    }
    burnout_mass = round(float(burnout["Weight (lb)"]) * LB_TO_KG, 2)

    print("Burnout from {}:".format(args.flight_csv))
    for key, value in updates.items():
        old = config["simulation_parameters"].get(key, "not set")
        print("  {}: {} -> {}".format(key, old, value))
        config["simulation_parameters"][key] = value
    old_mass = config["mass_properties"].get("burnout_mass", "not set")
    print("  burnout_mass: {} -> {}".format(old_mass, burnout_mass))
    config["mass_properties"]["burnout_mass"] = burnout_mass

    # a couple of sanity numbers so a bad export or wrong rocket stands out
    apogee_agl = flight_data["Altitude (ft)"].max() * FT_TO_M
    print("\nRASAero projected apogee: {:0.0f} m AGL ({:0.0f} ft)".format(
        apogee_agl, apogee_agl / FT_TO_M))
    print("(altitude ASL = RASAero AGL + launch_altitude of {} m from the config)".format(launch_altitude))

    with open(args.config, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")

    print("\nUpdated {}".format(args.config))


if __name__ == "__main__":
    main()
