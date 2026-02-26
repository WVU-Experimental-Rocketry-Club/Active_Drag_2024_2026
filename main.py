#!/usr/bin/env python3
"""
main.py - Active Drag Rocket Simulation - Main Entry Point
==========================================================
WVU Experimental Rocketry Club - IREC 2026 30k SRAD Category

Purpose:
    Entry point for running rocket trajectory simulations with active airbrake control.
    Loads configuration files, executes simulations, generates comparison plots, and
    outputs flight data for analysis.

Usage:
    python main.py [--config CONFIG_FILE] [--output OUTPUT_DIR] [--plot]
    
    Options:
        --config: Path to JSON configuration file (default: configs/competition_rocket_activedrag.json)
        --output: Directory for output files (default: data/simulation_results/)
        --plot: Generate matplotlib plots comparing baseline vs active drag trajectories

Simulation Modes:
    1. Baseline: No airbrakes (percent_deploy = 0%) - determines maximum apogee
    2. Active Drag: Airbrakes controlled to achieve target apogee
    3. Comparison: Runs both modes and plots side-by-side results

Workflow:
    1. Parse command-line arguments
    2. Load JSON configuration file (rocket params, motor data, target apogee)
    3. Load aerodynamic data (drag coefficient tables from data/aero/)
    4. Run baseline simulation (no brakes)
    5. Run active drag simulation (with controller)
    6. Generate comparison plots:
        - Altitude vs Time
        - Velocity vs Time  
        - Deployment Percentage vs Time
        - Predicted vs Actual Apogee
    7. Export flight data to CSV (data/simulation_results/)
    8. Print summary statistics (max altitude, burnout velocity, deployment stats)

Configuration Files:
    - configs/competition_rocket_activedrag.json: Full-scale IREC competition rocket
    - configs/example_rocket.json: Smaller test rocket (4" diameter, 10k ft target)

Dependencies:
    - sim.flightSimulation: run_simulation() - main trajectory propagation
    - core.aerodynamics: Load drag coefficient data
    - matplotlib: Plotting and visualization
    - json: Configuration file parsing
    - argparse: Command-line interface

Output Files:
    - baseline_trajectory.csv: Time-series data without airbrakes
    - active_drag_trajectory.csv: Time-series data with controller
    - comparison_plot.png: Visual comparison of trajectories
    - flight_summary.txt: Statistical summary of simulation results

Notes:
    - TODO: Implement config loading (currently stub)
    - TODO: Integrate state machine for event-driven simulation
    - TODO: Add Monte Carlo mode for uncertainty quantification
    - TODO: Support batch processing of multiple configurations
"""

import os
import sys
import json
from pathlib import Path
import argparse
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from src.flightSoftware.ad_controller import AirbrakeController
from src.sim.flightSimulation import run_simulation
from src.core import atmosphere


class SimulationRunner:
    """
    Orchestrates simulation execution, configuration loading, and result visualization.
    
    Attributes:
        config_path (Path): Path to configuration JSON file
        output_dir (Path): Directory for output files
        config (dict): Loaded configuration dictionary
        results (dict): Simulation results storage
    """
    
    def __init__(self, config_path, output_dir='data/simulation_results'):
        """
        Initialize simulation runner.
        
        Args:
            config_path (str or Path): Path to JSON configuration file
            output_dir (str or Path): Directory for output files
        """
        self.config_path = Path(config_path)
        with open(self.config_path, 'r') as f:
            self.rocketConfig = json.load(f)


        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.config = None
        self.results = {
            'baseline': None,
            'active_drag': None
        }
        
    def load_config(self):
        """
        Load simulation configuration from JSON file.
        
        Returns:
            dict: Configuration dictionary with rocket parameters, motor data, etc.
        """
        with open(self.config_path, 'r') as f:
            self.config = json.load(f)
        return self.config
    
    def load_aero_data(self):
        """
        Load aerodynamic drag coefficient data.
        
        Returns:
            np.ndarray: Drag coefficient lookup table
        """
        # TODO: Implement loading from Excel/CSV files
        # For now, return None - will be implemented when integrating legacy code

        results_file = 'legacy/activeDrag_mach_cd_Comp.xlsx'
        cd_base  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='D')['Cd'].tolist()
        cd_fb50  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='H')['Cd.1'].tolist()
        cd_fb100 = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='L')['Cd.2'].tolist()
        self.cd_fb = np.array([cd_base, cd_fb50, cd_fb100]).transpose()

        aero_file = self.config["file_paths"]["aero_file"]
        self.aero_file = pd.read_csv(aero_file)

        # Configure atmosphere module with weather data
        weather_file = self.config["file_paths"]["weather_file"]
        self.weather_data = pd.read_csv(weather_file)
        atmosphere.load_weather_data(self.weather_data)
        # atmosphere.use_isa_model()  # Use ISA for now
        
        return self.aero_file
    
    def run_baseline_simulation(self):
        """
        Run simulation with no airbrakes (baseline trajectory).
        
        Returns:
            dict: Baseline simulation results
                {'time': array, 'altitude': array, 'velocity': array, ...}
        """
        return run_simulation(self.rocketConfig, self.load_aero_data(), False)
    
    def run_active_drag_simulation(self):
        """
        Run simulation with active airbrake control.
        
        Returns:
            dict: Active drag simulation results
                {'time': array, 'altitude': array, 'velocity': array, 
                 'deployment': array, 'predicted_apogee': array}
        """
        return run_simulation(self.rocketConfig, self.load_aero_data(), True)
        
    
    def plot_results(self, show=True, save=True):
        """
        Generate comparison plots of baseline vs active drag trajectories.
        
        Args:
            show (bool): Display plots interactively
            save (bool): Save plots to output directory
        """
        # - Altitude vs Time (both trajectories)
        # - Velocity vs Time
        # - Deployment Percentage vs Time
        # - Predicted vs Actual Apogee
        target_apogee_AGL = self.config["active_drag_system"]["target_apogee_AGL"] # meters
        ground_level = self.config["simulation_parameters"]["launch_altitude"] # meters
        target_apogee = ground_level + target_apogee_AGL

        plt.style.use('default')
        px = 1/plt.rcParams['figure.dpi']  # pixel in inches
        fig, axs = plt.subplots(2, 3, constrained_layout=True)
        fig.set_size_inches(10, 6)

        axs[0, 0].plot([0, self.results['baseline'][0,-1]], [self.results['baseline'][1,-1], self.results['baseline'][1,-1]], 'k-.')
        axs[0, 0].plot([0, self.results['baseline'][0,-1]], [target_apogee, target_apogee], 'k--')
        axs[0, 0].plot(self.results['baseline'][0,:], self.results['baseline'][1,:])
        axs[0, 0].plot(self.results['active_drag'][0,:], self.results['active_drag'][1,:])
        axs[0, 0].set_title("Altitude")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Altitude (m)")
        axs[0, 0].legend(["Projected Apogee","Target Apogee", "Base", "FB"])
        axs[0, 0].grid()

        axs[0, 1].plot(self.results['baseline'][0,:], self.results['baseline'][2,:])
        axs[0, 1].plot(self.results['active_drag'][0,:], self.results['active_drag'][2,:])
        axs[0, 1].set_title("Velocity")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Velocity (m/s)")
        axs[0, 1].legend(["Base", "FB"])
        axs[0, 1].grid()

        axs[1, 0].plot(self.results['baseline'][0,:], self.results['baseline'][5,:])
        axs[1, 0].plot(self.results['active_drag'][0,:], self.results['active_drag'][5,:])
        axs[1, 0].set_title("Acceleration")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Acceleration (m/s^2)")
        axs[1, 0].legend(["Base", "FB"])
        axs[1, 0].grid()

        axs[1, 2].plot(self.results['baseline'][0,:], self.results['baseline'][6,:])
        axs[1, 2].plot(self.results['active_drag'][0,:], self.results['active_drag'][6,:])
        axs[1, 2].set_title("Brake Deployment Percentage")
        axs[1, 2].set_xlabel("Time (s)")
        axs[1, 2].set_ylabel("Brake Deployment (%)")
        axs[1, 2].legend(["Base", "Airbrake Deployment (%)"])
        axs[1, 2].set_ylim([-10, 110])
        axs[1, 2].grid()

        axs[1, 1].plot(self.results['baseline'][0,:], self.results['baseline'][6,:])
        axs[1, 1].plot(self.results['active_drag'][0,:], self.results['active_drag'][7,:])
        axs[1, 1].set_yticks(range(0, 91, 15))
        axs[1, 1].set_title("Brake Deployment (deg))")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Brake Deployment (deg)")
        axs[1, 1].legend(["Base", "Airbrake Deployment (deg)"])
        axs[1, 1].set_ylim([-10, 100])
        axs[1, 1].grid()

        maxBrakeForce = self.rocketConfig["active_drag_system"]["max_brake_force"]
        axs[0, 2].plot([0, self.results['active_drag'][0,-1]], [maxBrakeForce, maxBrakeForce], 'k--')
        axs[0, 2].plot(self.results['baseline'][0,:], self.results['baseline'][8,:])
        axs[0, 2].plot(self.results['active_drag'][0,:], self.results['active_drag'][8,:])
        axs[0, 2].set_title("Brake Force")
        axs[0, 2].set_xlabel("Time (s)")
        axs[0, 2].set_ylabel("Brake Force (N)")
        axs[0, 2].legend(["Maximum allowed brake force", "Base", "Airbrake Force (N)"])
        axs[0, 2].grid()
        plt.show()
        
    
    def export_results(self):
        """Export simulation results to CSV files"""
    
    def print_summary(self):
        """Print statistical summary of simulation results"""
        # TODO: Calculate and print:
        # - Max altitude (baseline vs active drag)
        # - Burnout velocity
        # - Apogee prediction accuracy
        # - Deployment statistics (max, mean, time spent deployed)
        projected_apogee_ASL = self.results['baseline'][1,-1]
        projected_apogee_AGL = projected_apogee_ASL - self.config["simulation_parameters"]["launch_altitude"]
        target_apogee_ASL = self.config["simulation_parameters"]["launch_altitude"] + self.config["active_drag_system"]["target_apogee_AGL"]
        target_apogee_AGL = self.config["active_drag_system"]["target_apogee_AGL"]
        airbrakeApogeeASL = self.results['active_drag'][1,-1]
        airbrakeApogeeAGL = self.results['active_drag'][1,-1] - self.config["simulation_parameters"]["launch_altitude"]
        apogeeReduction = self.results['baseline'][1,-1] - self.results['active_drag'][1,-1]
        apogeeError = self.results['active_drag'][1,-1] - (self.config["simulation_parameters"]["launch_altitude"] + self.config["active_drag_system"]["target_apogee_AGL"])


        print("\n--------------------------------------")
        print("Metric:")
        print("Projected Apogee: {:0.0f} m ({:0.0f} m AGL)".format(projected_apogee_ASL, projected_apogee_AGL))
        print("Target Apogee: {:0.0f} m ({:0.0f} m AGL)".format(target_apogee_ASL, target_apogee_AGL))
        print("Apogee With Airbrakes (ASL): {:0.0f} m".format(airbrakeApogeeASL))
        print("Apogee With Airbrakes (AGL): {:0.0f} m".format(airbrakeApogeeAGL))
        print("Apogee Reduction (FB): {:0.0f} m".format(apogeeReduction))
        print("Apogee Error: {:0.0f} m".format(apogeeError))

        print("\n--------------------------------------")
        print("Imperial:")
        print("Projected Apogee: {:0.0f} ft ({:0.0f} ft AGL)".format(projected_apogee_ASL * 3.28084, projected_apogee_AGL * 3.28084))
        print("Target Apogee: {:0.0f} ft ({:0.0f} ft AGL)".format(target_apogee_ASL * 3.28084, target_apogee_AGL * 3.28084))
        print("Apogee With Airbrakes (ASL): {:0.0f} ft".format(airbrakeApogeeASL * 3.28084))
        print("Apogee With Airbrakes (AGL): {:0.0f} ft".format(airbrakeApogeeAGL * 3.28084))
        print("Apogee Reduction (FB): {:0.0f} ft".format(apogeeReduction * 3.28084))
        print("Apogee Error: {:0.0f} ft".format(apogeeError * 3.28084))
        print("\n--------------------------------------")

        print("Brake Deployment (FB): {:0.0f}%".format(self.results['active_drag'][6,-1]))
        print("Brake Deployment Angle (FB): {:0.0f} degrees".format(self.results['active_drag'][7,-1]))

        
    
    def run_full_analysis(self, plot=True):
        """
        Run complete analysis: baseline, active drag, plotting, export.
        
        Args:
            plot (bool): Whether to generate plots
            
        Returns:
            dict: Complete results from both simulations
        """
        print("Loading configuration...")
        self.load_config()
        
        print("Loading aerodynamic data...")
        self.load_aero_data()
        
        print("Running baseline simulation...")
        self.results['baseline'] = self.run_baseline_simulation()
        
        print("Running active drag simulation...")
        self.results['active_drag'] = self.run_active_drag_simulation()
        
        print("Exporting results...")
        self.export_results()

        self.print_summary()

        if plot:
            print("Generating plots...")
            self.plot_results()
        

        
        return self.results


def main():
    """Main entry point for command-line execution"""
    parser = argparse.ArgumentParser(
        description='Active Drag Rocket Simulation - WVU Experimental Rocketry'
    )
    parser.add_argument(
        '--config', 
        type=str, 
        #default='configs/competition_rocket_2026.json',
        default='configs/4inAD.json',
        help='Path to configuration JSON file'
    )
    parser.add_argument(
        '--output', 
        type=str, 
        default='data/simulation_results',
        help='Output directory for results'
    )
    parser.add_argument(
        '--plot', 
        action='store_true',
        help='Generate comparison plots'
    )
    parser.add_argument(
        '--no-plot', 
        action='store_true',
        help='Skip plot generation'
    )
    
    args = parser.parse_args()
    
    # Create simulation runner
    runner = SimulationRunner(args.config, args.output)
    
    # Run full analysis
    plot = args.plot or not args.no_plot
    results = runner.run_full_analysis(plot=plot)
    
    print("\nSimulation complete!")
    print(f"Results saved to: {runner.output_dir}")


if __name__ == '__main__':
    main()
