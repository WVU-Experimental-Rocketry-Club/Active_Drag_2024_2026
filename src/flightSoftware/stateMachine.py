"""
stateMachine.py - Flight Phase State Machine
============================================

Purpose:
    Manages rocket flight phases and transitions between states. Ensures correct
    sequencing of events (boost, coast, airbrake deployment, descent, landing) and
    controls when different subsystems are active.

Flight Phases (States):
    - PAD: Rocket on launch pad, awaiting ignition detection
    - BOOST: Motor burning, high acceleration, no airbrake control
    - COAST: Post-burnout, unpowered flight, monitoring for deployment conditions
    - AIRBRAKE_ACTIVE: Active drag control engaged, adjusting deployment
    - APOGEE: Peak altitude reached, transitioning to descent
    - DESCENT: Falling, awaiting landing (future: parachute deployment)
    - LANDED: Touchdown detected, flight complete

State Transitions:
    - PAD → BOOST: Acceleration threshold exceeded (launch detection)
    - BOOST → COAST: Thrust termination detected (burnout)
    - COAST → AIRBRAKE_ACTIVE: Time > burnout + 1s AND velocity < 411 m/s
    - AIRBRAKE_ACTIVE → APOGEE: Velocity ≈ 0 (apogee detection)
    - APOGEE → DESCENT: Falling detected (negative velocity)
    - DESCENT → LANDED: Altitude near ground, low velocity

Key Functions:
    - FlightPhase(Enum): Enumeration of flight states
    - StateMachine class: Manages current state and transitions
    - update(): Evaluates transition conditions, updates current state
    - get_state(): Returns current flight phase
    
Hardware Integration:
    - Controls when sensors are polled (GPS active during coast/airbrakes)
    - Manages datalogger recording (start at launch, stop at landing)
    - Enables/disables airbrake control based on phase
    
Dependencies:
    - navigation.py for state estimation
    - ad_controller.py for deployment commands
    - datalogger.py for flight data recording
"""

from enum import Enum, auto


class FlightPhase(Enum):
    """Enumeration of rocket flight phases"""
    PAD = auto()              # On launch pad, pre-flight
    BOOST = auto()            # Motor burning, powered ascent
    COAST = auto()            # Post-burnout, unpowered ascent
    AIRBRAKE_ACTIVE = auto()  # Active drag control engaged
    APOGEE = auto()           # Peak altitude reached
    DESCENT = auto()          # Falling, parachute deployed
    LANDED = auto()           # Touchdown, flight complete


class StateMachine:
    """
    Manages flight phase transitions and system state.
    
    Attributes:
        current_phase (FlightPhase): Current flight phase
        phase_start_time (float): Time when current phase began
        burnout_time (float): Time when motor burnout occurred
        deployment_enabled (bool): Whether airbrakes can be deployed
        flight_events (dict): Dictionary of detected events and their timestamps
    """
    
    def __init__(self, config):
        """
        Initialize state machine with flight configuration.
        
        Args:
            config (dict): Configuration dictionary with thresholds and timing
                - launch_accel_threshold: Acceleration to detect liftoff (m/s²)
                - burnout_accel_threshold: Acceleration drop indicating burnout (m/s²)
                - deployment_delay: Time after burnout before enabling airbrakes (s)
                - deployment_velocity_max: Max velocity for airbrake deployment (m/s)
                - apogee_velocity_threshold: Velocity near zero for apogee detection (m/s)
                - landing_altitude: Altitude indicating touchdown (m)
                - landing_velocity: Velocity indicating landing (m/s)
        """
        self.config = config
        self.current_phase = FlightPhase.PAD
        self.phase_start_time = 0.0
        self.burnout_time = None
        self.deployment_enabled = False
        self.flight_events = {}
        
    def update(self, time, altitude, velocity, acceleration):
        """
        Update state machine based on current flight conditions.
        
        Args:
            time (float): Current simulation time (s)
            altitude (float): Current altitude (m)
            velocity (float): Current vertical velocity (m/s)
            acceleration (float): Current vertical acceleration (m/s²)
            
        Returns:
            FlightPhase: Updated current phase
        """
        old_phase = self.current_phase
        
        # State transition logic
        if self.current_phase == FlightPhase.PAD:
            if acceleration > self.config.get('launch_accel_threshold', 20.0):
                self._transition_to(FlightPhase.BOOST, time)
                self.flight_events['launch'] = time
                
        elif self.current_phase == FlightPhase.BOOST:
            if acceleration < self.config.get('burnout_accel_threshold', 5.0):
                self._transition_to(FlightPhase.COAST, time)
                self.burnout_time = time
                self.flight_events['burnout'] = time
                
        elif self.current_phase == FlightPhase.COAST:
            # Check deployment conditions
            time_since_burnout = time - self.burnout_time if self.burnout_time else 0
            deployment_delay = self.config.get('deployment_delay', 1.0)
            max_deploy_velocity = self.config.get('deployment_velocity_max', 411.0)
            
            if (time_since_burnout > deployment_delay and 
                velocity < max_deploy_velocity and 
                velocity > 0):
                self._transition_to(FlightPhase.AIRBRAKE_ACTIVE, time)
                self.deployment_enabled = True
                self.flight_events['deployment_start'] = time
                
        elif self.current_phase == FlightPhase.AIRBRAKE_ACTIVE:
            apogee_threshold = self.config.get('apogee_velocity_threshold', 1.0)
            if abs(velocity) < apogee_threshold:
                self._transition_to(FlightPhase.APOGEE, time)
                self.deployment_enabled = False
                self.flight_events['apogee'] = time
                
        elif self.current_phase == FlightPhase.APOGEE:
            if velocity < -apogee_threshold:
                self._transition_to(FlightPhase.DESCENT, time)
                self.flight_events['descent_start'] = time
                
        elif self.current_phase == FlightPhase.DESCENT:
            landing_alt = self.config.get('landing_altitude', 10.0)
            landing_vel = self.config.get('landing_velocity', 5.0)
            if altitude < landing_alt and abs(velocity) < landing_vel:
                self._transition_to(FlightPhase.LANDED, time)
                self.flight_events['landing'] = time
                
        return self.current_phase
    
    def _transition_to(self, new_phase, time):
        """Internal method to handle phase transitions"""
        self.current_phase = new_phase
        self.phase_start_time = time
        
    def get_state(self):
        """Returns current flight phase"""
        return self.current_phase
    
    def is_airbrake_enabled(self):
        """Returns whether airbrakes should be active"""
        return self.deployment_enabled
    
    def get_phase_duration(self, time):
        """Returns time elapsed in current phase"""
        return time - self.phase_start_time
    
    def get_flight_events(self):
        """Returns dictionary of flight events and their timestamps"""
        return self.flight_events
    
    def reset(self):
        """Reset state machine to initial conditions"""
        self.current_phase = FlightPhase.PAD
        self.phase_start_time = 0.0
        self.burnout_time = None
        self.deployment_enabled = False
        self.flight_events = {}