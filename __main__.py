'''
Active Drag Flight Simulation
Simulates sounding rocket flight with active airbrake control to hit a precise target altitude

Sim uses: 
    4th order Runge-Kutta integration for flight dynamics
    Bilinear interpolation for drag coeff calculation
    Proportional control for airbrake deployment
    
'''
import matplotlib.pyplot as plt
import numpy as np
import json
import sys

# Reading from stdin to avoid hardcoding a filename, piped output from `cat` preferred
# cat simconfig.json | python3 .
CONFIG_DATA = json.loads(sys.stdin.read())
DT = float(CONFIG_DATA["timestep"])
TARGET_APOGEE = float(CONFIG_DATA["target_apogee"])
display_plots = bool(CONFIG_DATA["display_plots"])

cd_base: list[float]  = CONFIG_DATA["cd_data"]["0"]
cd_fb = np.array([CONFIG_DATA["cd_data"]["0"], CONFIG_DATA["cd_data"]["50"], CONFIG_DATA["cd_data"]["100"]]).transpose()


def cd_interp(cd_array, velocity, percent_deploy: float):
    sound_speed = 340
    mach_num = velocity/sound_speed
    mach_pts = CONFIG_DATA["cd_data"]["mach_numbers"]
    deploy_pts = [0, 33, 100] # airbrakes off, half, or all on
    i, j = 0, 0
    # Find closest mach point
    for mach_val in mach_pts: # loops through mach pts until it finds val higher than current mach pt
        if mach_val > mach_num:
            i = mach_pts.index(mach_val) - 1
            break
        else:
            i = len(mach_pts) - 1 # handles edge cases above mach 2.0 (extrapolates)
    # Find closest deploy val
    for deploy_val in deploy_pts: # loop to make sure breaks deploy at correct value based on mach
        if deploy_val > percent_deploy:
            j = deploy_pts.index(deploy_val) - 1
            break
        else: 
            j = len(deploy_pts) - 2 # bounds checker
    
    if (i == -1) or (i == len(mach_pts)-1): # if slower than mach 0.2 or faster than 2.0, don't interpolate
        if i == -1:
            i = 0
        f1 = cd_array[i,j] # mach, closest smaller deploy value
        f2 = cd_array[i,j+1] # mach, closest greater deploy value
    # interpolation
    else:
        f1 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j] # weight and drag coeff for lower mach point
        f2 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j+1] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j+1] # weight and drag coeff for upper mach point
    # interpolate CD values
    cd = (deploy_pts[j+1] - percent_deploy)/(deploy_pts[j+1] - deploy_pts[j])*f1 + (percent_deploy - deploy_pts[j])/(deploy_pts[j+1] - deploy_pts[j])*f2
    return cd
'''
    Noah's comments for above section:
    Sound speed MIGHT be too oversimplified since it doesn't account for altitude (but i'm a cs major so it could be negligible for all i know?)
    No bounds checking for if percent_deploy is negative or >100% (could cause array indexing errors)
    Extrapolation beyond 2.0 or 0.2 is scary but when the hell would this thing ever be deploying before mach .2, or after mach 2? 
    
    by the end of function call 
    Velocity converted to mach number
    found appropriate data brackets in both mach and deployment dimensions
    performed bilinear interpolation to estimate drag coefficient
    returned a cd value for use in drag force calculation
'''

# Calculates air density at any altitude using standard atmosphere model
def get_atmosphere_density(altitude: float) -> float:

    rho_b = 1.2250 # air density at sea level
    Tb = 288.15 # International Standard Atmosphere value (ISA)
    Lb = 0.0065 # Temp lapse rate in K/m (6.5 degree C per 1000m) Basically how fast temp drops with altitude
    hb = 0 # reference altitude in meters
    g0 = 9.80665 # gravitational acceleration in m/s^2 (assumes constant grav)
    R = 8.3144598 # Universal gas constant in j/(mol*K)
    M = 0.0289644 # Molar mass of air in kg/mol (doesn't account for humidity)
    h = altitude
    rho = rho_b*((Tb - (h - hb)*Lb)/Tb)**((g0*M)/(R*Lb) - 1) # barometric forumla for tropospheric conditions
    return rho


# Calculates total drag force acting on rocket using standard drag equation
def get_drag(cd_array, velocity, percent_deploy, altitude, diameter):
    V = velocity # velocity in m/s
    A = np.pi*(diameter/2)**2 # rocket cross-sectional area in m^2 (area of circle = pi * radius^2)
    rho = get_atmosphere_density(altitude) # Get air density at current altitude
    cd = cd_interp(cd_array, velocity, percent_deploy) # Gets drag coeff via interpolation (interp function from above)
    drag = 1/2*rho*V**2*cd*A # standard aerodynamic drag equation 

    return drag

def get_acceleration(altitude: float, velocity: float, mass: float, thrust: float, gravity: float, cd_array, percent_deploy: float, diameter: float) -> float:
    drag = get_drag(cd_array, velocity, percent_deploy, altitude, diameter)
    acceleration = (thrust - drag)/mass - gravity
    return acceleration

def runge_kutta(altitude: float, velocity: float, mass: float, thrust: float, gravity: float, cd_array, percent_deploy: float, diameter: float, dt: float):
    
    k1_velocity = velocity # k1 = f(y0, t0)
    k1_acceleration = get_acceleration(altitude, k1_velocity, mass, thrust, gravity, cd_array, percent_deploy, diameter)

    k2_velocity = velocity + 1/2*k1_acceleration*dt # k2 = f(y0+(k1 * dt/2), t0+(dt/2))
    k2_acceleration = get_acceleration(altitude + 1/2*k1_velocity*dt, k2_velocity, mass, thrust, gravity, cd_array, percent_deploy, diameter)

    k3_velocity = velocity + 1/2*k2_acceleration*dt # k3 = f(y0+(k2 * dt/2), t0+(dt/2))
    k3_acceleration = get_acceleration(altitude + 1/2*k2_velocity*dt, k3_velocity, mass, thrust, gravity, cd_array, percent_deploy, diameter)

    k4_velocity = velocity + k3_acceleration*dt # k4 = f(y0+(k3*dt), t0+dt)
    k4_acceleration = get_acceleration(altitude + k3_velocity*dt, k4_velocity, mass, thrust, gravity, cd_array, percent_deploy, diameter)

    # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    
    return (
        altitude + 1/6*(k1_velocity + 2*k2_velocity + 2*k3_velocity + k4_velocity)*dt,
        velocity + 1/6*(k1_acceleration + 2*k2_acceleration + 2*k3_acceleration + k4_acceleration)*dt
    )

def deploy_brakes(target_apogee, altitude: float, velocity: float, mass: float, thrust: float, gravity: float, cd_array, percent_deploy: float, diameter: float, dt: float):
    apogee_error = CONFIG_DATA["apogee_error_m"]
    deploy_time = CONFIG_DATA["deploy_time_s"]

    if thrust: # don't deploy brakes while burning
        return 0
    
    # propogate to apogee (zero velocity)
    while velocity > 0:
        altitude, velocity = runge_kutta(altitude, velocity, mass, thrust, gravity, cd_array, percent_deploy, diameter, dt)

    # if overshooting, increase brake deployment
    if (altitude - target_apogee) > apogee_error:
        percent_deploy = percent_deploy + 100/(deploy_time/dt) 
    
    # else if undershooting, reduce brake deployment
    elif (altitude - target_apogee) < -apogee_error:
        percent_deploy = percent_deploy - 100/(deploy_time/dt)
    
    percent_deploy = np.clip(percent_deploy, 0, 100)
    return percent_deploy

def run_simulation(cd_array, target_apogee: float, dt: float):
    ### Setup/initialization
    mass = float(CONFIG_DATA["rocket_total_mass_kg"]) # kg
    burnout_mass = float(CONFIG_DATA["rocket_burnout_mass_kg"]) # kg
    propellant_mass = mass - burnout_mass # kg
    diameter = float(CONFIG_DATA["rocket_diameter_m"]) # m
    total_impulse = float(CONFIG_DATA["total_impulse_ns"]) # newton seconds
    burn_time = float(CONFIG_DATA["burn_time_s"]) # second
    thrust = total_impulse / burn_time # avg thrust
    gravity = 9.80665 # m/s/s
    n = 0
    time = [0.0]
    altitude = [0.0]
    velocity = [0.0]
    acceleration = [0.0]
    percent_deploy = [0.0]

    # Event variables
    deployment_velocity = None
    deployment_altitude = None

    ### Run to apogee
    while velocity[n] >= 0:
        altitude_current, velocity_current = (altitude[n], velocity[n]) #prev state alt/vel
        altitude_current, velocity_current = runge_kutta(altitude_current, velocity_current, mass, thrust, gravity, cd_array, percent_deploy[n-1], diameter, dt) # propogate to next altitude/velocity
        n = n + 1
        time.append(n*dt) 
        altitude.append(altitude_current) #update to new runge kutta altitude
        velocity.append(velocity_current) #update to new velocity
        acceleration.append(get_acceleration(altitude_current, velocity_current, mass, thrust, gravity, cd_array, percent_deploy[n-1], diameter))
        percent_propellant_remaining = max(1 - (time[n] / burn_time), 0)
        mass = burnout_mass + (percent_propellant_remaining * propellant_mass)
        thrust = total_impulse / burn_time if percent_propellant_remaining else 0
        percent_deploy.append(deploy_brakes(target_apogee, altitude_current, velocity_current, mass, thrust, gravity, cd_array, percent_deploy[n-1], diameter, dt)) #determine braking amount   
        if time[n] > (burn_time+1) and velocity_current < 411 and deployment_altitude is None:
            deployment_velocity = velocity_current
            deployment_altitude = altitude_current
            print("Brake Deployment Velocity (FB): {:0.0f}".format(deployment_velocity))
            print("Brake Deployment Altitude (FB): {:0.0f}".format(deployment_altitude))
    
    return np.array([time, altitude, velocity, acceleration, percent_deploy])

base_results = run_simulation(cd_fb, 11000, DT)
fb_results = run_simulation(cd_fb, TARGET_APOGEE, DT)

projected_apogee = base_results[1,-1]
mach_pts = CONFIG_DATA["cd_data"]["mach_numbers"]

print("Projected Apogee: {:0.0f} m".format(projected_apogee))
print("Target Apogee: {:0.0f} m".format(TARGET_APOGEE))
print("Apogee (FB): {:0.0f} m".format(fb_results[1,-1]))

print("Brake Deployment (FB): {:0.0f}%".format(fb_results[4,-1]))




if(display_plots):

    plt.figure()
    plt.plot(mach_pts, cd_base, 'C0x-')
    plt.plot(mach_pts, cd_fb[:,1], 'C2x--')
    plt.plot(mach_pts, cd_fb[:,2], 'C2x-')
    plt.title("Airbrake Drag")
    plt.xlabel("Mach Number")
    plt.ylabel("Drag Coefficient")
    plt.legend(["Base", "FB50", "FB100"], loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid()

    plt.figure()
    plt.plot([0, base_results[0,-1]], [projected_apogee, projected_apogee], 'k-.')
    plt.plot([0, base_results[0,-1]], [TARGET_APOGEE, TARGET_APOGEE], 'k--')
    plt.plot(base_results[0,:], base_results[1,:])

    plt.plot(fb_results[0,:], fb_results[1,:])

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend(["Projected Apogee","Target Apogee", "Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0,:], base_results[2,:])

    plt.plot(fb_results[0,:], fb_results[2,:])

    plt.title("Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend(["Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0,:], base_results[3,:])
    plt.plot(fb_results[0,:], fb_results[3,:])
    plt.title("Acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend(["Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0,:], base_results[4,:], label="_nolegend_")
    plt.plot(fb_results[0,:], fb_results[4,:])
    plt.title("Brake Deployment")
    plt.xlabel("Time (s)")
    plt.ylabel("Brake Deployment (%)")
    plt.legend(["FB"])
    plt.ylim([0, 100])
    plt.grid()

    plt.show()