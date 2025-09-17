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
import pandas as pd

# time step for numerical integration in seconds (every 0.1 seconds, calc velocity/position)
dt = 0.1
target_apogee = 8455
# excell file containing data results from comp flight (can be found in the ggl drive)
results_file = 'activeDrag_mach_cd_Comp.xlsx'

display_plots = True

# ignore first 8 rows, read 11 rows on collumn D (base collumn)
cd_base  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='D')['Cd'].tolist()
# reads H (50% deployed collumn)
cd_fb50  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='H')['Cd.1'].tolist()
# reads L (100% deployed collumn)
cd_fb100 = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='L')['Cd.2'].tolist()

cd_fb = np.array([cd_base, cd_fb50, cd_fb100]).transpose()
'''
Noah's Comments
The block above should be changed so that it's not taking in hard-coded excell data
if the excell layout changes, code breaks
No error checking if file is malformed or by-god missing somehow
No validation that expected data was actually read
Obviously only handles 0, 50, and 100% deployment which is ideally continuous
'''

def cd_interp(cd_array, velocity, percent_deploy):
    sound_speed = 340
    mach_num = velocity/sound_speed
    mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    deploy_pts = [0, 33, 100] # airbrakes off, half, or all on. angle based not percentage
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
def atm_density(altitude):
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
    rho = atm_density(altitude) # Get air density at current altitude
    cd = cd_interp(cd_array, velocity, percent_deploy) # Gets drag coeff via interpolation (interp function from above)
    drag = 1/2*rho*V**2*cd*A # standard aerodynamic drag equation 
    return drag

def get_acceleration(state, accel_consts, drag_args):
    altitude = state[0]
    velocity = state[1]
    mass = accel_consts[0]
    thrust = accel_consts[1]
    gravity = accel_consts[2]
    cd_array = drag_args[0]
    percent_deploy = drag_args[1]
    diameter = drag_args[2]
    drag = get_drag(cd_array, velocity, percent_deploy, altitude, diameter)
    acceleration = (thrust - drag)/mass - gravity
    return acceleration

def runge_kutta(state, accel_consts, drag_args, dt):
    altitude = state[0]
    velocity = state[1]

    k1_velocity = state[1] # k1 = f(y0, t0)
    k1_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + 1/2*k1_velocity*dt
    state[1] = velocity + 1/2*k1_acceleration*dt

    k2_velocity = state[1] # k2 = f(y0+(k1 * dt/2), t0+(dt/2))
    k2_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + 1/2*k2_velocity*dt
    state[1] = velocity + 1/2*k2_acceleration*dt

    k3_velocity = state[1] # k3 = f(y0+(k2 * dt/2), t0+(dt/2))
    k3_acceleration = get_acceleration(state, accel_consts, drag_args)
    state[0] = altitude + k3_velocity*dt
    state[1] = velocity + k3_acceleration*dt

    k4_velocity = state[1] # k4 = f(y0+(k3*dt), t0+dt)
    k4_acceleration = get_acceleration(state, accel_consts, drag_args)

    # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    state[0] = altitude + 1/6*(k1_velocity + 2*k2_velocity + 2*k3_velocity + k4_velocity)*dt
    state[1] = velocity + 1/6*(k1_acceleration + 2*k2_acceleration + 2*k3_acceleration + k4_acceleration)*dt

    return state

def deploy_brakes(target_apogee, state, accel_consts, drag_args, dt):
    apogee_error = 5
    deploy_time = 3
    percent_deploy = drag_args[1]

    # propogate to apogee (zero velocity)
    while state[1] > 0:
        state = runge_kutta(state, accel_consts, drag_args, dt)

    # if overshooting, increase brake deployment
    if (state[0] - target_apogee) > apogee_error:
        percent_deploy = percent_deploy + 100/(deploy_time/dt) 
    
    # else if undershooting, reduce brake deployment
    elif (state[0] - target_apogee) < -apogee_error:
        percent_deploy = percent_deploy - 100/(deploy_time/dt)
    
    percent_deploy = np.clip(percent_deploy, 0, 100)
    return percent_deploy

def run_simulation(cd_array, target_apogee, dt):
    ### Setup/initialization
    mass = 59.95 # kg
    burnout_mass = 40.62 # kg
    propellant_mass = mass - burnout_mass # kg
    diameter = 0.158 # m
    total_impulse = 39734 # newton seconds
    burn_time = 9.5 # second
    thrust = total_impulse / burn_time # avg thrust
    gravity = 9.80665 # m/s/s
    n = 0
    time = [0]
    altitude = [0]
    velocity = [0]
    acceleration = [0]
    percent_deploy = [0]
    accel_consts = [mass, thrust, gravity]
    drag_args = [cd_array, percent_deploy[0], diameter]

    # Event variables
    deploymentVelocity = 0
    deploymentAltitude = 0
    initialDeployFlag = False

    ### Run to apogee
    while velocity[n] >= 0:
        state = [altitude[n], velocity[n]] #prev state alt/vel
        state = runge_kutta(state, accel_consts, drag_args, dt) # propogate to next altitude/velocity
        n = n + 1
        time.append(n*dt) 
        altitude.append(state[0]) #update to new runge kutta altitude
        velocity.append(state[1]) #update to new velocity
        acceleration.append(get_acceleration(state, accel_consts, drag_args))
        if time[n] > burn_time: #begin after burnout
            accel_consts[1] = 0 #turn off motor
            accel_consts[0] = burnout_mass # remove motor mass
            if time[n] > (burn_time+1) and state[1] < 411:
                if(initialDeployFlag == False):
                    initialDeployFlag = True
                    deploymentVelocity = state[1]
                    deploymentAltitude = state[0]
                    print("Brake Deployment Velocity (FB): {:0.0f}".format(deploymentVelocity))
                    print("Brake Deployment Altitude (FB): {:0.0f}".format(deploymentAltitude))
                percent_deploy.append(deploy_brakes(target_apogee, state, accel_consts, drag_args, dt)) #determine braking amount
                drag_args[1] = percent_deploy[n] # add brake drag to computations
            else:
                percent_deploy.append(0) #brakes not deployed before burnout        
        else:
            percent_deploy.append(0) #brakes not deployed before burnout
            percentPropellantRemaining = 1 - (time[n] / burn_time)
            accel_consts[0] = burnout_mass + (percentPropellantRemaining * propellant_mass)

    output = np.array([time, altitude, velocity, acceleration, percent_deploy])
    return output

base_results = run_simulation(cd_fb, 11000, dt)
fb_results = run_simulation(cd_fb, target_apogee, dt)

projected_apogee = base_results[1,-1]
mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]

print("Projected Apogee: {:0.0f} m".format(projected_apogee))
print("Target Apogee: {:0.0f} m".format(target_apogee))
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
    plt.plot([0, base_results[0,-1]], [target_apogee, target_apogee], 'k--')
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