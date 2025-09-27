import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

dt = 0.1
target_apogee = 8455
results_file = 'activeDrag_mach_cd_Comp.xlsx'

display_plots = True

cd_base  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='D')['Cd'].tolist()
cd_fb50  = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='H')['Cd.1'].tolist()
cd_fb100 = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='L')['Cd.2'].tolist()

cd_fb = np.array([cd_base, cd_fb50, cd_fb100]).transpose()



def cd_interp(cd_array, velocity, percent_deploy: float):
    sound_speed = 340
    mach_num = velocity/sound_speed
    mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    deploy_pts = [0, 33, 100] # airbrakes off, half, or all on
    i, j = 0, 0
    # Find closest mach point
    for mach_val in mach_pts: 
        if mach_val > mach_num:
            i = mach_pts.index(mach_val) - 1
            break
        else:
            i = len(mach_pts) - 1
    # Find closest deploy val
    for deploy_val in deploy_pts:
        if deploy_val > percent_deploy:
            j = deploy_pts.index(deploy_val) - 1
            break
        else:
            j = len(deploy_pts) - 2
    
    if (i == -1) or (i == len(mach_pts)-1): #if smallest or largest mach value
        if i == -1:
            i = 0
        f1 = cd_array[i,j] #mach, closest smaller deploy value
        f2 = cd_array[i,j+1] #mach, closest greater deploy value
    else:
        f1 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j]
        f2 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i,j+1] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1,j+1]
    # interpolate CD values
    cd = (deploy_pts[j+1] - percent_deploy)/(deploy_pts[j+1] - deploy_pts[j])*f1 + (percent_deploy - deploy_pts[j])/(deploy_pts[j+1] - deploy_pts[j])*f2
    return cd

def get_atmosphere_density(altitude: float) -> float:
    rho_b = 1.2250
    Tb = 288.15
    Lb = 0.0065
    hb = 0
    g0 = 9.80665
    R = 8.3144598
    M = 0.0289644
    h = altitude
    rho = rho_b*((Tb - (h - hb)*Lb)/Tb)**((g0*M)/(R*Lb) - 1)
    return rho

def get_drag(cd_array, velocity: float, percent_deploy: float, altitude: float, diameter: float) -> float:
    V = velocity
    A = np.pi*(diameter/2)**2
    rho = get_atmosphere_density(altitude)
    cd = cd_interp(cd_array, velocity, percent_deploy)
    drag = 1/2*rho*V**2*cd*A
    return drag

def get_acceleration(altitude: float, velocity: float, accel_consts, drag_args) -> float:
    mass = accel_consts[0]
    thrust = accel_consts[1]
    gravity = accel_consts[2]
    cd_array = drag_args[0]
    percent_deploy = drag_args[1]
    diameter = drag_args[2]
    drag = get_drag(cd_array, velocity, percent_deploy, altitude, diameter)
    acceleration = (thrust - drag)/mass - gravity
    return acceleration

def runge_kutta(altitude: float, velocity: float, accel_consts, drag_args, dt: float):
    
    k1_velocity = velocity # k1 = f(y0, t0)
    k1_acceleration = get_acceleration(altitude, k1_velocity, accel_consts, drag_args)

    k2_velocity = velocity + 1/2*k1_acceleration*dt # k2 = f(y0+(k1 * dt/2), t0+(dt/2))
    k2_acceleration = get_acceleration(altitude + 1/2*k1_velocity*dt, k2_velocity, accel_consts, drag_args)

    k3_velocity = velocity + 1/2*k2_acceleration*dt # k3 = f(y0+(k2 * dt/2), t0+(dt/2))
    k3_acceleration = get_acceleration(altitude + 1/2*k2_velocity*dt, k3_velocity, accel_consts, drag_args)

    k4_velocity = velocity + k3_acceleration*dt # k4 = f(y0+(k3*dt), t0+dt)
    k4_acceleration = get_acceleration(altitude + k3_velocity*dt, k4_velocity, accel_consts, drag_args)

    # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    
    return (
        altitude + 1/6*(k1_velocity + 2*k2_velocity + 2*k3_velocity + k4_velocity)*dt,
        velocity + 1/6*(k1_acceleration + 2*k2_acceleration + 2*k3_acceleration + k4_acceleration)*dt
    )

def deploy_brakes(target_apogee, altitude: float, velocity: float, accel_consts, drag_args, dt: float):
    apogee_error = 5
    deploy_time = 3
    percent_deploy = drag_args[1]

    # propogate to apogee (zero velocity)
    while velocity > 0:
        altitude, velocity = runge_kutta(altitude, velocity, accel_consts, drag_args, dt)

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
    mass = 59.95 # kg
    burnout_mass = 40.62 # kg
    propellant_mass = mass - burnout_mass # kg
    diameter = 0.158 # m
    total_impulse = 39734 # newton seconds
    burn_time = 9.5 # second
    thrust = total_impulse / burn_time # avg thrust
    gravity = 9.80665 # m/s/s
    n = 0
    time = [0.0]
    altitude = [0.0]
    velocity = [0.0]
    acceleration = [0.0]
    percent_deploy = [0.0]
    accel_consts = [mass, thrust, gravity]
    drag_args = [cd_array, percent_deploy[0], diameter]

    # Event variables
    deployment_velocity = None
    deployment_altitude = None

    ### Run to apogee
    while velocity[n] >= 0:
        altitude_current, velocity_current = (altitude[n], velocity[n]) #prev state alt/vel
        altitude_current, velocity_current = runge_kutta(altitude_current, velocity_current, accel_consts, drag_args, dt) # propogate to next altitude/velocity
        n = n + 1
        time.append(n*dt) 
        altitude.append(altitude_current) #update to new runge kutta altitude
        velocity.append(velocity_current) #update to new velocity
        acceleration.append(get_acceleration(altitude_current, velocity_current, accel_consts, drag_args))
        if time[n] > burn_time: #begin after burnout
            accel_consts[1] = 0 #turn off motor
            accel_consts[0] = burnout_mass # remove motor mass
            if time[n] > (burn_time+1) and velocity_current < 411:
                if deployment_altitude is None:
                    deployment_velocity = velocity_current
                    deployment_altitude = altitude_current
                    print("Brake Deployment Velocity (FB): {:0.0f}".format(deployment_velocity))
                    print("Brake Deployment Altitude (FB): {:0.0f}".format(deployment_altitude))
                percent_deploy.append(deploy_brakes(target_apogee, altitude_current, velocity_current, accel_consts, drag_args, dt)) #determine braking amount
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