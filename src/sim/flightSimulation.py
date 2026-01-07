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
    state[0] = altitude + 1/2 * k3_velocity*dt
    state[1] = velocity + 1/2 * k3_acceleration*dt

    k4_velocity = state[1] # k4 = f(y0+(k3*dt), t0+dt)
    k4_acceleration = get_acceleration(state, accel_consts, drag_args)

    # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
    state[0] = altitude + 1/6*(k1_velocity + 2*k2_velocity + 2*k3_velocity + k4_velocity)*dt
    state[1] = velocity + 1/6*(k1_acceleration + 2*k2_acceleration + 2*k3_acceleration + k4_acceleration)*dt

    return state

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