def cd_interp(cd_array, velocity, percent_deploy):
    sound_speed = 340
    mach_num = velocity/sound_speed
    mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    deploy_pts = [0, 33, 100] # airbrakes off, half, or all on
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


def get_drag(cd_array, velocity, percent_deploy, altitude, diameter):
    V = velocity
    A = np.pi*(diameter/2)**2
    rho = atm_density(altitude)
    cd = cd_interp(cd_array, velocity, percent_deploy)
    drag = 1/2*rho*V**2*cd*A
    return drag