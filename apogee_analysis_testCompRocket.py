import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from itertools import starmap
from numpy.typing import ArrayLike


class ActiveDrag:
    def __init__(self):
        # configurable
        self.target_apogee: int = 8455
        self.diameter: float = 0.158  # m
        self.burnout_mass: float = 40.62  # kg
        self.total_mass: float = 59.95  # kg
        self.total_impulse: int = 39734  # newton seconds
        self.burn_time: float = 9.5  # second

        self.propellant_mass: float = self.total_mass - self.burnout_mass  # kg
        self.avg_thrust: float = self.total_impulse / self.burn_time  # avg thrust
        self.gravity: float = 9.80665  # m/s/s
        self.dt: float = 0.1  # s

        # mutable attributes
        self.drag_args: tuple[ArrayLike, float, float]
        self.initialDeployFlag: bool = False
        self.burnout: bool = False
        self.mass: float = self.total_mass  # kg
        self.thrust: float = self.avg_thrust  # newton seconds

    @staticmethod
    def cd_interp(cd_array: ArrayLike, velocity: float, percent_deploy: float) -> float:
        sound_speed = 340
        mach_num = velocity/sound_speed
        mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
        deploy_pts = [0, 33, 100]  # airbrakes off, half, or all on
        i: int = 0
        j: int = 0
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

        if (i == -1) or (i == len(mach_pts)-1):  # if smallest or largest mach value
            if i == -1:
                i = 0
            f1 = cd_array[i, j]  # mach, closest smaller deploy value
            f2 = cd_array[i, j+1]  # mach, closest greater deploy value
        else:
            f1 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i, j] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1, j]
            f2 = (mach_pts[i+1] - mach_num)/(mach_pts[i+1] - mach_pts[i])*cd_array[i, j+1] + (mach_num - mach_pts[i])/(mach_pts[i+1] - mach_pts[i])*cd_array[i+1, j+1]
        # Interpolate CD values
        cd = (deploy_pts[j+1] - percent_deploy)/(deploy_pts[j+1] - deploy_pts[j])*f1 + (percent_deploy - deploy_pts[j])/(deploy_pts[j+1] - deploy_pts[j])*f2
        return cd

    @staticmethod
    def atm_density(h: float) -> float:
        """Calculate atmospheric density at altitude

        Parameters
        ----------
        h : float
            altitude

        Returns
        -------
        rho : float
            The atmospheric density at altitude ``h``
        """
        rho_b = 1.2250
        Tb = 288.15
        Lb = 0.0065
        hb = 0
        g0 = 9.80665
        R = 8.3144598
        M = 0.0289644
        rho = rho_b*((Tb - (h - hb)*Lb)/Tb)**((g0*M)/(R*Lb) - 1)
        return rho

    def get_drag(self, cd_array: ArrayLike, velocity: float, percent_deploy: float, altitude: float) -> float:
        A = np.pi*(self.diameter/2)**2
        rho = self.atm_density(altitude)
        cd = self.cd_interp(cd_array, velocity, percent_deploy)
        drag = 1/2*rho*velocity**2*cd*A
        return drag

    def get_acceleration(self, state: tuple[float, float]) -> float:
        altitude, velocity = state
        gravity = self.gravity
        mass = self.mass
        thrust = self.thrust
        cd_array, percent_deploy, _ = self.drag_args

        drag = self.get_drag(cd_array, velocity, percent_deploy, altitude)
        acceleration = (thrust - drag) / mass - gravity

        return acceleration

    def runge_kutta(self, k0_state: tuple[float, float]) -> tuple[float, float]:
        dt = self.dt
        state = k0_state

        def update_state(kn_state: tuple[float, float]) -> tuple[float, float]:
            r = tuple(starmap(lambda x, y: x + y * 1/2 * dt, (k0_state, kn_state)))
            return r[0], r[1]

        k1_velocity = state[1]  # k1 = f(y0, t0)
        k1_acceleration = self.get_acceleration(state)
        k1_state = (k1_velocity, k1_acceleration)
        state = update_state(k1_state)

        k2_velocity = state[1]  # k2 = f(y0+(k1 * dt/2), t0+(dt/2))
        k2_acceleration = self.get_acceleration(state)
        k2_state = (k2_velocity, k2_acceleration)
        state = update_state(k2_state)

        k3_velocity = state[1]  # k3 = f(y0+(k2 * dt/2), t0+(dt/2))
        k3_acceleration = self.get_acceleration(state)
        k3_state = (k3_velocity, k3_acceleration)
        state = update_state(k3_state)

        k4_velocity = state[1]  # k4 = f(y0+(k3*dt), t0+dt)
        k4_acceleration = self.get_acceleration(state)

        # y1 = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt
        altitude, velocity = k0_state
        alt_f = altitude + 1/6 * (k1_velocity + 2 * k2_velocity + 2 * k3_velocity + k4_velocity) * dt
        vel_f = velocity + 1/6 * (k1_acceleration + 2 * k2_acceleration + 2 * k3_acceleration + k4_acceleration) * dt

        return alt_f, vel_f

    def deploy_brakes(self, target_apogee: int, state: tuple[float, float]) -> float:
        _, percent_deploy, _ = self.drag_args
        dt = self.dt
        altitude, velocity = state

        apogee_error = 5
        deploy_time = 3

        # propogate to apogee (zero velocity)
        while velocity > 0:
            state = self.runge_kutta(state)
            altitude, velocity = state

        # if overshooting, increase brake deployment
        if (altitude - target_apogee) > apogee_error:
            percent_deploy = percent_deploy + 100/(deploy_time/dt)

        # else if undershooting, reduce brake deployment
        elif (altitude - target_apogee) < -apogee_error:
            percent_deploy = percent_deploy - 100/(deploy_time/dt)

        percent_deploy = np.clip(percent_deploy, 0, 100)
        return percent_deploy

    def run_simulation(self, cd_array: ArrayLike, target_apogee: int = None) -> ArrayLike:
        dt = self.dt
        burn_time = self.burn_time
        target_apogee = self.target_apogee if not target_apogee else target_apogee

        # Setup/initialization
        n: int = 0
        time: list[float] = [0]
        altitude: list[float] = [0.0]
        velocity: list[float] = [0.0]
        acceleration: list[float] = [0.0]
        percent_deploy: list[float] = [0.0]
        self.drag_args = (cd_array, percent_deploy[0], self.diameter)

        # Run to apogee
        while velocity[n] >= 0:
            state = self.runge_kutta((altitude[n], velocity[n]))  # propogate to next altitude/velocity
            newAltitude, newVelocity = state
            n = n + 1
            time.append(n*dt)
            altitude.append(newAltitude)  # update to new runge kutta altitude
            velocity.append(newVelocity)  # update to new runge kutta velocity
            acceleration.append(self.get_acceleration(state))
            if time[n] > burn_time:  # begin after burnout
                if not self.burnout:
                    self.burnout = True  # turn off motor
                    self.thrust = 0.0
                if time[n] >= (burn_time + 1) and newVelocity < 411:
                    if not self.initialDeployFlag:
                        self.initialDeployFlag = True
                        print("Brake Deployment Altitude (FB): {:0.0f}".format(newAltitude))
                        print("Brake Deployment Velocity (FB): {:0.0f}".format(newVelocity))
                    percent_deploy.append(self.deploy_brakes(target_apogee, state))  # determine braking amount
                    self.drag_args = (cd_array, percent_deploy[n], self.diameter)  # add brake drag to computations
                else:
                    percent_deploy.append(0)  # brakes not deployed before burnout
            else:
                percent_deploy.append(0)  # brakes not deployed before burnout
                percentPropellantRemaining = (not self.burnout) * (1 - (time[n] / burn_time))
                self.mass = self.burnout_mass + (self.propellant_mass * percentPropellantRemaining)

        return np.array([time, altitude, velocity, acceleration, percent_deploy])


results_file = 'activeDrag_mach_cd_Comp.xlsx'

cd_base = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='D')['Cd'].tolist()
cd_fb50 = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='H')['Cd.1'].tolist()
cd_fb100 = pd.read_excel(results_file, skiprows=8, nrows=11, usecols='L')['Cd.2'].tolist()

cd_fb = np.array([cd_base, cd_fb50, cd_fb100]).transpose()

base_results = ActiveDrag().run_simulation(cd_fb, 11000)
fb_results = ActiveDrag().run_simulation(cd_fb)

projected_apogee = base_results[1, -1]
mach_pts = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]

target_apogee = ActiveDrag().target_apogee
print("Projected Apogee: {:0.0f} m".format(projected_apogee))
print("Target Apogee: {:0.0f} m".format(target_apogee))
print("Apogee (FB): {:0.0f} m".format(fb_results[1, -1]))
print("Brake Deployment (FB): {:0.0f}%".format(fb_results[4, -1]))

display_plots: bool = True

if display_plots:
    plt.figure()
    plt.plot(mach_pts, cd_base, 'C0x-')
    plt.plot(mach_pts, cd_fb[:, 1], 'C2x--')
    plt.plot(mach_pts, cd_fb[:, 2], 'C2x-')
    plt.title("Airbrake Drag")
    plt.xlabel("Mach Number")
    plt.ylabel("Drag Coefficient")
    plt.legend(["Base", "FB50", "FB100"], loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid()

    plt.figure()
    plt.plot([0, base_results[0, -1]], [projected_apogee, projected_apogee], 'k-.')
    plt.plot([0, base_results[0, -1]], [target_apogee, target_apogee], 'k--')
    plt.plot(base_results[0, :], base_results[1, :])

    plt.plot(fb_results[0, :], fb_results[1, :])

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend(["Projected Apogee", "Target Apogee", "Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0, :], base_results[2, :])

    plt.plot(fb_results[0, :], fb_results[2, :])

    plt.title("Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend(["Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0, :], base_results[3, :])
    plt.plot(fb_results[0, :], fb_results[3, :])
    plt.title("Acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend(["Base", "FB"])
    plt.grid()

    plt.figure()
    plt.plot(base_results[0, :], base_results[4, :], label="_nolegend_")
    plt.plot(fb_results[0, :], fb_results[4, :])
    plt.title("Brake Deployment")
    plt.xlabel("Time (s)")
    plt.ylabel("Brake Deployment (%)")
    plt.legend(["FB"])
    plt.ylim([0, 100])
    plt.grid()

    plt.show()
