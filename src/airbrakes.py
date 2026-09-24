"""Predictive airbrake controller.

Each control step, the controller forecasts apogee with a 2-D point-mass coast
model (gravity + drag that depends on Mach and brake deployment), then solves
for the deployment level that lands apogee on the target using bisection.

The forecast model deliberately uses *nominal* vehicle parameters. Real flights
deviate from nominal (mass, drag, thrust, wind), and the closed loop corrects
for that error every step. That is the point of the feedback design.
"""
import math

from rocket_model import DRAG_TABLE, LAUNCH_ELEVATION_M

G0 = 9.80665
R_AIR = 287.05
GAMMA = 1.4
REF_AREA = math.pi * (127 / 2000) ** 2  # rocket cross-section, m^2

BURNOUT_TIME = 3.7      # s, matches the motor definition
MAX_BRAKE_CD = 1.2      # extra Cd (rocket-area reference) at full deployment


def isa(alt_asl):
    """ISA temperature (K) and density (kg/m^3) below 11 km."""
    t = 288.15 - 0.0065 * alt_asl
    p = 101325.0 * (t / 288.15) ** 5.25588
    return t, p / (R_AIR * t)


def rocket_cd(mach):
    """Linear interpolation of the nominal rocket drag table."""
    pts = DRAG_TABLE
    if mach <= pts[0][0]:
        return pts[0][1]
    for (m0, c0), (m1, c1) in zip(pts, pts[1:]):
        if mach <= m1:
            return c0 + (c1 - c0) * (mach - m0) / (m1 - m0)
    return pts[-1][1]


def brake_cd(deployment):
    """Extra drag coefficient contributed by the brakes (linear in deployment)."""
    return MAX_BRAKE_CD * deployment


def predict_apogee(alt_asl, vh, vz, deployment, mass, dt=0.2):
    """Forecast apogee (m ASL) for a coasting vehicle at constant deployment.

    2-D point mass: horizontal speed vh, vertical speed vz, drag opposes the
    total velocity vector. Semi-implicit Euler integration.
    """
    z, vhh, vzz = alt_asl, vh, vz
    for _ in range(4000):
        if vzz <= 0.0:
            break
        temp, rho = isa(z)
        speed = math.hypot(vhh, vzz)
        mach = speed / math.sqrt(GAMMA * R_AIR * temp)
        cd = rocket_cd(mach) + brake_cd(deployment)
        k = 0.5 * rho * cd * REF_AREA / mass
        vzz += (-G0 - k * speed * vzz) * dt
        vhh += (-k * speed * vhh) * dt
        z += vzz * dt
    return z


def solve_deployment(alt_asl, vh, vz, mass, target_asl, iters=10):
    """Bisection on deployment in [0, 1] so predicted apogee hits the target.

    Returns 0 if the vehicle is already predicted to fall short (brakes cannot
    add altitude) and 1 if even full deployment overshoots.
    """
    if predict_apogee(alt_asl, vh, vz, 0.0, mass) <= target_asl:
        return 0.0
    if predict_apogee(alt_asl, vh, vz, 1.0, mass) >= target_asl:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if predict_apogee(alt_asl, vh, vz, mid, mass) > target_asl:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def make_controller(target_agl_m, coast_mass, max_rate=0.5):
    """Build a RocketPy controller function.

    max_rate is the actuator slew limit in deployment-fraction per second.
    """
    target_asl = target_agl_m + LAUNCH_ELEVATION_M

    def controller(time, sampling_rate, state, state_history, observed_variables, air_brakes):
        if time < BURNOUT_TIME + 0.5:
            air_brakes.deployment_level = 0.0
            return (time, 0.0, 0.0)

        z, vx, vy, vz = state[2], state[3], state[4], state[5]
        if vz <= 0.0:
            air_brakes.deployment_level = 0.0
            return (time, 0.0, z)

        vh = math.hypot(vx, vy)
        command = solve_deployment(z, vh, vz, coast_mass, target_asl)
        dt = 1.0 / sampling_rate if sampling_rate else 0.05
        step = max_rate * dt
        current = air_brakes.deployment_level
        new = min(max(command, current - step), current + step)
        air_brakes.deployment_level = min(max(new, 0.0), 1.0)
        return (time, air_brakes.deployment_level, predict_apogee(z, vh, vz, air_brakes.deployment_level, coast_mass) - LAUNCH_ELEVATION_M)

    return controller
