"""Generic rocket model for 6-DOF simulation with RocketPy.

All parameters are generic, illustrative values for a mid-size high-power rocket.
Nothing here is tied to any team's vehicle.
"""
import math

from rocketpy import Environment, Flight, Rocket, SolidMotor

LAUNCH_LAT = 32.99          # deg (New Mexico high desert, illustrative)
LAUNCH_LON = -106.97        # deg
LAUNCH_ELEVATION_M = 1400   # m above sea level

# Generic drag table: [Mach, Cd] (used by the rocket model and the airbrake predictor)
DRAG_TABLE = [(0.1, 0.42), (0.8, 0.45), (1.2, 0.60), (2.0, 0.50)]

# Simple generic thrust curve: [time (s), thrust (N)]
THRUST_CURVE = [
    (0.0, 0.0), (0.05, 2600.0), (0.3, 2400.0), (1.5, 2100.0),
    (2.8, 1800.0), (3.4, 900.0), (3.7, 0.0),
]


def make_environment(wind_speed=0.0, wind_dir_deg=90.0):
    env = Environment(latitude=LAUNCH_LAT, longitude=LAUNCH_LON,
                      elevation=LAUNCH_ELEVATION_M)
    env.set_atmospheric_model(
        type="custom_atmosphere",
        pressure=None,
        temperature=None,   # None -> standard atmosphere (ISA)
        wind_u=[(0, wind_speed * math.sin(math.radians(wind_dir_deg)))],
        wind_v=[(0, wind_speed * math.cos(math.radians(wind_dir_deg)))],
    )
    return env


def make_motor(thrust_scale=1.0):
    curve = [(t, f * thrust_scale) for t, f in THRUST_CURVE]
    return SolidMotor(
        thrust_source=curve,
        dry_mass=1.8,
        dry_inertia=(0.125, 0.125, 0.002),
        nozzle_radius=33 / 1000,
        grain_number=5,
        grain_density=1815,
        grain_outer_radius=33 / 1000,
        grain_initial_inner_radius=15 / 1000,
        grain_initial_height=120 / 1000,
        grain_separation=5 / 1000,
        grains_center_of_mass_position=0.397,
        center_of_dry_mass_position=0.317,
        nozzle_position=0,
        burn_time=3.7,
        throat_radius=11 / 1000,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )


def make_rocket(motor, dry_mass=19.0, cd_scale=1.0, airbrake_controller=None):
    rocket = Rocket(
        radius=127 / 2000,
        mass=dry_mass,
        inertia=(6.321, 6.321, 0.034),
        power_off_drag=[(m, c * cd_scale) for m, c in DRAG_TABLE],
        power_on_drag=[(m, c * cd_scale) for m, c in DRAG_TABLE],
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=-1.255)
    rocket.set_rail_buttons(upper_button_position=0.082, lower_button_position=-0.618, angular_position=45)
    rocket.add_nose(length=0.55829, kind="vonKarman", position=1.278)
    rocket.add_trapezoidal_fins(n=4, root_chord=0.120, tip_chord=0.060, span=0.110,
                                position=-1.04956, cant_angle=0)
    if airbrake_controller is not None:
        rocket.add_air_brakes(
            drag_coefficient_curve=lambda deployment, mach: 1.2 * deployment,
            controller_function=airbrake_controller,
            sampling_rate=10,
            clamp=True,
            initial_observed_variables=[0.0, 0.0, 0.0],
            name="AirBrakes",
        )
    rocket.add_parachute("Main", cd_s=10.0, trigger=800, sampling_rate=105, lag=1.5)
    return rocket


def run_flight(env, rocket, rail_length=5.2, inclination=85, heading=0):
    return Flight(rocket=rocket, environment=env, rail_length=rail_length,
                  inclination=inclination, heading=heading,
                  terminate_on_apogee=False)
