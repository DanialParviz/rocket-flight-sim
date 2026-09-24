"""Compare one flight with and without the airbrake controller."""
import argparse

from airbrakes import make_controller
from rocket_model import LAUNCH_ELEVATION_M, make_environment, make_motor, make_rocket, run_flight

M_TO_FT = 3.28084
COAST_MASS_KG = 20.8   # nominal dry mass + motor casing after burnout

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-ft", type=float, default=9000.0)
    args = parser.parse_args()
    target_m = args.target_ft / M_TO_FT

    env = make_environment(wind_speed=3.0)
    free = run_flight(env, make_rocket(make_motor()))
    ctrl = make_controller(target_m, COAST_MASS_KG)
    braked = run_flight(env, make_rocket(make_motor(), airbrake_controller=ctrl))

    for name, f in (("No brakes", free), ("With brakes", braked)):
        print(f"{name:12s} apogee: {(f.apogee - LAUNCH_ELEVATION_M) * M_TO_FT:8.0f} ft AGL")
    print(f"Target:      {args.target_ft:8.0f} ft AGL")
