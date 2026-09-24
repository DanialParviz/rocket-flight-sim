"""Run one nominal flight and print key results."""
from rocket_model import make_environment, make_motor, make_rocket, run_flight

if __name__ == "__main__":
    env = make_environment(wind_speed=3.0)
    flight = run_flight(env, make_rocket(make_motor()))
    print(f"Apogee (AGL):      {flight.apogee - env.elevation:8.1f} m")
    print(f"Apogee time:       {flight.apogee_time:8.1f} s")
    print(f"Max velocity:      {flight.max_speed:8.1f} m/s")
    print(f"Rail exit speed:   {flight.out_of_rail_velocity:8.1f} m/s")
    print(f"Impact velocity:   {flight.impact_velocity:8.1f} m/s")
