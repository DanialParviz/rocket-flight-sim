"""Integration tests against the full RocketPy 6-DOF model."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from airbrakes import make_controller, predict_apogee
from rocket_model import LAUNCH_ELEVATION_M, make_environment, make_motor, make_rocket, run_flight

COAST_MASS = 20.8


@pytest.fixture(scope="module")
def free_flight():
    env = make_environment(wind_speed=0.0)
    return env, run_flight(env, make_rocket(make_motor()))


def test_nominal_apogee_in_expected_band(free_flight):
    env, flight = free_flight
    agl_ft = (flight.apogee - LAUNCH_ELEVATION_M) * 3.28084
    assert 9000 < agl_ft < 11500


def test_predictor_agrees_with_6dof_at_burnout(free_flight):
    """Validation: the fast 2-D point-mass predictor vs the full 6-DOF apogee."""
    env, flight = free_flight
    t_burn = 3.7 + 0.5
    row = min(flight.solution, key=lambda r: abs(r[0] - t_burn))
    z, vx, vy, vz = row[3], row[4], row[5], row[6]
    predicted = predict_apogee(z, (vx**2 + vy**2) ** 0.5, vz, 0.0, COAST_MASS)
    assert abs(predicted - flight.apogee) / (flight.apogee - LAUNCH_ELEVATION_M) < 0.03


def test_controller_lowers_apogee_toward_target(free_flight):
    env, free = free_flight
    target_m = 2743.2   # 9,000 ft
    ctrl = make_controller(target_m, COAST_MASS)
    braked = run_flight(env, make_rocket(make_motor(), airbrake_controller=ctrl))
    free_err = abs(free.apogee - LAUNCH_ELEVATION_M - target_m)
    braked_err = abs(braked.apogee - LAUNCH_ELEVATION_M - target_m)
    assert braked_err < 0.25 * free_err
