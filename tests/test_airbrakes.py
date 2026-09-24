"""Unit tests for the airbrake predictor and solver (fast, no RocketPy flight)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import airbrakes as ab

MASS = 20.8
ALT = 1400.0 + 1200.0   # m ASL
VZ = 250.0
VH = 20.0


def test_vacuum_limit_matches_analytic_ballistic():
    """With drag effectively removed, apogee must match h + v^2 / (2 g)."""
    original = ab.rocket_cd
    ab.rocket_cd = lambda mach: 0.0
    try:
        apogee = ab.predict_apogee(ALT, 0.0, VZ, 0.0, MASS, dt=0.01)
    finally:
        ab.rocket_cd = original
    expected = ALT + VZ**2 / (2 * ab.G0)
    assert math.isclose(apogee, expected, rel_tol=2e-3)


def test_drag_lowers_apogee_vs_vacuum():
    vacuum = ALT + VZ**2 / (2 * ab.G0)
    assert ab.predict_apogee(ALT, VH, VZ, 0.0, MASS) < vacuum


def test_more_deployment_means_lower_apogee():
    apogees = [ab.predict_apogee(ALT, VH, VZ, d, MASS) for d in (0.0, 0.25, 0.5, 0.75, 1.0)]
    assert all(a > b for a, b in zip(apogees, apogees[1:]))


def test_solver_hits_target_within_tolerance():
    free = ab.predict_apogee(ALT, VH, VZ, 0.0, MASS)
    full = ab.predict_apogee(ALT, VH, VZ, 1.0, MASS)
    target = 0.5 * (free + full)
    d = ab.solve_deployment(ALT, VH, VZ, MASS, target, iters=14)
    assert 0.0 < d < 1.0
    assert abs(ab.predict_apogee(ALT, VH, VZ, d, MASS) - target) < 5.0


def test_solver_retracts_when_below_target():
    free = ab.predict_apogee(ALT, VH, VZ, 0.0, MASS)
    assert ab.solve_deployment(ALT, VH, VZ, MASS, free + 500.0) == 0.0


def test_solver_saturates_when_target_unreachable():
    full = ab.predict_apogee(ALT, VH, VZ, 1.0, MASS)
    assert ab.solve_deployment(ALT, VH, VZ, MASS, full - 500.0) == 1.0


def test_isa_sea_level():
    temp, rho = ab.isa(0.0)
    assert math.isclose(temp, 288.15)
    assert math.isclose(rho, 1.225, rel_tol=5e-3)
