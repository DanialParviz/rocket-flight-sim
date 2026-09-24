"""Monte Carlo comparison: identical random conditions, brakes off vs on."""
import argparse
import multiprocessing as mp
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from airbrakes import make_controller
from rocket_model import LAUNCH_ELEVATION_M, make_environment, make_motor, make_rocket, run_flight

M_TO_FT = 3.28084
COAST_MASS_KG = 20.8
TARGET_FT = 9000.0


def one_run(seed):
    rng = np.random.default_rng(seed)
    wind = max(0.0, rng.normal(4.0, 2.0))
    wind_dir = rng.uniform(0, 360)
    incl = rng.normal(85.0, 1.0)
    thrust_scale = rng.normal(1.0, 0.03)
    cd_scale = rng.normal(1.0, 0.05)
    dry_mass = rng.normal(19.0, 0.3)

    env = make_environment(wind_speed=wind, wind_dir_deg=wind_dir)
    out = []
    for controlled in (False, True):
        ctrl = make_controller(TARGET_FT / M_TO_FT, COAST_MASS_KG) if controlled else None
        rocket = make_rocket(make_motor(thrust_scale), dry_mass=dry_mass, cd_scale=cd_scale, airbrake_controller=ctrl)
        flight = run_flight(env, rocket, inclination=incl)
        out.append((flight.apogee - LAUNCH_ELEVATION_M) * M_TO_FT)
    return tuple(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--runs", type=int, default=100)
    parser.add_argument("-w", "--workers", type=int, default=os.cpu_count())
    parser.add_argument("-o", "--out", default="../results")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    with mp.Pool(args.workers) as pool:
        results = np.array(pool.map(one_run, range(args.runs)))
    free, braked = results[:, 0], results[:, 1]

    def report(name, a):
        err = a - TARGET_FT
        print(f"{name:12s} mean {a.mean():7.0f} ft | std {a.std():5.0f} ft | mean abs error {np.abs(err).mean():5.0f} ft")

    print(f"Runs: {args.runs} | target {TARGET_FT:.0f} ft AGL")
    report("No brakes", free)
    report("With brakes", braked)
    print(f"Std reduction: {100 * (1 - braked.std() / free.std()):.0f}%")
    print(f"Runs that overshoot target by >100 ft: no brakes {np.sum(free > TARGET_FT + 100)}, with brakes {np.sum(braked > TARGET_FT + 100)}")
    print(f"Runs that fell short by >100 ft (brakes cannot add altitude): {np.sum(braked < TARGET_FT - 100)}")

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bins = np.linspace(min(free.min(), braked.min()) - 100, max(free.max(), braked.max()) + 100, 35)
    ax.hist(free, bins=bins, alpha=0.6, color="#c0392b", label=f"No brakes (std {free.std():.0f} ft)")
    ax.hist(braked, bins=bins, alpha=0.7, color="#3b6fb6", label=f"With brakes (std {braked.std():.0f} ft)")
    ax.axvline(TARGET_FT, color="k", ls="--", label=f"Target {TARGET_FT:.0f} ft")
    ax.set_xlabel("Apogee AGL (ft)")
    ax.set_ylabel("Runs")
    ax.set_title("Airbrake controller: apogee dispersion")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "airbrake_comparison.png"), dpi=150)


if __name__ == "__main__":
    main()
