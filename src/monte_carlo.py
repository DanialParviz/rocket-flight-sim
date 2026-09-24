"""Monte Carlo dispersion analysis across randomized flight conditions.

Randomizes wind, launch angle, motor thrust, drag, and vehicle mass, then
reports apogee statistics. Runs in parallel with multiprocessing.
"""
import argparse
import multiprocessing as mp
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rocket_model import LAUNCH_ELEVATION_M, make_environment, make_motor, make_rocket, run_flight

M_TO_FT = 3.28084


def one_run(seed):
    rng = np.random.default_rng(seed)
    wind = max(0.0, rng.normal(4.0, 2.0))           # m/s
    wind_dir = rng.uniform(0, 360)                   # deg
    incl = rng.normal(85.0, 1.0)                     # launch rail angle, deg
    thrust_scale = rng.normal(1.0, 0.03)             # +/-3% motor variation
    cd_scale = rng.normal(1.0, 0.05)                 # +/-5% drag uncertainty
    dry_mass = rng.normal(19.0, 0.3)                 # kg

    env = make_environment(wind_speed=wind, wind_dir_deg=wind_dir)
    rocket = make_rocket(make_motor(thrust_scale), dry_mass=dry_mass, cd_scale=cd_scale)
    flight = run_flight(env, rocket, inclination=incl)
    return flight.apogee - LAUNCH_ELEVATION_M, flight.x(flight.t_final), flight.y(flight.t_final)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--runs", type=int, default=200)
    parser.add_argument("-w", "--workers", type=int, default=os.cpu_count())
    parser.add_argument("-o", "--out", default="../results")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    with mp.Pool(args.workers) as pool:
        results = pool.map(one_run, range(args.runs))

    apogee_m = np.array([r[0] for r in results])
    apogee_ft = apogee_m * M_TO_FT
    landing = np.array([[r[1], r[2]] for r in results])

    print(f"Runs: {args.runs}")
    print(f"Apogee mean: {apogee_ft.mean():8.0f} ft")
    print(f"Apogee std:  {apogee_ft.std():8.0f} ft ({100 * apogee_ft.std() / apogee_ft.mean():.1f}%)")
    print(f"Apogee 5-95%: {np.percentile(apogee_ft, 5):.0f} to {np.percentile(apogee_ft, 95):.0f} ft")

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    ax[0].hist(apogee_ft, bins=25, color="#3b6fb6", edgecolor="white")
    ax[0].axvline(apogee_ft.mean(), color="k", ls="--", label=f"mean {apogee_ft.mean():.0f} ft")
    ax[0].set_xlabel("Apogee AGL (ft)")
    ax[0].set_ylabel("Runs")
    ax[0].set_title("Apogee dispersion")
    ax[0].legend()
    ax[1].scatter(landing[:, 0], landing[:, 1], s=10, alpha=0.6, color="#c0392b")
    ax[1].scatter([0], [0], marker="x", color="k", label="launch pad")
    ax[1].set_xlabel("East (m)")
    ax[1].set_ylabel("North (m)")
    ax[1].set_title("Landing dispersion")
    ax[1].set_aspect("equal", "datalim")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "monte_carlo.png"), dpi=150)


if __name__ == "__main__":
    main()
