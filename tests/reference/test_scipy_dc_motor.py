#!/usr/bin/env python3
import argparse
import csv
import subprocess

import numpy as np
from scipy.integrate import solve_ivp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True)
    args = parser.parse_args()

    output = subprocess.check_output([args.trace], text=True)
    rows = list(csv.DictReader(output.splitlines()))
    times = np.array([float(row["time_s"]) for row in rows])
    observed = np.array([
        [float(row["current_a"]),
         float(row["angular_speed_rad_s"]),
         float(row["shaft_angle_rad"])]
        for row in rows
    ])

    resistance = 2.0
    inductance = 0.5
    back_emf = 0.1
    torque_constant = 0.1
    inertia = 0.02
    friction = 0.02
    voltage = 12.0
    load_torque = 0.2

    def derivative(_, state):
        current, speed, _angle = state
        return [
            (voltage - resistance * current - back_emf * speed) /
            inductance,
            (torque_constant * current - friction * speed - load_torque) /
            inertia,
            speed,
        ]

    reference = solve_ivp(
        derivative,
        (times[0], times[-1]),
        np.zeros(3),
        t_eval=times,
        rtol=1e-11,
        atol=1e-13,
        method="DOP853",
    ).y.T
    error = np.max(np.abs(observed - reference), axis=0)
    limits = np.array([2e-8, 2e-7, 2e-7])
    if np.any(error > limits):
        raise SystemExit(
            f"SciPy parity failed: max errors {error}, limits {limits}")
    print(f"SciPy DC motor parity passed: max errors {error}")


if __name__ == "__main__":
    main()

