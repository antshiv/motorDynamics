#!/usr/bin/env python3
import argparse
import csv
import subprocess

import numpy as np
from scipy.integrate import solve_ivp


TWO_PI = 2.0 * np.pi


def trapezoid(angle):
    angle = angle % TWO_PI
    if angle < np.pi / 6.0:
        return 6.0 * angle / np.pi
    if angle < 5.0 * np.pi / 6.0:
        return 1.0
    if angle < 7.0 * np.pi / 6.0:
        return 1.0 - 6.0 * (angle - 5.0 * np.pi / 6.0) / np.pi
    if angle < 11.0 * np.pi / 6.0:
        return -1.0
    return -1.0 + 6.0 * (angle - 11.0 * np.pi / 6.0) / np.pi


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True)
    args = parser.parse_args()
    rows = list(csv.DictReader(subprocess.check_output(
        [args.trace], text=True).splitlines()))
    times = np.array([float(row["time_s"]) for row in rows])
    observed = np.array([[float(row[key]) for key in
                          ("ia_a", "ib_a", "ic_a", "speed_rad_s", "angle_rad")]
                         for row in rows])

    resistance = 0.4
    inductance = 0.01
    back_emf_constant = 0.02
    torque_constant = 0.02
    pole_pairs = 7.0
    inertia = 0.002
    friction = 0.001
    load_torque = 0.01
    excitation = 80.0

    def derivative(time, state):
        currents = state[:3]
        speed = state[3]
        angle = state[4]
        voltages = np.array([
            6.0 + 3.0 * np.sin(excitation * time),
            6.0 + 3.0 * np.sin(excitation * time - 2.0 * np.pi / 3.0),
            6.0 + 3.0 * np.sin(excitation * time + 2.0 * np.pi / 3.0),
        ])
        electrical_angle = pole_pairs * angle
        shapes = np.array([
            trapezoid(electrical_angle),
            trapezoid(electrical_angle - 2.0 * np.pi / 3.0),
            trapezoid(electrical_angle + 2.0 * np.pi / 3.0),
        ])
        back_emf = back_emf_constant * speed * shapes
        neutral = np.mean(voltages - back_emf - resistance * currents)
        current_dot = (
            voltages - neutral - resistance * currents - back_emf
        ) / inductance
        torque = torque_constant * np.dot(shapes, currents)
        speed_dot = (torque - load_torque - friction * speed) / inertia
        return np.concatenate((current_dot, [speed_dot, speed]))

    reference = solve_ivp(
        derivative, (times[0], times[-1]), np.zeros(5), t_eval=times,
        method="DOP853", rtol=2e-10, atol=1e-12,
    ).y.T
    errors = np.max(np.abs(observed - reference), axis=0)
    limits = np.array([3e-3, 3e-3, 3e-3, 3e-3, 3e-4])
    if np.any(errors > limits):
        raise SystemExit(f"SciPy BLDC parity failed: {errors}, limits {limits}")
    print(f"SciPy BLDC parity passed: max errors {errors}")


if __name__ == "__main__":
    main()
