#!/usr/bin/env python3
"""
Fit a first-order ARX model for drone longitudinal velocity:
    v[k+1] = a * v[k] + b * u[k]

Inputs:
    CSV file containing: time, velocity, control_input
Outputs:
    Identified model parameters (a, b)
    Simulation function to predict next velocity
"""

import numpy as np
import pandas as pd


# ---------------------------------------------------------
# 1. Load CSV data
# ---------------------------------------------------------
def load_data(csv_path):
    df = pd.read_csv(csv_path)

    # Expected columns — change names here if your log differs
    vel = df["g_veh_vel_x_est"].to_numpy()             # drone forward velocity (m/s)
    u_cmd = df["g_ctrl_vel_x_cmd"].to_numpy()         # control input (from PID+FF)
    
    # Ensure shapes align
    n = min(len(vel), len(u_cmd))
    vel = vel[:n]
    u_cmd = u_cmd[:n]

    return vel, u_cmd


# ---------------------------------------------------------
# 2. Fit ARX(1,1) using least squares
# ---------------------------------------------------------
def fit_arx(vel, u_cmd):
    """
    Fit the model:
        v[k+1] = a * v[k] + b * u[k]
    """

    # Prepare matrices for least squares
    v_k = vel[:-1]        # v[k]
    v_k1 = vel[1:]        # v[k+1]
    u_k = u_cmd[:-1]      # u[k]

    # Construct regressor matrix: [v[k], u[k]]
    X = np.column_stack([v_k, u_k])

    # Least squares solve: v[k+1] = X * theta
    theta, *_ = np.linalg.lstsq(X, v_k1, rcond=None)

    a, b = theta
    return a, b


# ---------------------------------------------------------
# 3. Build the plant model function
# ---------------------------------------------------------
def make_plant_function(a, b):
    """
    Returns a callable function:
        next_v = plant(current_v, control_u)
    """

    def plant(v, u):
        return a*v + b*u

    return plant


# ---------------------------------------------------------
# 4. Example usage
# ---------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Fit ARX velocity model")
    parser.add_argument("csv", type=str, help="Path to CSV with v_d and u_cmd")
    args = parser.parse_args()

    print("\nLoading data...")
    vel, u_cmd = load_data(args.csv)

    print("Fitting ARX(1,1) model...")
    a, b = fit_arx(vel, u_cmd)

    print("\n=========================================")
    print(" Identified ARX Velocity Model: ")
    print("   v[k+1] = a * v[k] + b * u[k]")
    print("-----------------------------------------")
    print(f" a = {a:.6f}")
    print(f" b = {b:.6f}")
    print("=========================================\n")

    # Build plant model
    plant = make_plant_function(a, b)

    # Example prediction
    v0 = vel[0]
    u0 = u_cmd[0]
    v1_hat = plant(v0, u0)
    print(f"Example prediction:")
    print(f"  Given v0={v0:.3f}, u0={u0:.3f} → v1_hat={v1_hat:.3f}")
    print()
