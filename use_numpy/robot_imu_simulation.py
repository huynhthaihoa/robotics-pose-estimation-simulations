'''
This script simulates a robot attempting to track a moving target trajectory over 
several timesteps under the same unbalanced noise profile 
(highly accurate GPS translation, but imperfect IMU).
'''

import numpy as np
import argparse

from lie_utils import se3_exp, se3_log, compute_se3_inv_right_jacobian

# --- Main IMU & Sensor Update Simulation ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    # Simulation Parameters Configuration
    parser.add_argument("--dt-imu", type=float, default=0.01, help="IMU update interval in seconds (default: 0.01s for 100Hz)")
    parser.add_argument("--total-seconds", type=int, default=3, help="Total simulation duration in seconds")
    parser.add_argument("--snapshots-per-second", type=int, default=4, help="Number of IMU tracking snapshots to log per simulated second")

    # Gauss-Newton Convergence Criteria configuration
    parser.add_argument("--gn-tol", type=float, default=1e-6, help="Gauss-Newton convergence tolerance")
    parser.add_argument("--gn-max-iters", type=int, default=10, help="Maximum Gauss-Newton iterations")
    
    # Velocity Magnitude Configuration (for random twist generation)
    parser.add_argument("--max-linear-vel", type=float, default=1.0, help="Maximum linear velocity magnitude in m/s (default: 1.0)")
    parser.add_argument("--max-angular-vel", type=float, default=0.5, help="Maximum angular velocity magnitude in rad/s (default: 0.5)")
    
    args = parser.parse_args()

    np.set_printoptions(suppress=True, precision=4)
    
    # 1. Setup Initial States (Both ground-truth and robot estimate start at origin)
    T_true = np.eye(4)
    T_est = np.eye(4)
    
    # 2. Configure Simulation Parameters
    dt_imu = args.dt_imu          # IMU updates at 100 Hz (every 10ms)
    total_seconds = args.total_seconds      # Run the simulation for 3 seconds total
    
    # The true movement of the robot: a randomized 6D body-frame twist velocity
    # [vx, vy, vz, wx, wy, wz] -- linear velocity in m/s, angular velocity in rad/s
    true_twist_velocity = np.concatenate([
        np.random.uniform(-args.max_linear_vel, args.max_linear_vel, 3),   # linear velocity ∈ [-max_linear_vel, max_linear_vel] m/s per axis
        np.random.uniform(-args.max_angular_vel, args.max_angular_vel, 3)    # angular velocity ∈ [-max_angular_vel, max_angular_vel] rad/s per axis
    ])
    
    # 3. Setup Correction Weights (Precision GPS for position, noisy gyro)
    unbalanced_cov = np.zeros((6, 6))
    np.fill_diagonal(unbalanced_cov, [0.001, 0.001, 0.001, 1000.0, 1000.0, 1000.0])
    omega_info = np.linalg.inv(unbalanced_cov)

    # 4. Setup Gauss-Newton Convergence Criteria (replaces a fixed iteration count)
    gn_tol = args.gn_tol        # stop once the correction step norm drops below this
    gn_max_iters = args.gn_max_iters    # safety cap in case convergence stalls

    print("Starting IMU Propagation Timeline Simulation...")
    print("IMU frequency: 100 Hz | Main Global Sensor updates: 1 Hz")
    print(f"Initial State: X={T_est[0,3]:.2f}, Y={T_est[1,3]:.2f}\n")
    print("-" * 90)
    
    # Master Timeline Loop
    imu_steps_per_second = int(1.0 / dt_imu)
    snapshot_interval = max(1, imu_steps_per_second // args.snapshots_per_second)

    for second in range(1, total_seconds + 1):
        print(f"\n[START OF SECOND {second}] --- High Frequency IMU Guessing Phase ---")
        
        # --- PHASE 1: IMU VELOCITY PROPAGATION (100 Micro-Steps) ---
        for micro_step in range(imu_steps_per_second):
            # Move the real-world trajectory forward
            T_true = np.dot(T_true, se3_exp(true_twist_velocity * dt_imu))
            
            # Simulate a slightly imperfect IMU reading (adding minor motion noise)
            # The robot reads this velocity and estimates its position blindly
            imu_reading = true_twist_velocity + np.random.normal(0, 0.02, 6)
            T_est = np.dot(T_est, se3_exp(imu_reading * dt_imu))
            
            # Print a few snippets of the fast intermediate tracking
            if micro_step % snapshot_interval == 0:
                print(f"  └─ IMU Step {micro_step:02d}: Est Pos = [{T_est[0,3]:.3f}, {T_est[1,3]:.3f}]")
        
        # --- PHASE 2: LOWER FREQUENCY POSITION FILTERING (1 Hz Update) ---
        print(f"\n[SENSOR TICK] Global Position Measurement Arrived!")
        print(f"  Pre-Correction Error Distance: {np.linalg.norm(T_true[0:3, 3] - T_est[0:3, 3]):.4f} meters")
        
        # Run local optimization cycles using the manifold formulas, iterating
        # until the correction step shrinks below gn_tol (or gn_max_iters is hit)
        for opt_iter in range(gn_max_iters):
            # Compute tracking residual matrix in the local tangent workspace
            T_error = np.dot(np.linalg.inv(T_true), T_est)
            e_vector = se3_log(T_error)

            # Evaluate analytical Jacobian mapping
            J = compute_se3_inv_right_jacobian(e_vector)

            # Solve normal system with our confidence profile
            H = np.dot(J.T, np.dot(omega_info, J)) + np.eye(6) * 1e-4
            g = -np.dot(J.T, np.dot(omega_info, e_vector))
            delta_xi = np.linalg.solve(H, g)

            # Correct the estimate matrix
            T_est = np.dot(T_est, se3_exp(delta_xi))

            step_norm = np.linalg.norm(delta_xi)
            print(f"    ├─ GN iter {opt_iter + 1}: |delta_xi| = {step_norm:.8f}")
            if step_norm < gn_tol:
                print(f"    └─ Converged after {opt_iter + 1} iteration(s) (|delta_xi| < {gn_tol})")
                break
        else:
            print(f"    └─ Reached max iterations ({gn_max_iters}) without full convergence")

        print(f"  Post-Correction Status: True X,Y = [{T_true[0,3]:.3f}, {T_true[1,3]:.3f}] | Est X,Y = [{T_est[0,3]:.3f}, {T_est[1,3]:.3f}]")
        print(f"  Final Post-Correction Error Distance: {np.linalg.norm(T_true[0:3, 3] - T_est[0:3, 3]):.4f} meters")
        print("-" * 90)

    print("\nTracking Loop Concluded.")