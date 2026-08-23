# Python Codebase — Robotics Pose Estimation Simulations

## Introduction

A collection of from-scratch simulations exploring pose/state estimation on
manifolds: how orientation and pose should be integrated and corrected on
**SO(3)**/**SE(3)** rather than treated as flat vectors, and how a **prior**
(a motion model driven by noisy control/odometry inputs) can be fused with
**measurements** either **recursively** (Kalman filtering) or in **batch**
(Gauss-Newton optimization).

Two implementation styles run side by side for the core comparison scripts:
- **Plain numpy**: skew-symmetric matrices, `Exp`/`Log` maps, and Jacobians
  written out by hand (Rodrigues' formula, the SE(3) exponential/logarithm,
  the analytical inverse right Jacobian).
- **manifpy**: the same math delegated to the
  [`manif`](https://github.com/artivis/manif) Lie-theory library's Python
  bindings (`T.rplus`, `T.rminus`, `T.act`, all with analytical Jacobians
  returned as out-parameters), so no manifold formula is hand-rolled.

## Library structure

- [imu_integration_comparison.py](imu_integration_comparison.py): naive
  Euler-angle vs. SO(3) exp-map orientation integration — plain numpy.
- [imu_integration_comparison_manif.py](imu_integration_comparison_manif.py):
  the same comparison, fully on `manifpy`'s SE(3)/SO(3) ops.
- [robot_imu_simulation.py](robot_imu_simulation.py): high-rate IMU
  propagation + a low-rate Gauss-Newton pose correction (à la a GPS fix) —
  plain numpy, with the full SE(3) Exp/Log/inverse-right-Jacobian math
  written from scratch.
- [robot_imu_simulation_manif.py](robot_imu_simulation_manif.py): the same
  simulation, on `manifpy`.
- [simulation2.py](simulation2.py): IMU pre-integration — compresses a burst
  of high-frequency IMU samples into one relative measurement plus
  first-order bias Jacobians, then shows an instant Taylor-expansion
  correction when the bias estimate changes, without re-integrating.
- [pointcloud_pose_tracking.py](pointcloud_pose_tracking.py): tracks a rigid
  object's pose from a motion-model prior (noisy control inputs) fused with
  noisy point-cloud measurements of its known geometry, comparing a
  recursive **EKF** against a **batch Gauss-Newton** smoother over the whole
  trajectory — on `manifpy`.
- [main.py](main.py): unused `uv init` placeholder entry point.
- [pyproject.toml](pyproject.toml): uv project file. `manifpy` is sourced
  from the local sibling checkout `../manif` (see Installation below).

## Installation

- Install [uv](https://docs.astral.sh/uv/).
- `manifpy` is not on PyPI here — this project's `pyproject.toml` points
  `uv` at a local sibling checkout (`../manif`), which must already have its
  Python bindings built (`pip3 install --user ../manif`, requires a working
  Eigen3 + CMake toolchain) before `uv sync` can resolve it. The
  plain-numpy scripts (`imu_integration_comparison.py`,
  `robot_imu_simulation.py`, `simulation2.py`) don't need this.
- Then sync dependencies: `uv sync`
- Run any script with `uv run`, e.g.: `uv run python imu_integration_comparison.py`

> **Note:** regarding `manifpy`, please refer to [manif (A small C++11 header-only library for Lie theory)](https://github.com/artivis/manif)

## Usage

### 1. Naive vs. exp-map IMU integration

`imu_integration_comparison.py` / `imu_integration_comparison_manif.py`

Integrates the same noisy gyro + body-velocity stream two ways — attitude
kept as a flat Euler-angle vector (`euler += omega*dt`) vs. attitude kept on
SO(3) and updated via the exponential map — to isolate the error the flat
vector-space approximation introduces on its own. Ground truth is the
exp-map integration of the noise-free rates. Plots rotation and position
error (log scale) over time.

```
uv run python imu_integration_comparison.py --duration 20.0 --dt 0.005 --gyro-noise-std 0.02 --vel-noise-std 0.05 --seed 0 --out out.png
```

- `--duration`: simulation length in seconds (default `20.0`)
- `--dt`: IMU sample interval in seconds (default `0.005`)
- `--gyro-noise-std`: gyro noise std-dev, rad/s (default `0.02`)
- `--vel-noise-std`: body-velocity noise std-dev, m/s (default `0.05`)
- `--seed`: RNG seed (default `0`)
- `--out`: save the figure to this path instead of showing it (default: show)

### 2. IMU propagation + Gauss-Newton position correction

`robot_imu_simulation.py` / `robot_imu_simulation_manif.py`

Simulates a robot with a 100 Hz IMU (noisy body twist) and a 1 Hz
high-accuracy global position fix (e.g. GPS). Each second: propagate the
pose estimate through 100 noisy IMU micro-steps on SE(3), then run a
Gauss-Newton correction against the position fix (an unbalanced information
matrix trusts position far more than orientation) until the correction step
norm drops below `--gn-tol` or `--gn-max-iters` is hit. Prints pre/post
correction error each second (no plot).

```
uv run python robot_imu_simulation.py --dt-imu 0.01 --total-seconds 3 --snapshots-per-second 4 --gn-tol 1e-6 --gn-max-iters 10 --max-linear-vel 1.0 --max-angular-vel 0.5
```

- `--dt-imu`: IMU update interval in seconds (default `0.01`, i.e. 100 Hz)
- `--total-seconds`: total simulation duration in seconds (default `3`)
- `--snapshots-per-second`: number of intermediate IMU steps logged per second (default `4`)
- `--gn-tol`: Gauss-Newton convergence tolerance (default `1e-6`)
- `--gn-max-iters`: maximum Gauss-Newton iterations (default `10`)
- `--max-linear-vel`: max linear velocity magnitude for the random true twist, m/s (default `1.0`)
- `--max-angular-vel`: max angular velocity magnitude for the random true twist, rad/s (default `0.5`)

### 3. IMU pre-integration with bias Jacobians

`simulation2.py`

Streams 1 second of high-frequency IMU samples into a single
`PreintegratedIMUBundle` (compressed relative rotation/velocity/position,
plus their Jacobians wrt gyro/accel bias). Then simulates a graph-SLAM-style
bias update and applies it to the bundle via a first-order Taylor correction
— instant, versus re-running the whole integration loop.

```
uv run python simulation2.py --frequency-hz 100 --gyro-bias 0.01 -0.01 0.02 --accel-bias 0.05 0.00 -0.05 --linear-vel 1.0 0.1 0.0 --angular-vel 0.0 0.0 0.5 --optimized-gyro-bias 0.008 -0.009 0.018 --optimized-accel-bias 0.045 0.002 -0.048
```

- `--frequency-hz`: IMU sampling frequency integrated over 1 simulated second (default `100`)
- `--gyro-bias`, `--accel-bias`: initial estimated sensor biases (default `0.01 -0.01 0.02` rad/s, `0.05 0.00 -0.05` m/s²)
- `--linear-vel`, `--angular-vel`: true commanded body-frame twist (default `1.0 0.1 0.0` m/s, `0.0 0.0 0.5` rad/s)
- `--optimized-gyro-bias`, `--optimized-accel-bias`: post-optimization corrected biases to apply via the Taylor correction

### 4. Point-cloud pose tracking: EKF vs. batch Gauss-Newton

`pointcloud_pose_tracking.py`

Tracks a rigid object's SE(3) pose from a combination of a motion-model
prior (noisy control-input twist) and noisy point-cloud measurements of the
object's known body-frame geometry (`z_i = T.act(p_i) + noise`). Two shared,
documented functions — `motion_model` and `observation_model`, both built on
`manifpy`'s analytical Jacobians — feed three solvers: a prior-only
dead-reckoning baseline, a recursive **EKF** (constant-size state, online),
and a **batch Gauss-Newton** smoother that jointly optimizes the whole
trajectory at once against prior/motion/measurement factors. Prints
final/RMS rotation+position error per method and plots rotation error,
position error, and the x-y trajectory of all four (ground truth included).

```
uv run python pointcloud_pose_tracking.py --duration 5.0 --dt 0.1 --n-points 20 --vel-noise-std 0.05 --gyro-noise-std 0.02 --point-noise-std 0.03 --init-pose-noise-std 0.1 --gn-tol 1e-6 --gn-max-iters 20 --seed 0 --out out.png
```

- `--duration`: simulation length in seconds (default `5.0`)
- `--dt`: motion/measurement step interval in seconds (default `0.1`)
- `--n-points`: number of body-frame point-cloud landmarks (default `20`)
- `--vel-noise-std`: input linear-velocity noise std-dev, m/s (default `0.05`)
- `--gyro-noise-std`: input angular-velocity noise std-dev, rad/s (default `0.02`)
- `--point-noise-std`: point-cloud measurement noise std-dev, m (default `0.03`)
- `--init-pose-noise-std`: std-dev used to perturb the initial pose guess, and to set the prior/EKF-init covariance (default `0.1`)
- `--gn-tol`: batch Gauss-Newton convergence tolerance (default `1e-6`)
- `--gn-max-iters`: maximum batch Gauss-Newton iterations (default `20`)
- `--seed`: RNG seed (default `0`)
- `--out`: save the figure to this path instead of showing it (default: show)
