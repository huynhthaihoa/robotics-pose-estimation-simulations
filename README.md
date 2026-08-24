# Python Codebase — Robotics Pose Estimation Simulations

## Introduction

A collection of from-scratch simulations exploring pose/state estimation on
manifolds: how orientation and pose should be integrated and corrected on
**SO(3)**/**SE(3)** rather than treated as flat vectors, and how a **prior**
(a motion model driven by noisy control/odometry inputs) can be fused with
**measurements** either **recursively** (Kalman filtering) or in **batch**
(Gauss-Newton optimization).

Two implementation styles run side by side for the core comparison scripts,
split into sibling directories with matching filenames:
- [use_numpy/](use_numpy/): skew-symmetric matrices, `Exp`/`Log` maps, and
  Jacobians written out by hand (Rodrigues' formula, the SE(3)
  exponential/logarithm, the analytical inverse right Jacobian), shared
  across scripts via [use_numpy/lie_utils.py](use_numpy/lie_utils.py).
- [use_manif/](use_manif/): the same math delegated to the
  [`manif`](https://github.com/artivis/manif) Lie-theory library's Python
  bindings (`T.rplus`, `T.rminus`, `T.act`, all with analytical Jacobians
  returned as out-parameters), so no manifold formula is hand-rolled.

Each `use_manif/<name>.py` is the manifpy counterpart of `use_numpy/<name>.py`
of the same filename.

## Library structure

### [use_numpy/](use_numpy/) — plain numpy, hand-rolled Lie-group math

- [lie_utils.py](use_numpy/lie_utils.py): shared module of the hand-rolled
  Lie-group helpers (`skew`, `rotation_geodesic_error`, `so3_exp`,
  `so3_right_jacobian`, `se3_exp`, `se3_log`, `se3_inv`, `se3_adjoint`,
  `compute_so3_inv_right_jacobian`, `compute_se3_inv_right_jacobian`,
  `se3_right_jacobian`), imported by `imu_integration_comparison.py`,
  `imu_preintegration.py`, `pointcloud_pose_tracking.py`, and `pose_graph.py`.
  `robot_imu_simulation.py` still keeps its own inline copies of the
  skew/Jacobian helpers and hasn't been migrated to import from here yet.
- [imu_integration_comparison.py](use_numpy/imu_integration_comparison.py):
  naive Euler-angle vs. SO(3) exp-map orientation integration.
- [robot_imu_simulation.py](use_numpy/robot_imu_simulation.py): high-rate
  IMU propagation + a low-rate Gauss-Newton pose correction (à la a GPS fix),
  with the full SE(3) Exp/Log/inverse-right-Jacobian math written from
  scratch inline.
- [imu_preintegration.py](use_numpy/imu_preintegration.py): IMU
  pre-integration — compresses a burst of high-frequency IMU samples into
  one relative measurement plus first-order bias Jacobians, then shows an
  instant Taylor-expansion correction when the bias estimate changes,
  without re-integrating.
- [pointcloud_pose_tracking.py](use_numpy/pointcloud_pose_tracking.py):
  tracks a rigid object's pose from a motion-model prior (noisy control
  inputs) fused with noisy point-cloud measurements of its known geometry,
  comparing a recursive **EKF**, a recursive **invariant EKF** (body-frame
  residual, state-independent measurement Jacobian), and a **batch
  Gauss-Newton** smoother over the whole trajectory, reusing `lie_utils.py`'s
  SE(3) Exp/Log/inverse-right-Jacobian math plus a hand-rolled adjoint/
  right-Jacobian for the motion-model Jacobians. Also reports empirical
  per-step time and peak memory for each of the four approaches.
- [pose_graph.py](use_numpy/pose_graph.py): a small closed-loop 3D
  pose-graph relaxation (odometry drift + one loop closure), jointly
  optimized via Levenberg-Marquardt, reusing the same `lie_utils.py` SE(3)
  Exp/Log/inverse-right-Jacobian/adjoint math as `pointcloud_pose_tracking.py`.

### [use_manif/](use_manif/) — same simulations, on `manifpy`

- [imu_integration_comparison.py](use_manif/imu_integration_comparison.py)
- [robot_imu_simulation.py](use_manif/robot_imu_simulation.py)
- [imu_preintegration.py](use_manif/imu_preintegration.py): same bias-Jacobian
  preintegration bundle, with the SO(3) Exp map / right Jacobian / skew(hat)
  math delegated to manif's `rplus` Jacobian out-parameters and
  `SO3Tangent.hat()` instead of hand-rolled formulas.
- [pointcloud_pose_tracking.py](use_manif/pointcloud_pose_tracking.py): same
  four solvers (dead-reckoning, EKF, invariant EKF, batch Gauss-Newton) as the
  `use_numpy` version, with `motion_model`/`observation_model` built on
  `manifpy`'s `rplus`/`rminus`/`act` Jacobian out-parameters instead of
  `lie_utils.py`.
- [pose_graph.py](use_manif/pose_graph.py)

### Root

- [main.py](main.py): unused `uv init` placeholder entry point.
- [pyproject.toml](pyproject.toml): uv project file. `manifpy` is sourced
  from the local sibling checkout `../manif` (see Installation below).

## Installation

- Install [uv](https://docs.astral.sh/uv/).
- `manifpy` is not on PyPI here — this project's `pyproject.toml` points
  `uv` at a local sibling checkout (`../manif`), which must already have its
  Python bindings built (`pip3 install --user ../manif`, requires a working
  Eigen3 + CMake toolchain) before `uv sync` can resolve it. The
  [use_numpy/](use_numpy/) scripts don't need this — only the
  [use_manif/](use_manif/) scripts do.
- Then sync dependencies: `uv sync`
- Run any script with `uv run` from the `python-codebase` directory, e.g.:
  `uv run python use_numpy/imu_integration_comparison.py`

## Usage

### 1. Naive vs. exp-map IMU integration

`use_numpy/imu_integration_comparison.py` / `use_manif/imu_integration_comparison.py`

Integrates the same noisy gyro + body-velocity stream two ways — attitude
kept as a flat Euler-angle vector (`euler += omega*dt`) vs. attitude kept on
SO(3) and updated via the exponential map — to isolate the error the flat
vector-space approximation introduces on its own. Ground truth is the
exp-map integration of the noise-free rates. Plots rotation and position
error (log scale) over time.

```
uv run python use_numpy/imu_integration_comparison.py --duration 20.0 --dt 0.005 --gyro-noise-std 0.02 --vel-noise-std 0.05 --seed 0 --out out.png
```

- `--duration`: simulation length in seconds (default `20.0`)
- `--dt`: IMU sample interval in seconds (default `0.005`)
- `--gyro-noise-std`: gyro noise std-dev, rad/s (default `0.02`)
- `--vel-noise-std`: body-velocity noise std-dev, m/s (default `0.05`)
- `--seed`: RNG seed (default `0`)
- `--out`: save the figure to this path instead of showing it (default: show)

### 2. IMU propagation + Gauss-Newton position correction

`use_numpy/robot_imu_simulation.py` / `use_manif/robot_imu_simulation.py`

Simulates a robot with a 100 Hz IMU (noisy body twist) and a 1 Hz
high-accuracy global position fix (e.g. GPS). Each second: propagate the
pose estimate through 100 noisy IMU micro-steps on SE(3), then run a
Gauss-Newton correction against the position fix (an unbalanced information
matrix trusts position far more than orientation) until the correction step
norm drops below `--gn-tol` or `--gn-max-iters` is hit. Prints pre/post
correction error each second (no plot).

```
uv run python use_numpy/robot_imu_simulation.py --dt-imu 0.01 --total-seconds 3 --snapshots-per-second 4 --gn-tol 1e-6 --gn-max-iters 10 --max-linear-vel 1.0 --max-angular-vel 0.5
```

- `--dt-imu`: IMU update interval in seconds (default `0.01`, i.e. 100 Hz)
- `--total-seconds`: total simulation duration in seconds (default `3`)
- `--snapshots-per-second`: number of intermediate IMU steps logged per second (default `4`)
- `--gn-tol`: Gauss-Newton convergence tolerance (default `1e-6`)
- `--gn-max-iters`: maximum Gauss-Newton iterations (default `10`)
- `--max-linear-vel`: max linear velocity magnitude for the random true twist, m/s (default `1.0`)
- `--max-angular-vel`: max angular velocity magnitude for the random true twist, rad/s (default `0.5`)

### 3. IMU pre-integration with bias Jacobians

`use_numpy/imu_preintegration.py` / `use_manif/imu_preintegration.py`

Streams 1 second of high-frequency IMU samples into a single
`PreintegratedIMUBundle` (compressed relative rotation/velocity/position,
plus their Jacobians wrt gyro/accel bias). Then simulates a graph-SLAM-style
bias update and applies it to the bundle via a first-order Taylor correction
— instant, versus re-running the whole integration loop.

```
uv run python use_numpy/imu_preintegration.py --frequency-hz 100 --gyro-bias 0.01 -0.01 0.02 --accel-bias 0.05 0.00 -0.05 --linear-vel 1.0 0.1 0.0 --angular-vel 0.0 0.0 0.5 --optimized-gyro-bias 0.008 -0.009 0.018 --optimized-accel-bias 0.045 0.002 -0.048
```

- `--frequency-hz`: IMU sampling frequency integrated over 1 simulated second (default `100`)
- `--gyro-bias`, `--accel-bias`: initial estimated sensor biases (default `0.01 -0.01 0.02` rad/s, `0.05 0.00 -0.05` m/s²)
- `--linear-vel`, `--angular-vel`: true commanded body-frame twist (default `1.0 0.1 0.0` m/s, `0.0 0.0 0.5` rad/s)
- `--optimized-gyro-bias`, `--optimized-accel-bias`: post-optimization corrected biases to apply via the Taylor correction

### 4. Point-cloud pose tracking: EKF vs. invariant EKF vs. batch Gauss-Newton

`use_numpy/pointcloud_pose_tracking.py` / `use_manif/pointcloud_pose_tracking.py`

Tracks a rigid object's SE(3) pose from a combination of a motion-model
prior (noisy control-input twist) and noisy point-cloud measurements of the
object's known body-frame geometry (`z_i = T.act(p_i) + noise`). Two shared,
documented functions — `motion_model` and `observation_model`, both with
analytical Jacobians (hand-rolled SE(3) Exp/Log/adjoint math via
`lie_utils.py` in the `use_numpy` version; `manifpy`'s
`rplus`/`rminus`/`act` out-parameters in the `use_manif` version) — feed
four solvers: a prior-only dead-reckoning baseline, a recursive **EKF**
(constant-size state, online), a recursive **invariant EKF** (same predict
step, but the update expresses the residual in the estimate's body frame,
making the measurement Jacobian state-independent instead of re-linearized
around the current rotation every step; for this problem's isotropic
point-noise model this is provably equivalent to the plain EKF's corrections
at every step, so the practical win here is a fixed, precomputed Jacobian
rather than different accuracy), and a **batch
Gauss-Newton** smoother that jointly optimizes the whole trajectory at once
against prior/motion/measurement factors. Prints final/RMS rotation+position
error per method, plus each method's empirical average per-step wall-clock
time and peak memory (`measure_performance`, via `time.perf_counter` +
`tracemalloc`), and plots rotation error, position error, and the x-y
trajectory of all five (ground truth included).

```
uv run python use_numpy/pointcloud_pose_tracking.py --duration 5.0 --dt 0.1 --n-points 20 --vel-noise-std 0.05 --gyro-noise-std 0.02 --point-noise-std 0.03 --init-pose-noise-std 0.1 --gn-tol 1e-6 --gn-max-iters 20 --seed 0 --out out.png
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

### 5. 3D pose-graph relaxation

`use_numpy/pose_graph.py` / `use_manif/pose_graph.py`

A robot drives a closed 4-node square loop, accumulating drift from noisy
relative-pose ("odometry") edges between consecutive nodes, then detects it
has returned to the start and adds one loop-closure edge back to node 0. All
node poses are jointly refined by Levenberg-Marquardt against every edge's
residual `e_ij = Log(Z_ij^-1 * X_i^-1 * X_j)`, using analytical
`compose`/`rminus` Jacobians chained together (hand-rolled SE(3) Exp/Log/
adjoint math via `lie_utils.py` in the `use_numpy` version; `manifpy`'s
out-parameters in the `use_manif` version) — the same motion-factor
Jacobian-chaining pattern as `run_batch_gn` in `pointcloud_pose_tracking.py`,
generalized from a twist-based motion model to a directly-measured relative
pose. Prints per-iteration chi-squared error plus final/RMS rotation+position
error (uncorrected odometry vs. optimized), and plots the XY trajectory
against ground truth.

```
uv run python use_numpy/pose_graph.py --side-length 2.0 --pos-noise-std 0.05 --rot-noise-std 0.01 --loop-noise-scale 0.5 --damping 0.01 --gn-tol 1e-6 --gn-max-iters 10 --seed 0 --out out.png
```

- `--side-length`: side length of the square ground-truth loop, m (default `2.0`)
- `--pos-noise-std`: odometry-edge translation noise std-dev, m (default `0.05`)
- `--rot-noise-std`: odometry-edge rotation noise std-dev, rad (default `0.01`)
- `--loop-noise-scale`: noise std-dev multiplier for the loop-closure edge (default `0.5`)
- `--damping`: Levenberg-Marquardt damping factor (default `0.01`)
- `--gn-tol`: convergence tolerance on the correction step norm (default `1e-6`)
- `--gn-max-iters`: maximum optimization iterations (default `10`)
- `--seed`: RNG seed (default `0`)
- `--out`: save the figure to this path instead of showing it (default: show)
