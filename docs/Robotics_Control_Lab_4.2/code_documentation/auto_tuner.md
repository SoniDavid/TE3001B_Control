# auto_tuner.py

**Package**: `xarm_perturbations`
**Node name**: `grid_search_tuner`
**Entry point**: `auto_tuner = xarm_perturbations.auto_tuner:main`

---

## Purpose

An automated **grid-search gain tuner** that:
1. Iterates over all combinations of `(Kp, Kd)` from configurable test lists
2. Pushes each combination to a running `trajectory_and_controller` node via the `SetParameters` ROS 2 service
3. Waits for `/cycle_metrics` to collect RMSE and saturation ratio
4. Scores each combination and saves results to a CSV
5. Reports the top-10 combinations when the search completes

**Requires**: `trajectory_and_controller` must be running and its parameter callback must be registered (it calls `/trajectory_and_controller/set_parameters`).

---

## Cost Function (Score)

```
score = avg_rmse_total + lambda_sat * avg_sat_ratio
```

Lower score is better. The `lambda_sat` weight (default 0.10) penalizes excessive velocity saturation, which indicates the gains are too aggressive.

---

## Class: `GridSearchTuner(Node)`

### Constructor

1. Declares all parameters (see table below)
2. Builds the full Cartesian product of `kp_values x kd_values` using `itertools.product`
3. Creates the `SetParameters` service client for the controller node
4. Subscribes to `/cycle_metrics` for feedback
5. Opens the results CSV file
6. Calls `set_next_parameters()` to start the first test immediately

---

### `set_next_parameters()`

Selects the next `(Kp, Kd)` pair from the combination list and:
1. Builds a `SetParameters.Request` with both gains as `DOUBLE_ARRAY` (isotropic: same value for X, Y, Z)
2. Sends the async RPC to the controller
3. Sets `awaiting_metrics = True` and initializes `cycles_to_ignore` (to skip the first transient cycle after a parameter change)
4. Resets all accumulators

---

### `_on_param_set_done(future)`

Callback for the async `SetParameters` service call. On success, the tuner simply waits for `/cycle_metrics` data. On failure, it skips to the next combination.

---

### `metrics_callback(msg)`

Receives `Float64MultiArray` from `/cycle_metrics` with 5 values:
`[rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio]`

**Transient skip**: The first `cycles_to_ignore` metrics after a parameter change are discarded to allow the controller to settle.

**Accumulation**: If `cycles_per_test > 1`, metrics are averaged over multiple cycles before scoring (reduces measurement noise).

Once enough cycles are collected, the score is computed, written to CSV, and `set_next_parameters()` is called for the next combination.

---

### `finish_search()`

Called when all combinations have been tested. Closes the CSV file, sorts results by score, and logs the top-10 to the console along with the best `(Kp, Kd)` pair.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `controller_node` | `/trajectory_and_controller` | Name of the controller node (used to build service name) |
| `lam_sat` | 0.10 | Weight for saturation penalty in score |
| `ignore_first_cycle` | `true` | Skip first cycle after parameter change |
| `cycles_per_test` | 1 | Number of cycles to average per (Kp, Kd) combo |
| `results_csv` | `data/tuning_results.csv` | Output CSV path |
| `kp_values` | [2.620, 2.635, 2.650, 2.665, 2.680] | Kp values to test |
| `kd_values` | [0.595, 0.610, 0.625, 0.640, 0.655] | Kd values to test |

Default grid: 5 x 5 = **25 combinations**.

---

## Results CSV Format

| Column | Description |
|---|---|
| `kp` | Proportional gain tested |
| `kd` | Derivative gain tested |
| `rmse_x` | Average RMSE in X |
| `rmse_y` | Average RMSE in Y |
| `rmse_z` | Average RMSE in Z |
| `rmse_total` | Average total 3D RMSE |
| `sat_ratio` | Average saturation ratio |
| `score` | `rmse_total + lam_sat * sat_ratio` |

---

## How to Use

### 1. Start the controller (in terminal 1)

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p radius:=0.15 -p frequency:=0.035 -p plane:=xy -p yaw_deg:=90.0 \
  -p max_speed:=0.12 -p deadband:=0.002
```

### 2. Start the tuner (in terminal 2)

```bash
ros2 run xarm_perturbations auto_tuner
```

### 3. With a custom search range

```bash
ros2 run xarm_perturbations auto_tuner --ros-args \
  -p kp_values:="[2.0, 2.5, 3.0]" \
  -p kd_values:="[0.3, 0.6, 0.9]" \
  -p cycles_per_test:=2 \
  -p lam_sat:=0.05
```

---

## Related Files

- [trajectory_and_controller.md](trajectory_and_controller.md) — the node being tuned (provides `/cycle_metrics` and `SetParameters` service)
- [analysis_evaluation.md](analysis_evaluation.md) — offline analysis of the best-gain run
