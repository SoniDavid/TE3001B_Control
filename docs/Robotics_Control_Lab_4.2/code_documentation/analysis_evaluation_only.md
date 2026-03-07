# analysis_evaluation_only.py

**Package**: `xarm_perturbations`
**Type**: Standalone Python script (not a ROS 2 node)
**Run**: `python3 analysis_evaluation_only.py` (from the directory containing the CSV file)

---

## Purpose

Simpler offline analysis script for **a single experiment CSV file**. Computes RMSE and max absolute error for the entire file as one dataset (no time segmentation).

Use this script when each experiment is recorded as its own separate CSV. Use `analysis_evaluation.py` when all experiments are combined in one large CSV with different time ranges.

---

## How It Works

### 1. Load CSV

```python
df = pd.read_csv("baseline_2.csv")
```

Filename is hardcoded — edit before running.

### 2. Compute Error Columns

```python
df['e_x'] = df['x_des'] - df['x_act']
df['e_y'] = df['y_des'] - df['y_act']
df['e_z'] = df['z_des'] - df['z_act']
df['v_mag'] = sqrt(vx_cmd^2 + vy_cmd^2 + vz_cmd^2)
```

### 3. Compute Metrics

RMSE per axis and total RMSE:
```python
rmse_x = sqrt(mean(e_x^2))
rmse_y = sqrt(mean(e_y^2))
rmse_z = sqrt(mean(e_z^2))
rmse_total = sqrt(rmse_x^2 + rmse_y^2 + rmse_z^2)
```

Maximum absolute error per axis:
```python
max_err_x = max(|e_x|)
```

### 4. Generate Plots

Produces a single 2x2 figure with:
- **Plot 1** (top-left): XY trajectory — desired vs actual
- **Plot 2** (top-right): Position error vs time (e_x, e_y, e_z)
- **Plot 3** (bottom, full width): Commanded velocity magnitude vs time

Saves as `evaluacion_<csv_stem>.png` (e.g. `evaluacion_baseline_2.png`) and displays it.

---

## Console Output

```
RESULTADOS DE EVALUACION
RMSE X: 0.01234 m
RMSE Y: 0.00987 m
RMSE Z: 0.00012 m
Total RMSE: 0.01583 m
------------------------------------------
Max Error Absoluto X: 0.03210 m
Max Error Absoluto Y: 0.02876 m
Max Error Absoluto Z: 0.00045 m
Max Error Absoluto Global: 0.03210 m
```

---

## Differences from `analysis_evaluation.py`

| Feature | `analysis_evaluation_only.py` | `analysis_evaluation.py` |
|---|---|---|
| Number of experiments | 1 (entire CSV) | Multiple (time-segmented) |
| Time segmentation | No | Yes (hardcoded `segmentos` dict) |
| Loop over segments | No | Yes |
| Output images | 1 file | 1 file per experiment |
| Use case | Separate CSV per experiment | All experiments in one CSV |

---

## Related Files

- [analysis_evaluation.md](analysis_evaluation.md) — multi-experiment version
- [trajectory_and_controller.md](trajectory_and_controller.md) — produces the CSV files analyzed here
