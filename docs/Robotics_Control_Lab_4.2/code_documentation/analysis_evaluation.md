# analysis_evaluation.py

**Package**: `xarm_perturbations`
**Type**: Standalone Python script (not a ROS 2 node)
**Run**: `python3 analysis_evaluation.py` (from the directory containing the CSV file)

---

## Purpose

Offline analysis script for **multiple experiments stored in a single large CSV file**. The CSV is assumed to contain a continuous recording with all three experimental conditions (Baseline, Sinusoidal, Gaussian) recorded back-to-back, distinguished by time ranges.

For each defined time segment it computes:
- RMSE per axis (X, Y, Z) and total 3D RMSE
- Maximum absolute error per axis and global max
- Three plots per segment

---

## How It Works

### 1. Load CSV

```python
df_full = pd.read_csv("baseline.csv")
```

The filename is hardcoded at the top of `main()` — edit it before running.

### 2. Compute Derived Columns

```python
df_full['e_x'] = df_full['x_des'] - df_full['x_act']
df_full['e_y'] = df_full['y_des'] - df_full['y_act']
df_full['e_z'] = df_full['z_des'] - df_full['z_act']
df_full['v_mag'] = sqrt(vx_cmd^2 + vy_cmd^2 + vz_cmd^2)
```

### 3. Time Segments

```python
segmentos = {
    "Baseline":  (0,   75),
    "Sine":      (75,  225),
    "Gaussian":  (260, max_time)
}
```

These time boundaries are hardcoded and must match your actual recording. Edit `segmentos` to match your data.

### 4. Per-Segment Analysis

For each segment the script:
1. Slices `df_full` by time range
2. Computes RMSE and max absolute error (printed to console)
3. Generates a figure with 3 subplots
4. Saves the figure as `resultados_<experiment_name>.png`
5. Calls `plt.show()` — **the script pauses at each figure until closed**

---

## Plots Generated

Each segment produces one figure (`resultados_baseline.png`, `resultados_sine.png`, `resultados_gaussian.png`):

### Plot 1 — Trajectory in XY Plane
`x_des vs y_des` (dashed black) overlaid with `x_act vs y_act` (green). Equal-aspect ratio to avoid visual distortion of the figure-eight.

### Plot 2 — Position Error Over Time
`e_x`, `e_y`, `e_z` vs relative time (t - t_start). Useful for identifying which axis is most affected by perturbations.

### Plot 3 — Commanded Velocity Magnitude
`v_mag = ||[vx_cmd, vy_cmd, vz_cmd]||` vs relative time. Shows when the velocity saturator was active (flat regions near `max_speed = 0.12 m/s`).

---

## Console Output Example

```
==================================================
PROCESANDO EXPERIMENTO: BASELINE (De 0s a 75s)
==================================================
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

## Expected CSV Columns

The script expects the exact column names produced by `trajectory_and_controller.py`:

`time, x_des, y_des, z_des, x_act, y_act, z_act, ex, ey, ez, vx_cmd, vy_cmd, vz_cmd, v_norm, is_saturated`

---

## Customization

- **Change CSV file**: edit `csv_filename = "baseline.csv"` at the top of `main()`
- **Change time segments**: edit the `segmentos` dict
- **Add more experiments**: add more entries to `segmentos`
- **Save without showing**: replace `plt.show()` with `plt.close()` for batch processing

---

## Related Files

- [analysis_evaluation_only.md](analysis_evaluation_only.md) — single-experiment version (simpler)
- [trajectory_and_controller.md](trajectory_and_controller.md) — produces the CSV files analyzed here
