#!/usr/bin/env python3
"""
Step-response analysis for Challenge 4.1.

Usage:
  python step_analysis.py <csv_file>              # single trial
  python step_analysis.py <csv1> <csv2>           # comparison (e.g. CTC vs PD)
  python step_analysis.py <data_dir>              # auto-discover step CSVs

Generates:
  1. step_response_<label>.png      — position on step axis vs desired
  2. step_error_<label>.png         — tracking error on step axis over time
  3. step_comparison.png            — overlay of all loaded trials (error + position)
  4. step_metrics_<label>.png       — table: rise time, settling time, overshoot, SS error
  5. step_metrics.csv               — machine-readable metrics

Step metrics computed (per trial):
  - Rise time        (10 % → 90 % of step amplitude, on actual EE position)
  - Settling time    (last time |e_axis| > 2 % of step amplitude)
  - Overshoot        (max overshoot beyond the step target, as % of amplitude)
  - Steady-state err (mean |e_axis| over last 1 s of hold_after window)
  - RMSE (step axis) over hold_after window
"""

import sys
import os
import glob
import json
import csv as csv_mod

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_AXIS_COL = {'x': ('p_act_x', 'p_des_x'),
              'y': ('p_act_y', 'p_des_y'),
              'z': ('p_act_z', 'p_des_z')}


def _label_from_path(path: str) -> str:
    base = os.path.splitext(os.path.basename(path))[0]
    # shorten long auto-generated names
    if 'ctc' in base and 'nopert' in base:
        return 'CTC'
    if 'pdpid' in base and 'nopert' in base:
        return 'PD/PID'
    if 'ctc' in base:
        return 'CTC'
    if 'pdpid' in base:
        return 'PD/PID'
    return base[:30]


def _load_meta(csv_path: str) -> dict:
    base = os.path.splitext(csv_path)[0]
    candidates = [base + '_metadata.json'] + sorted(
        glob.glob(os.path.join(os.path.dirname(csv_path),
                               os.path.basename(base)[:-2] + '*_metadata.json')))
    for p in candidates:
        if os.path.exists(p):
            with open(p) as f:
                return json.load(f)
    return {}


def _discover_step_csvs(directory: str):
    """Return list of step-trial CSV paths found in directory."""
    all_csv = sorted(glob.glob(os.path.join(directory, 'trial_*.csv')))
    # Prefer files whose companion metadata declares trajectory_mode=step.
    step_files = []
    other_files = []
    for f in all_csv:
        meta = _load_meta(f)
        if meta.get('trajectory_mode') == 'step':
            step_files.append(f)
        else:
            other_files.append(f)
    return step_files if step_files else all_csv  # fall back to all if none labelled


def _detect_axis(df: pd.DataFrame, meta: dict) -> str:
    """Detect which axis was stepped from metadata or by largest range."""
    if meta:
        tp = meta.get('trajectory_params', {})
        ax = tp.get('step_axis', '')
        if ax in ('x', 'y', 'z'):
            return ax
    # Fallback: axis with largest range in p_des
    ranges = {
        'x': df['p_des_x'].max() - df['p_des_x'].min(),
        'y': df['p_des_y'].max() - df['p_des_y'].min(),
        'z': df['p_des_z'].max() - df['p_des_z'].min(),
    }
    return max(ranges, key=ranges.get)


def _step_amplitude(df: pd.DataFrame, axis: str) -> float:
    """Amplitude of the step = range of p_des on step axis."""
    col_des = _AXIS_COL[axis][1]
    return float(df[col_des].max() - df[col_des].min())


def _hold_before_after(df: pd.DataFrame, axis: str = None, meta: dict = None) -> tuple:
    """
    Return (t_step, t_end) where t_step is the estimated step onset
    and t_end is the last time_rel.

    Priority for t_step:
      1) metadata trajectory_params.step_hold_before_s (if available)
      2) first change on desired position of detected step axis
    """
    t = df['time_rel'].values
    t_end = float(t[-1])

    if meta:
        tp = meta.get('trajectory_params', {})
        t_meta = tp.get('step_hold_before_s', None)
        if t_meta is not None:
            try:
                t_step = float(t_meta)
                # Clamp to available time range for robustness.
                t_step = float(np.clip(t_step, t[0], t_end))
                return t_step, t_end
            except (TypeError, ValueError):
                pass

    axis = axis if axis in ('x', 'y', 'z') else _detect_axis(df, meta or {})
    col_des = _AXIS_COL[axis][1]
    p_des = df[col_des].values
    p0 = p_des[0]
    changed = np.where(np.abs(p_des - p0) > 1e-6)[0]
    t_step = float(t[changed[0]]) if len(changed) > 0 else 0.0
    t_end = float(t[-1])
    return t_step, t_end


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_step_metrics(df: pd.DataFrame, axis: str, meta: dict = None) -> dict:
    """
    Compute standard step-response metrics for the given axis.

    Returns dict with keys:
        axis, amplitude_mm, rise_time_s, settling_time_s,
        overshoot_pct, ss_error_mm, rmse_mm
    """
    col_act, col_des = _AXIS_COL[axis]
    t = df['time_rel'].values
    p_act = df[col_act].values
    p_des = df[col_des].values
    e = p_des - p_act   # tracking error (positive = lagging)

    amp = _step_amplitude(df, axis)
    t_step, t_end = _hold_before_after(df, axis=axis, meta=meta)

    metrics = {'axis': axis, 'amplitude_mm': amp * 1000}

    # Only analyse the post-step window
    post_mask = t >= t_step
    if post_mask.sum() < 2 or amp < 1e-6:
        metrics.update({'rise_time_s': float('nan'),
                        'settling_time_s': float('nan'),
                        'overshoot_pct': float('nan'),
                        'ss_error_mm': float('nan'),
                        'rmse_mm': float('nan')})
        return metrics

    t_post = t[post_mask]
    p_act_post = p_act[post_mask]
    p_des_post = p_des[post_mask]
    e_post = e[post_mask]

    p_initial = p_des[~post_mask][-1] if (~post_mask).any() else p_des[0]
    p_target = p_des_post[-1]
    direction = np.sign(p_target - p_initial)

    # Rise time: 10 % → 90 % of amplitude
    p10 = p_initial + 0.10 * direction * amp
    p90 = p_initial + 0.90 * direction * amp
    cross10 = np.where(direction * (p_act_post - p10) >= 0)[0]
    cross90 = np.where(direction * (p_act_post - p90) >= 0)[0]
    t_10 = float(t_post[cross10[0]]) if len(cross10) else float('nan')
    t_90 = float(t_post[cross90[0]]) if len(cross90) else float('nan')
    rise_time = t_90 - t_10 if (not np.isnan(t_10) and not np.isnan(t_90)) else float('nan')

    # Settling time: last time |e_axis| > 2 % of amplitude
    band = 0.02 * amp
    outside = np.where(np.abs(e_post) > band)[0]
    if len(outside) > 0:
        settling_time = float(t_post[outside[-1]]) - t_step
    else:
        settling_time = 0.0   # already settled at step onset

    # Overshoot: max exceedance beyond target in step direction
    overshoot_abs = float(np.max(direction * (p_act_post - p_target)))
    overshoot_pct = (overshoot_abs / amp * 100.0) if amp > 0 else 0.0
    overshoot_pct = max(0.0, overshoot_pct)

    # Steady-state error: mean |e| over last 1 s (or last 20 % if < 1 s)
    window = max(1.0, (t_end - t_step) * 0.2)
    ss_mask = t_post >= (t_end - window)
    ss_error = float(np.mean(np.abs(e_post[ss_mask]))) if ss_mask.any() else float('nan')

    # RMSE over entire post-step window
    rmse = float(np.sqrt(np.mean(e_post ** 2)))

    metrics.update({
        'rise_time_s': round(rise_time, 4) if not np.isnan(rise_time) else float('nan'),
        'settling_time_s': round(settling_time, 4),
        'overshoot_pct': round(overshoot_pct, 2),
        'ss_error_mm': round(ss_error * 1000, 4),
        'rmse_mm': round(rmse * 1000, 4),
    })
    return metrics


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def plot_step_response(df: pd.DataFrame, axis: str, label: str,
                       metrics: dict, save_dir: str, meta: dict = None):
    """Position on step axis vs time, with step onset and settling band."""
    col_act, col_des = _AXIS_COL[axis]
    t = df['time_rel'].values
    p_act = df[col_act].values * 1000   # mm
    p_des = df[col_des].values * 1000

    t_step, _ = _hold_before_after(df, axis=axis, meta=meta)
    amp_mm = metrics['amplitude_mm']

    plt.style.use('bmh')
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, p_des, 'k--', lw=1.8, label='Reference (p_des)')
    ax.plot(t, p_act, color='steelblue', lw=1.2, label='Actual (p_act)')

    # 2 % settling band
    p_target = p_des[-1]
    band = 0.02 * amp_mm
    ax.axhline(p_target + band, color='gray', ls=':', lw=1, label='±2 % band')
    ax.axhline(p_target - band, color='gray', ls=':', lw=1)
    ax.axvline(t_step, color='orange', ls='--', lw=1.2, label=f'Step onset t={t_step:.1f}s')

    # Annotate metrics
    info = (f"Rise time:     {metrics['rise_time_s']:.3f} s\n"
            f"Settling time: {metrics['settling_time_s']:.3f} s\n"
            f"Overshoot:     {metrics['overshoot_pct']:.2f} %\n"
            f"SS error:      {metrics['ss_error_mm']:.3f} mm\n"
            f"RMSE:          {metrics['rmse_mm']:.3f} mm")
    ax.text(0.02, 0.05, info, transform=ax.transAxes, fontsize=9,
            va='bottom', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    ax.set_title(f'Step Response ({axis.upper()} axis, Δ={amp_mm:.1f} mm) — {label}',
                 fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(f'Position {axis.upper()} (mm)')
    ax.legend(fontsize=8)
    ax.grid(True)
    plt.tight_layout()

    fname = f'step_response_{label.replace("/", "_").replace(" ", "_")}.png'
    fpath = os.path.join(save_dir, fname)
    fig.savefig(fpath, dpi=130)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_step_error(df: pd.DataFrame, axis: str, label: str,
                    metrics: dict, save_dir: str, meta: dict = None):
    """Tracking error on step axis over time."""
    col_act, col_des = _AXIS_COL[axis]
    t = df['time_rel'].values
    e_mm = (df[col_des].values - df[col_act].values) * 1000

    t_step, _ = _hold_before_after(df, axis=axis, meta=meta)
    amp_mm = metrics['amplitude_mm']
    band = 0.02 * amp_mm

    plt.style.use('bmh')
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, e_mm, color='crimson', lw=1.0, label=f'Error e_{axis} = p_des − p_act')
    ax.axhline(band, color='gray', ls=':', lw=1, label='±2 % band')
    ax.axhline(-band, color='gray', ls=':', lw=1)
    ax.axhline(0, color='k', lw=0.6)
    ax.axvline(t_step, color='orange', ls='--', lw=1.2, label=f'Step onset')

    # shade settling region
    settle_end = t_step + metrics['settling_time_s']
    ax.axvspan(t_step, settle_end, alpha=0.10, color='orange', label='Settling region')

    ax.set_title(f'Tracking Error ({axis.upper()} axis) — {label}', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (mm)')
    ax.legend(fontsize=8)
    ax.grid(True)
    plt.tight_layout()

    fname = f'step_error_{label.replace("/", "_").replace(" ", "_")}.png'
    fpath = os.path.join(save_dir, fname)
    fig.savefig(fpath, dpi=130)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_comparison(trials: list, save_dir: str):
    """
    Overlay all trials on two panels:
      Left:  position on step axis (normalised to 0→1 for different amplitudes)
      Right: tracking error (mm)
    """
    if len(trials) < 2:
        return

    colors = ['steelblue', 'crimson', 'forestgreen', 'darkorange', 'purple']
    plt.style.use('bmh')
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle('Step-Response Comparison', fontsize=13, fontweight='bold')

    for i, (df, axis, label, metrics, meta) in enumerate(trials):
        col_act, col_des = _AXIS_COL[axis]
        t = df['time_rel'].values
        p_act = df[col_act].values * 1000
        p_des = df[col_des].values * 1000
        e_mm = p_des - p_act
        color = colors[i % len(colors)]
        t_step, _ = _hold_before_after(df, axis=axis, meta=meta)

        # Normalise position: 0 = pre-step, 1 = step target
        p0 = p_des[0]
        p1 = p_des[-1]
        span = p1 - p0 if abs(p1 - p0) > 0.1 else 1.0
        p_norm = (p_act - p0) / span
        d_norm = (p_des - p0) / span

        axes[0].plot(t - t_step, p_norm, color=color, lw=1.2, label=label)
        axes[1].plot(t - t_step, e_mm, color=color, lw=1.2, label=label)

    # Reference line
    axes[0].axhline(1.0, color='k', ls='--', lw=1, label='Reference')
    axes[0].axhline(0.98, color='gray', ls=':', lw=0.8)
    axes[0].axhline(1.02, color='gray', ls=':', lw=0.8, label='±2 % band')
    axes[1].axhline(0, color='k', lw=0.6)

    axes[0].set_title('Normalised Step Response')
    axes[0].set_xlabel('Time from step onset (s)')
    axes[0].set_ylabel('Normalised position')
    axes[0].legend(fontsize=9)
    axes[0].grid(True)

    axes[1].set_title('Tracking Error on Step Axis')
    axes[1].set_xlabel('Time from step onset (s)')
    axes[1].set_ylabel('Error (mm)')
    axes[1].legend(fontsize=9)
    axes[1].grid(True)

    plt.tight_layout()
    fpath = os.path.join(save_dir, 'step_comparison.png')
    fig.savefig(fpath, dpi=130)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_metrics_table(all_metrics: list, save_dir: str):
    """Save metrics for all trials as a formatted table PNG."""
    col_labels = ['Trial', 'Axis', 'Amp (mm)',
                  'Rise (s)', 'Settle (s)', 'Overshoot (%)',
                  'SS err (mm)', 'RMSE (mm)']
    rows = []
    for label, m in all_metrics:
        def fmt(v):
            return f'{v:.3f}' if not (isinstance(v, float) and np.isnan(v)) else 'N/A'
        rows.append([
            label,
            m['axis'].upper(),
            fmt(m['amplitude_mm']),
            fmt(m['rise_time_s']),
            fmt(m['settling_time_s']),
            fmt(m['overshoot_pct']),
            fmt(m['ss_error_mm']),
            fmt(m['rmse_mm']),
        ])

    fig, ax = plt.subplots(figsize=(13, 1.2 + 0.6 * len(rows)))
    ax.axis('off')
    tbl = ax.table(
        cellText=rows,
        colLabels=col_labels,
        cellLoc='center',
        loc='center',
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.7)

    for col in range(len(col_labels)):
        tbl[0, col].set_facecolor('#1a3a5c')
        tbl[0, col].set_text_props(color='white', fontweight='bold')
    for row in range(1, len(rows) + 1):
        for col in range(len(col_labels)):
            tbl[row, col].set_facecolor('#EAF0F8' if row % 2 == 0 else 'white')

    fig.suptitle('Step-Response Metrics Summary', fontsize=12, fontweight='bold', y=0.98)
    plt.tight_layout()
    fpath = os.path.join(save_dir, 'step_metrics_table.png')
    fig.savefig(fpath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {fpath}')


def save_metrics_csv(all_metrics: list, save_dir: str):
    fpath = os.path.join(save_dir, 'step_metrics.csv')
    with open(fpath, 'w', newline='') as f:
        w = csv_mod.writer(f)
        w.writerow(['trial', 'axis', 'amplitude_mm', 'rise_time_s',
                    'settling_time_s', 'overshoot_pct', 'ss_error_mm', 'rmse_mm'])
        for label, m in all_metrics:
            w.writerow([label, m['axis'], m['amplitude_mm'], m['rise_time_s'],
                        m['settling_time_s'], m['overshoot_pct'],
                        m['ss_error_mm'], m['rmse_mm']])
    print(f'  Saved: {fpath}')


def print_summary(all_metrics: list):
    header = (f"{'Trial':<18} {'Axis':>4} {'Amp(mm)':>8} "
              f"{'Rise(s)':>8} {'Settle(s)':>10} {'OS(%)':>7} "
              f"{'SSe(mm)':>9} {'RMSE(mm)':>9}")
    sep = '─' * len(header)
    print(f'\n{sep}\n{header}\n{sep}')
    for label, m in all_metrics:
        def fv(v):
            return f'{v:>9.3f}' if not (isinstance(v, float) and np.isnan(v)) else f"{'N/A':>9}"
        print(f"  {label:<16} {m['axis'].upper():>4} {m['amplitude_mm']:>8.2f} "
              f"{fv(m['rise_time_s'])} {fv(m['settling_time_s'])} "
              f"{fv(m['overshoot_pct'])} {fv(m['ss_error_mm'])} {fv(m['rmse_mm'])}")
    print(sep + '\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = sys.argv[1:]

    if not args:
        print('Usage: step_analysis.py <csv_file|csv1 csv2 ...|data_dir>')
        sys.exit(1)

    # Resolve CSV paths
    csv_paths = []
    save_dir = '.'
    if len(args) == 1 and os.path.isdir(args[0]):
        save_dir = args[0]
        csv_paths = _discover_step_csvs(args[0])
        if not csv_paths:
            print(f'No step-trial CSVs found in {args[0]}')
            sys.exit(1)
    else:
        csv_paths = [a for a in args if a.endswith('.csv')]
        save_dir = os.path.dirname(csv_paths[0]) if csv_paths else '.'

    if not csv_paths:
        print('No CSV files provided.')
        sys.exit(1)

    print(f'\nStep-response analysis  →  output dir: {save_dir}\n')

    trials = []       # (df, axis, label, metrics, meta)
    all_metrics = []  # (label, metrics_dict)

    for fpath in csv_paths:
        label = _label_from_path(fpath)
        print(f'─── {label}  [{fpath}]')
        df = pd.read_csv(fpath)
        meta = _load_meta(fpath)
        axis = _detect_axis(df, meta)
        metrics = compute_step_metrics(df, axis, meta=meta)

        print(f'    Axis: {axis.upper()}  Amplitude: {metrics["amplitude_mm"]:.2f} mm')
        print(f'    Rise time:     {metrics["rise_time_s"]} s')
        print(f'    Settling time: {metrics["settling_time_s"]} s')
        print(f'    Overshoot:     {metrics["overshoot_pct"]} %')
        print(f'    SS error:      {metrics["ss_error_mm"]} mm')
        print(f'    RMSE:          {metrics["rmse_mm"]} mm')

        plot_step_response(df, axis, label, metrics, save_dir, meta=meta)
        plot_step_error(df, axis, label, metrics, save_dir, meta=meta)

        trials.append((df, axis, label, metrics, meta))
        all_metrics.append((label, metrics))

    plot_comparison(trials, save_dir)
    plot_metrics_table(all_metrics, save_dir)
    save_metrics_csv(all_metrics, save_dir)
    print_summary(all_metrics)


if __name__ == '__main__':
    main()
