#!/usr/bin/env python3
"""
Offline analysis for Challenge 4.1 — CTC vs PD/PID comparison.

Usage:
  python analysis.py [csv_dir]          # auto-discovers 4 trial CSVs
  python analysis.py trial1.csv trial2.csv trial3.csv trial4.csv

Expects trial naming:
  trial_ctc_nopert_*.csv
  trial_pdpid_nopert_*.csv
  trial_ctc_pert_*.csv
  trial_pdpid_pert_*.csv

Generates per-trial:
  - Joint tracking plots (6 subplots, dwell windows highlighted)
  - Task-space plots  (x/y/z tracking, 3D path, EE error norm)
  - Phase portraits   (e_j vs ė_j for all 6 joints)

And comparison plots:
  - No-perturbation: CTC vs PD/PID overlay
  - With-perturbation: CTC vs PD/PID overlay

Plus summary table (console).
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from numpy.fft import rfft, rfftfreq

# Success threshold for waypoint evaluation
EPS_THRESH = 0.005   # m

TRIAL_KEYS = ['ctc_nopert', 'pdpid_nopert', 'ctc_pert', 'pdpid_pert']
TRIAL_LABELS = {
    'ctc_nopert':   'CTC — No Perturbation',
    'pdpid_nopert': 'PD/PID — No Perturbation',
    'ctc_pert':     'CTC — With Perturbation',
    'pdpid_pert':   'PD/PID — With Perturbation',
}


# ---------------------------------------------------------------------------
# CSV discovery
# ---------------------------------------------------------------------------

def discover_csvs(search_path: str):
    files = {}
    for key in TRIAL_KEYS:
        pattern = os.path.join(search_path, f'trial_{key}_*.csv')
        matches = sorted(glob.glob(pattern))
        if matches:
            files[key] = matches[-1]   # most recent
    return files


# ---------------------------------------------------------------------------
# Metric helpers
# ---------------------------------------------------------------------------

def compute_joint_metrics(df):
    """Joint-space RMSE, max error, and dwell-window mean absolute error (§9.1.1)."""
    metrics = {}
    dwell_mask = df['phase'] == 'dwell'
    for j in range(6):
        e = df[f'q_{j}'] - df[f'q_des_{j}']
        metrics[f'rmse_j{j}']       = float(np.sqrt((e**2).mean()))
        metrics[f'maxerr_j{j}']     = float(e.abs().max())
        # Dwell-window mean absolute error: ē_{j,w} averaged over all dwell windows
        e_dwell = e[dwell_mask].abs()
        metrics[f'dwell_mean_j{j}'] = float(e_dwell.mean()) if len(e_dwell) > 0 else 0.0
    metrics['rmse_avg']       = float(np.mean([metrics[f'rmse_j{j}']       for j in range(6)]))
    metrics['maxerr_avg']     = float(np.mean([metrics[f'maxerr_j{j}']     for j in range(6)]))
    metrics['dwell_mean_avg'] = float(np.mean([metrics[f'dwell_mean_j{j}'] for j in range(6)]))
    return metrics


def compute_ee_metrics(df):
    """Task-space end-effector error metrics."""
    e_ee = np.sqrt(
        (df['p_act_x'] - df['p_des_x'])**2 +
        (df['p_act_y'] - df['p_des_y'])**2 +
        (df['p_act_z'] - df['p_des_z'])**2
    )
    rmse_ee  = float(np.sqrt((e_ee**2).mean()))
    max_ee   = float(e_ee.max())
    return e_ee, rmse_ee, max_ee


def waypoint_success_rate(df, e_ee, thresh=EPS_THRESH):
    """
    Success rate: fraction of dwell windows where e_EE < thresh
    during the final 0.5 s of each dwell.
    """
    dwell_mask = df['phase'] == 'dwell'
    labels = df.loc[dwell_mask, 'wp_label'].unique()
    successes = 0
    total = 0
    for lbl in labels:
        lbl_mask = dwell_mask & (df['wp_label'] == lbl)
        t_vals = df.loc[lbl_mask, 'time_rel'].values
        if len(t_vals) < 2:
            continue
        t_end = t_vals.max()
        final_mask = lbl_mask & (df['time_rel'] >= t_end - 0.5)
        if final_mask.sum() == 0:
            continue
        e_final = e_ee[final_mask].mean()
        successes += int(e_final < thresh)
        total += 1
    rate = (successes / total * 100.0) if total > 0 else 0.0
    return rate, successes, total


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _highlight_dwells(ax, df):
    """Shade dwell windows on a time-axis plot."""
    dwell_mask = df['phase'] == 'dwell'
    if not dwell_mask.any():
        return
    t = df['time_rel'].values
    in_dwell = False
    for i, dm in enumerate(dwell_mask):
        if dm and not in_dwell:
            t0 = t[i]
            in_dwell = True
        elif not dm and in_dwell:
            ax.axvspan(t0, t[i], alpha=0.12, color='gray', label='_nolegend_')
            in_dwell = False
    if in_dwell:
        ax.axvspan(t0, t[-1], alpha=0.12, color='gray')


def plot_joint_tracking(df, key, save_dir):
    plt.style.use('bmh')
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(f'Joint Tracking — {TRIAL_LABELS[key]}', fontsize=13, fontweight='bold')
    t = df['time_rel'].values

    for j, ax in enumerate(axes.flat):
        q_meas = df[f'q_{j}'].values
        q_des  = df[f'q_des_{j}'].values
        ax.plot(t, np.degrees(q_des),  'k--', lw=1.5, label='Desired')
        ax.plot(t, np.degrees(q_meas), 'b-',  lw=1.0, alpha=0.85, label='Measured')
        _highlight_dwells(ax, df)
        ax.set_title(f'Joint {j+1}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.legend(fontsize=7)
        ax.grid(True)

    plt.tight_layout()
    fpath = os.path.join(save_dir, f'joints_{key}.png')
    fig.savefig(fpath, dpi=120)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_task_space(df, e_ee, key, save_dir, thresh=EPS_THRESH):
    plt.style.use('bmh')
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'Task-Space — {TRIAL_LABELS[key]}', fontsize=13, fontweight='bold')
    t = df['time_rel'].values

    # x, y, z tracking
    axes_xyz = [fig.add_subplot(3, 2, i+1) for i in range(3)]
    dims = [('x', 0), ('y', 1), ('z', 2)]
    for ax, (dim, _) in zip(axes_xyz, dims):
        ax.plot(t, df[f'p_des_{dim}'].values * 1000, 'k--', lw=1.5, label='Desired')
        ax.plot(t, df[f'p_act_{dim}'].values * 1000, 'b-',  lw=1.0, alpha=0.85, label='Actual')
        _highlight_dwells(ax, df)
        ax.set_title(f'{dim.upper()} tracking')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('mm')
        ax.legend(fontsize=8); ax.grid(True)

    # 3D path
    ax3d = fig.add_subplot(3, 2, 4, projection='3d')
    ax3d.plot(df['p_des_x'], df['p_des_y'], df['p_des_z'],
              'k--', lw=1.5, label='Desired', zorder=1)
    ax3d.plot(df['p_act_x'], df['p_act_y'], df['p_act_z'],
              'b-',  lw=0.8, alpha=0.8, label='Actual', zorder=2)

    # Mark waypoints from trajectory definition
    try:
        from xarm_ctc_challenge.trajectory import WAYPOINTS, P_HOME
        wp_xyz = np.array([wp[:3] for wp in WAYPOINTS])
        # High-layer waypoints (transit)
        high_mask = wp_xyz[:, 2] >= 0.35
        low_mask  = ~high_mask
        ax3d.scatter(wp_xyz[high_mask, 0], wp_xyz[high_mask, 1], wp_xyz[high_mask, 2],
                     c='orange', marker='^', s=60, zorder=5, label='WP (transit)')
        ax3d.scatter(wp_xyz[low_mask, 0],  wp_xyz[low_mask, 1],  wp_xyz[low_mask, 2],
                     c='red',    marker='o', s=60, zorder=5, label='WP (inspect)')
        # Home position
        ax3d.scatter(*P_HOME, c='green', marker='*', s=120, zorder=6, label='HOME')
        # Annotate waypoint labels
        for wp in WAYPOINTS:
            ax3d.text(wp[0], wp[1], wp[2], f' {wp[4]}', fontsize=5, color='dimgray')
    except ImportError:
        pass

    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3D Path (base frame)'); ax3d.legend(fontsize=6)

    # EE error norm
    ax_err = fig.add_subplot(3, 1, 3)
    ax_err.plot(t, e_ee * 1000, color='crimson', lw=1.0, label='||e_EE||')
    ax_err.axhline(thresh * 1000, color='k', ls='--', label=f'Threshold {thresh*1000:.1f} mm')
    _highlight_dwells(ax_err, df)
    if 'pert_enabled' in df.columns:
        pert_mask = df['pert_enabled'].astype(bool)
        if pert_mask.any():
            ax_err.fill_between(t, 0, e_ee.max() * 1000,
                                where=pert_mask.values, alpha=0.15,
                                color='orange', label='Perturbation active')
    ax_err.set_title('End-Effector Error Norm')
    ax_err.set_xlabel('Time (s)'); ax_err.set_ylabel('Error (mm)')
    ax_err.legend(fontsize=8); ax_err.grid(True)

    plt.tight_layout()
    fpath = os.path.join(save_dir, f'taskspace_{key}.png')
    fig.savefig(fpath, dpi=120)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_phase_portraits(df, key, save_dir):
    plt.style.use('bmh')
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    fig.suptitle(f'Phase Portraits — {TRIAL_LABELS[key]}', fontsize=13, fontweight='bold')
    t = df['time_rel'].values

    for j, ax in enumerate(axes.flat):
        e_j  = (df[f'q_des_{j}'] - df[f'q_{j}']).values
        de_j = np.gradient(e_j, t)
        ax.plot(np.degrees(e_j), np.degrees(de_j),
                color='steelblue', alpha=0.5, lw=0.6)
        ax.scatter([0], [0], color='k', s=60, zorder=5, label='Equilibrium')
        ax.set_title(f'Joint {j+1}')
        ax.set_xlabel('e_j (deg)'); ax.set_ylabel('ė_j (deg/s)')
        ax.legend(fontsize=7); ax.grid(True)

    plt.tight_layout()
    fpath = os.path.join(save_dir, f'phase_{key}.png')
    fig.savefig(fpath, dpi=120)
    plt.close(fig)
    print(f'  Saved: {fpath}')


def plot_comparison(dfs, e_ee_dict, pert: bool, save_dir):
    suffix = "pert" if pert else "nopert"
    keys   = [f'ctc_{suffix}', f'pdpid_{suffix}']
    avail  = [k for k in keys if k in dfs]
    if len(avail) < 2:
        return

    plt.style.use('bmh')
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    title_sfx = "With Perturbation" if pert else "No Perturbation"
    fig.suptitle(f'CTC vs PD/PID Comparison — {title_sfx}', fontsize=13, fontweight='bold')

    colors = {'ctc': 'crimson', 'pdpid': 'steelblue'}
    ls     = {'ctc': '-', 'pdpid': '--'}

    for key in avail:
        df  = dfs[key]
        e   = e_ee_dict[key]
        t   = df['time_rel'].values
        tag = key.split('_')[0]
        axes[0].plot(t, e * 1000, color=colors[tag], ls=ls[tag],
                     lw=1.0, label=TRIAL_LABELS[key])
        axes[1].plot(t, e * 1000, color=colors[tag], ls=ls[tag],
                     lw=1.0, label=TRIAL_LABELS[key])

    for ax in axes:
        ax.axhline(EPS_THRESH * 1000, color='k', ls=':', label=f'Threshold {EPS_THRESH*1000:.1f} mm')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('||e_EE|| (mm)')
        ax.legend(fontsize=8); ax.grid(True)
    axes[0].set_title('EE Error Norm Overlay')
    axes[1].set_title('EE Error Norm Overlay (zoom)')

    # Zoom to the dwell windows region if available
    df0 = dfs[avail[0]]
    dwell_start = df0.loc[df0['phase'] == 'dwell', 'time_rel']
    if not dwell_start.empty:
        axes[1].set_xlim(dwell_start.min(), dwell_start.max() + 5)

    plt.tight_layout()
    fpath = os.path.join(save_dir, f'comparison_{suffix}.png')
    fig.savefig(fpath, dpi=120)
    plt.close(fig)
    print(f'  Saved: {fpath}')


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------

def print_summary(metrics_all):
    header = f"{'Metric':<30} {'CTC NP':>10} {'PD NP':>10} {'CTC P':>10} {'PD P':>10}"
    sep = '─' * len(header)
    print(f"\n{sep}\n{header}\n{sep}")

    row_keys = [
        ('Joint RMSE avg (mrad)',        'rmse_avg',       1000.0),
        ('Joint MaxErr avg (mrad)',      'maxerr_avg',     1000.0),
        ('Joint Dwell Mean avg (mrad)',  'dwell_mean_avg', 1000.0),
        ('EE RMSE (mm)',                 'ee_rmse',        1000.0),
        ('EE MaxErr (mm)',               'ee_max',         1000.0),
        ('Waypoint Success (%)',         'wp_success',       1.0),
    ]
    for (name, mkey, scale) in row_keys:
        vals = []
        for key in TRIAL_KEYS:
            m = metrics_all.get(key, {})
            vals.append(f"{m.get(mkey, float('nan')) * scale:>10.2f}"
                        if mkey in m else f"{'N/A':>10}")
        print(f"  {name:<28} {'  '.join(vals)}")
    print(sep + '\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) >= 5:
        # Four explicit CSV files
        csv_map = {}
        for path in sys.argv[1:5]:
            for key in TRIAL_KEYS:
                if key in os.path.basename(path):
                    csv_map[key] = path
                    break
        else:
            for i, key in enumerate(TRIAL_KEYS):
                csv_map[key] = sys.argv[1 + i]
    else:
        search = sys.argv[1] if len(sys.argv) > 1 else '.'
        csv_map = discover_csvs(search)

    if not csv_map:
        print('No trial CSVs found. Provide a directory or explicit file paths.')
        sys.exit(1)

    save_dir = sys.argv[1] if (len(sys.argv) > 1 and os.path.isdir(sys.argv[1])) else '.'
    print(f'\nAnalysing {len(csv_map)} trial(s) → output dir: {save_dir}\n')

    dfs        = {}
    e_ee_dict  = {}
    metrics_all = {}

    for key, fpath in csv_map.items():
        print(f'─── {TRIAL_LABELS[key]} ───')
        print(f'    CSV: {fpath}')
        df = pd.read_csv(fpath)
        dfs[key] = df

        e_ee, rmse_ee, max_ee = compute_ee_metrics(df)
        e_ee_dict[key] = e_ee.values if hasattr(e_ee, 'values') else e_ee

        jm = compute_joint_metrics(df)
        sr, n_ok, n_tot = waypoint_success_rate(df, e_ee)

        metrics_all[key] = {
            **jm,
            'ee_rmse':    rmse_ee,
            'ee_max':     max_ee,
            'wp_success': sr,
        }

        print(f'    Joint RMSE avg : {jm["rmse_avg"]*1000:.3f} mrad')
        print(f'    EE RMSE        : {rmse_ee*1000:.3f} mm')
        print(f'    EE Max Error   : {max_ee*1000:.3f} mm')
        print(f'    Waypoint SR    : {sr:.1f}%  ({n_ok}/{n_tot})')

        plot_joint_tracking(df, key, save_dir)
        plot_task_space(df, e_ee_dict[key], key, save_dir)
        plot_phase_portraits(df, key, save_dir)

    # Comparison plots
    plot_comparison(dfs, e_ee_dict, pert=False, save_dir=save_dir)
    plot_comparison(dfs, e_ee_dict, pert=True,  save_dir=save_dir)

    # Summary
    print_summary(metrics_all)


if __name__ == '__main__':
    main()
