import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import rfft, rfftfreq
import sys
import os


# ---------------------------------------------------------------------------
# Shared helpers  (imported by analysis_evaluation_only.py as well)
# ---------------------------------------------------------------------------

def _compute_errors(df):
    df = df.copy()
    df['e_x']   = df['x_des'] - df['x_act']
    df['e_y']   = df['y_des'] - df['y_act']
    df['e_z']   = df['z_des'] - df['z_act']
    df['v_mag'] = np.sqrt(df['vx_cmd']**2 + df['vy_cmd']**2 + df['vz_cmd']**2)
    df['e_norm'] = np.sqrt(df['e_x']**2 + df['e_y']**2 + df['e_z']**2)
    return df


def _print_metrics(df, label=""):
    rmse_x     = np.sqrt((df['e_x']**2).mean())
    rmse_y     = np.sqrt((df['e_y']**2).mean())
    rmse_z     = np.sqrt((df['e_z']**2).mean())
    rmse_total = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)

    max_err_x     = df['e_x'].abs().max()
    max_err_y     = df['e_y'].abs().max()
    max_err_z     = df['e_z'].abs().max()
    max_err_total = max(max_err_x, max_err_y, max_err_z)

    print(f"\n{'='*52}")
    if label:
        print(f"  {label}")
    print(f"{'='*52}")
    print(f"  RMSE X         : {rmse_x:.5f} m")
    print(f"  RMSE Y         : {rmse_y:.5f} m")
    print(f"  RMSE Z         : {rmse_z:.5f} m")
    print(f"  Total RMSE     : {rmse_total:.5f} m")
    print(f"  {'-'*44}")
    print(f"  Max |Error| X  : {max_err_x:.5f} m")
    print(f"  Max |Error| Y  : {max_err_y:.5f} m")
    print(f"  Max |Error| Z  : {max_err_z:.5f} m")
    print(f"  Max |Error| 3D : {max_err_total:.5f} m")
    print(f"{'='*52}\n")

    return rmse_x, rmse_y, rmse_z, rmse_total


def _plot_time_domain(df, t_rel, title):
    """Standard 3-panel time-domain figure."""
    plt.style.use('bmh')
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(title, fontsize=14, fontweight='bold')

    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(df['x_des'], df['y_des'], 'k--', label='Deseada', linewidth=2)
    ax1.plot(df['x_act'], df['y_act'], 'g-',  label='Actual',  alpha=0.8, linewidth=1.5)
    ax1.set_title("1. Trayectoria XY")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.axis('equal')

    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(t_rel, df['e_x'], label='Error X', color='r', alpha=0.8)
    ax2.plot(t_rel, df['e_y'], label='Error Y', color='g', alpha=0.8)
    ax2.plot(t_rel, df['e_z'], label='Error Z', color='b', alpha=0.8)
    ax2.set_title("2. Error de posicion")
    ax2.set_xlabel("Tiempo en prueba (s)")
    ax2.set_ylabel("Error (m)")
    ax2.legend()

    ax3 = plt.subplot(2, 1, 2)
    ax3.plot(t_rel, df['v_mag'], color='purple', label='|V_cmd|')
    ax3.set_title("3. Magnitud velocidad comandada")
    ax3.set_xlabel("Tiempo en prueba (s)")
    ax3.set_ylabel("Velocidad (m/s)")
    ax3.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    return fig


def _plot_frequency_domain(df, title, perturbation_info=""):
    """
    Frequency-domain analysis with 4 subplots:
      (a) FFT magnitude per axis
      (b) Total error FFT + -3dB bandwidth estimation
      (c) Phase portrait  e_x vs de_x/dt
      (d) Phase portrait  e_y vs de_y/dt

    perturbation_info: optional string printed on the figure
    (e.g. "Sine: A=0.02 m/s, f=0.5 Hz, axis=x"
          "Gaussian: sigma=0.01 m/s")
    """
    ts = df['time'].values
    dt_mean = float(np.mean(np.diff(ts)))
    N = len(ts)

    freqs = rfftfreq(N, d=dt_mean)

    Ex    = np.abs(rfft(df['e_x'].values))   * 2.0 / N
    Ey    = np.abs(rfft(df['e_y'].values))   * 2.0 / N
    Ez    = np.abs(rfft(df['e_z'].values))   * 2.0 / N
    E_tot = np.abs(rfft(df['e_norm'].values)) * 2.0 / N

    # -3 dB bandwidth (skip DC bin 0)
    search = E_tot[1:]
    peak_val = search.max()
    bw_candidates = np.where(search <= peak_val / np.sqrt(2))[0]
    bw_hz = freqs[bw_candidates[0] + 1] if len(bw_candidates) > 0 else freqs[-1]
    dom_hz = freqs[1:][np.argmax(E_tot[1:])]

    de_x = np.gradient(df['e_x'].values, ts)
    de_y = np.gradient(df['e_y'].values, ts)

    plt.style.use('bmh')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    suptitle = f"{title} — Analisis en Frecuencia"
    if perturbation_info:
        suptitle += f"\n{perturbation_info}"
    fig.suptitle(suptitle, fontsize=12, fontweight='bold')

    # (a) Per-axis FFT
    ax = axes[0, 0]
    ax.semilogy(freqs[1:], Ex[1:],  label='|E_x(f)|', color='r', alpha=0.8)
    ax.semilogy(freqs[1:], Ey[1:],  label='|E_y(f)|', color='g', alpha=0.8)
    ax.semilogy(freqs[1:], Ez[1:],  label='|E_z(f)|', color='b', alpha=0.8)
    ax.set_xlabel("Frecuencia (Hz)")
    ax.set_ylabel("Amplitud (m)")
    ax.set_title("(a) Espectro de error por eje")
    ax.legend()
    ax.grid(True, which='both')

    # (b) Total error + bandwidth
    ax = axes[0, 1]
    ax.semilogy(freqs[1:], E_tot[1:], color='purple', label='|E_total(f)|')
    ax.axhline(peak_val / np.sqrt(2), color='k',     linestyle='--',
               label='-3 dB threshold')
    ax.axvline(bw_hz,                 color='orange', linestyle='--',
               label=f'BW = {bw_hz:.4f} Hz')
    ax.axvline(dom_hz,                color='red',    linestyle=':',
               label=f'Dominant = {dom_hz:.4f} Hz')
    ax.set_xlabel("Frecuencia (Hz)")
    ax.set_ylabel("Amplitud (m)")
    ax.set_title(f"(b) Error total — BW = {bw_hz:.4f} Hz")
    ax.legend(fontsize=8)
    ax.grid(True, which='both')

    # (c) Phase portrait X
    ax = axes[1, 0]
    ax.plot(df['e_x'].values, de_x, color='r', alpha=0.6, linewidth=0.8)
    ax.scatter([0], [0], color='k', s=60, zorder=5, label='Equilibrio')
    ax.set_xlabel("e_x (m)")
    ax.set_ylabel("ė_x (m/s)")
    ax.set_title("(c) Retrato de fase — eje X")
    ax.legend()
    ax.grid(True)

    # (d) Phase portrait Y
    ax = axes[1, 1]
    ax.plot(df['e_y'].values, de_y, color='g', alpha=0.6, linewidth=0.8)
    ax.scatter([0], [0], color='k', s=60, zorder=5, label='Equilibrio')
    ax.set_xlabel("e_y (m)")
    ax.set_ylabel("ė_y (m/s)")
    ax.set_title("(d) Retrato de fase — eje Y")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()

    print(f"  Ancho de banda estimado (-3dB): {bw_hz:.5f} Hz")
    print(f"  Frecuencia dominante del error: {dom_hz:.5f} Hz")

    return fig, bw_hz


# ---------------------------------------------------------------------------
# Perturbation metadata strings — edit to match your actual experiment values
# ---------------------------------------------------------------------------
PERTURBATION_INFO = {
    "Baseline":  "",
    "Sine":      "Perturbacion senoidal: A = 0.02 m/s, f = 0.5 Hz, eje = x",
    "Gaussian":  "Perturbacion gaussiana: sigma = 0.01 m/s (todos los ejes)",
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    csv_filename = "baseline.csv"

    try:
        df_full = pd.read_csv(csv_filename)
    except FileNotFoundError:
        print(f"Error: No se encontro el archivo {csv_filename}")
        sys.exit(1)

    print(f"Analizando datos de: {csv_filename}")

    df_full = _compute_errors(df_full)

    # Time segments — edit these to match your recording
    segmentos = {
        "Baseline": (0,   75),
        "Sine":     (75,  225),
        "Gaussian": (260, df_full['time'].max()),
    }

    for nombre_exp, (t_inicio, t_fin) in segmentos.items():
        print(f"\n{'='*50}")
        print(f"EXPERIMENTO: {nombre_exp.upper()}  ({t_inicio}s — {t_fin}s)")
        print(f"{'='*50}")

        df = df_full[
            (df_full['time'] >= t_inicio) & (df_full['time'] <= t_fin)
        ].copy()

        if df.empty:
            print(f"Advertencia: no hay datos en [{t_inicio}, {t_fin}]")
            continue

        t_rel = df['time'] - t_inicio

        # ── Time-domain metrics & plots ────────────────────────────────────
        _print_metrics(df, label=nombre_exp)

        fig_td = _plot_time_domain(df, t_rel, title=f"Experimento: {nombre_exp}")
        nombre_td = f"resultados_{nombre_exp.lower()}.png"
        fig_td.savefig(nombre_td)
        print(f"Figura guardada: {nombre_td}")
        plt.show()

        # ── Frequency-domain analysis ──────────────────────────────────────
        pert_info = PERTURBATION_INFO.get(nombre_exp, "")
        fig_fd, bw = _plot_frequency_domain(
            df, title=f"Experimento: {nombre_exp}",
            perturbation_info=pert_info)
        nombre_fd = f"frecuencia_{nombre_exp.lower()}.png"
        fig_fd.savefig(nombre_fd)
        print(f"Figura guardada: {nombre_fd}")
        plt.show()


if __name__ == '__main__':
    main()
