"""
Waypoint trajectory simulator — matches Start_Waypoint_Segment() logic in main.c
Run:  python plot_trajectory.py
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parameters (mirror your Live Expressions values) ─────────────────────────
WP_POS_DEG  = [90.0, 180.0, 45.0, 0.0]   # absolute targets  (degrees)
WP_TIME_SEC = [1.0,   0.8,  1.2,  1.0]   # segment durations (seconds)
START_POS_DEG = 0.0

MAX_VELOCITY = 6.28   # rad/s  — safety ceiling
T1_SEG       = 0.10   # s  — S-curve jerk duration
T2_SEG       = 0.10   # s  — S-curve const-accel duration
T_ACC_SEG    = 0.30   # s  — Trapezoid accel duration

DT           = 0.002  # s  — matches POS_DT (500 Hz TIM7)
DEG2RAD      = np.pi / 180.0

# ── Safety clamp (same formula as wp_load handler in main.c) ─────────────────
def min_safe_time(abs_disp_rad, traj_type):
    vel_t = abs_disp_rad / MAX_VELOCITY if MAX_VELOCITY > 0 else 0.0
    if traj_type == "trapezoid":
        return max(2 * T_ACC_SEG, T_ACC_SEG + vel_t)
    else:  # scurve
        ramp = 4 * T1_SEG + 2 * T2_SEG
        need = 2 * T1_SEG + T2_SEG + vel_t
        return max(ramp, need)

# ── S-curve segment (matches SCurve_SetTarget_ByTime + SCurve_Update) ────────
def scurve_segment(disp_rad, seg_time):
    d   = abs(disp_rad)
    sgn = 1.0 if disp_rad >= 0 else -1.0

    t_cruise = max(0.0, seg_time - 4 * T1_SEG - 2 * T2_SEG)
    denom    = 2 * T1_SEG + T2_SEG + t_cruise
    v_peak   = d / denom if denom > 1e-9 else 0.0
    a_peak   = v_peak / (T1_SEG + T2_SEG) if (T1_SEG + T2_SEG) > 1e-9 else 0.0
    j_val    = a_peak / T1_SEG if T1_SEG > 1e-9 else 0.0

    # Cumulative segment boundaries
    T = np.array([
        0,
        T1_SEG,
        T1_SEG + T2_SEG,
        2 * T1_SEG + T2_SEG,
        2 * T1_SEG + T2_SEG + t_cruise,
        3 * T1_SEG + T2_SEG + t_cruise,
        3 * T1_SEG + 2 * T2_SEG + t_cruise,
        4 * T1_SEG + 2 * T2_SEG + t_cruise,
    ])

    # Velocity at each boundary (positive direction)
    v1 = j_val * T1_SEG ** 2 / 2
    v2 = v1 + a_peak * T2_SEG

    t_arr = np.arange(0, T[7] + DT / 2, DT)
    vel   = np.zeros(len(t_arr))
    acc   = np.zeros(len(t_arr))
    jrk   = np.zeros(len(t_arr))

    for i, t in enumerate(t_arr):
        if   t <= T[1]:  tau = t - T[0]; jk = +j_val; a =  j_val*tau;              v = j_val*tau**2/2
        elif t <= T[2]:  tau = t - T[1]; jk = 0;       a =  a_peak;                v = v1 + a_peak*tau
        elif t <= T[3]:  tau = t - T[2]; jk = -j_val; a =  a_peak - j_val*tau;    v = v2 + a_peak*tau - j_val*tau**2/2
        elif t <= T[4]:                  jk = 0;       a =  0;                      v = v_peak
        elif t <= T[5]:  tau = t - T[4]; jk = -j_val; a = -j_val*tau;              v = v_peak - j_val*tau**2/2
        elif t <= T[6]:  tau = t - T[5]; jk = 0;       a = -a_peak;                v = (v_peak - v1) - a_peak*tau
        else:            tau = t - T[6]; jk = +j_val; a = -a_peak + j_val*tau;    v = v1 - a_peak*tau + j_val*tau**2/2
        vel[i] = sgn * max(0.0, v)
        acc[i] = sgn * a
        jrk[i] = sgn * jk

    pos = np.zeros(len(t_arr))
    for i in range(1, len(t_arr)):
        pos[i] = pos[i-1] + vel[i] * DT

    return t_arr, pos, vel, acc, jrk

# ── Trapezoid segment (matches Trapezoid_SetTarget_ByTime + Trapezoid_Update) ─
def trapezoid_segment(disp_rad, seg_time):
    d   = abs(disp_rad)
    sgn = 1.0 if disp_rad >= 0 else -1.0

    t_cruise = max(0.0, seg_time - 2 * T_ACC_SEG)
    denom    = T_ACC_SEG + t_cruise
    v_peak   = d / denom if denom > 1e-9 else 0.0
    a_used   = v_peak / T_ACC_SEG if T_ACC_SEG > 1e-9 else 0.0

    T1 = T_ACC_SEG
    T2 = T_ACC_SEG + t_cruise
    T3 = T2 + T_ACC_SEG

    t_arr = np.arange(0, T3 + DT / 2, DT)
    vel   = np.zeros(len(t_arr))
    acc   = np.zeros(len(t_arr))

    for i, t in enumerate(t_arr):
        if   t <= T1: tau = t;      v = a_used * tau;              a =  a_used
        elif t <= T2:               v = v_peak;                    a =  0.0
        else:         tau = t - T2; v = v_peak - a_used * tau;    a = -a_used
        vel[i] = sgn * max(0.0, v)
        acc[i] = sgn * a

    pos = np.zeros(len(t_arr))
    for i in range(1, len(t_arr)):
        pos[i] = pos[i-1] + vel[i] * DT

    jrk = np.zeros(len(t_arr))   # trapezoid has no jerk output
    return t_arr, pos, vel, acc, jrk

# ── Build full sequence ───────────────────────────────────────────────────────
def simulate(traj_type):
    seg_fn   = scurve_segment if traj_type == "scurve" else trapezoid_segment

    all_t, all_p, all_v, all_a, all_j = [], [], [], [], []
    wp_boundaries = [0.0]
    t_offset   = 0.0
    prev_deg   = START_POS_DEG
    clamped    = []

    for i, (target_deg, req_time) in enumerate(zip(WP_POS_DEG, WP_TIME_SEC)):
        disp_rad = (target_deg - prev_deg) * DEG2RAD
        abs_disp = abs(disp_rad)
        min_t    = min_safe_time(abs_disp, traj_type)
        actual_t = max(req_time, min_t)
        clamped.append(actual_t > req_time + 1e-6)

        t_arr, pos, vel, acc, jrk = seg_fn(disp_rad, actual_t)

        # Convert to degrees and offset to absolute position
        start_rad = prev_deg * DEG2RAD
        all_t.append(t_arr + t_offset)
        all_p.append((pos + start_rad) / DEG2RAD)
        all_v.append(vel / DEG2RAD)
        all_a.append(acc / DEG2RAD)
        all_j.append(jrk / DEG2RAD)

        t_offset += t_arr[-1]
        prev_deg  = target_deg
        wp_boundaries.append(t_offset)

    return (np.concatenate(all_t), np.concatenate(all_p),
            np.concatenate(all_v), np.concatenate(all_a),
            np.concatenate(all_j), wp_boundaries, clamped)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10))
fig.suptitle("Waypoint Trajectory — Cascade Control (Mode 3)\n"
             f"Waypoints: {WP_POS_DEG}°   Times: {WP_TIME_SEC} s   "
             f"max_velocity = {MAX_VELOCITY} rad/s", fontsize=11)

gs = gridspec.GridSpec(4, 2, hspace=0.55, wspace=0.35)
titles   = ["Position (°)", "Velocity (°/s)", "Acceleration (°/s²)", "Jerk (°/s³)"]
colors   = ["#2196F3", "#4CAF50", "#FF9800", "#9C27B0"]
signals  = [1, 2, 3, 4]   # indices into simulate() result

for col, (label, ttype) in enumerate([("S-Curve", "scurve"), ("Trapezoid", "trapezoid")]):
    t, p, v, a, j, boundaries, clamped = simulate(ttype)
    data = [p, v, a, j]

    for row, (sig, title, color) in enumerate(zip(data, titles, colors)):
        ax = fig.add_subplot(gs[row, col])
        ax.plot(t, sig, color=color, linewidth=1.6)

        # Waypoint boundary lines
        for bi, bt in enumerate(boundaries):
            ax.axvline(bt, color="gray", linestyle="--", linewidth=0.8, alpha=0.7)

        # Waypoint labels on position row
        if row == 0:
            ax.set_title(f"{label}  (t1={T1_SEG}s, t2={T2_SEG}s)" if ttype == "scurve"
                         else f"{label}  (t_acc={T_ACC_SEG}s)", fontsize=9)
            for i, (deg, bt) in enumerate(zip([START_POS_DEG] + WP_POS_DEG,
                                               boundaries)):
                ax.annotate(f"WP{i}\n{deg:.0f}°", xy=(bt, deg),
                            fontsize=7, ha="center", va="bottom", color="gray")
            for i, cl in enumerate(clamped):
                if cl:
                    bt = boundaries[i + 1]
                    ax.annotate("⚠ clamped", xy=(bt, WP_POS_DEG[i]),
                                fontsize=7, color="red", ha="right")

        ax.set_ylabel(title, fontsize=8)
        ax.set_xlabel("Time (s)" if row == 3 else "", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="black", linewidth=0.5)

        # Velocity: overlay max_velocity limit
        if row == 1:
            lim = MAX_VELOCITY / DEG2RAD
            ax.axhline( lim, color="red", linestyle=":", linewidth=1, label=f"+max ({MAX_VELOCITY} rad/s)")
            ax.axhline(-lim, color="red", linestyle=":", linewidth=1)
            ax.legend(fontsize=7, loc="upper right")

plt.savefig("trajectory_plot.png", dpi=150, bbox_inches="tight")
print("Saved: trajectory_plot.png")
plt.show()
