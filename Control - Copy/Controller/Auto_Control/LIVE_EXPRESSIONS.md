# Live Expressions — Developer UI
`Auto_Control Debug.launch`

Variables are pre-loaded in the `.launch` file in the order below.
To reset: close the project, edit the `.launch` file, reopen — do NOT add XML comments inside `<listAttribute>`.

---

## 1 · Master Control

| Variable | Type | Description |
|----------|------|-------------|
| `control_mode` | uint8 R/W | 3 = Cascade (trajectory + pos PID + vel PID) · 7 = Kalman4 Cal |
| `traj_type` | uint8 R/W | 0 = S-Curve · 1 = Trapezoid |
| `pid_enabled` | uint8 R/W | 0 = freeze all PIDs, hold PWM=0 · 1 = run |
| `start_move` | uint8 R/W | Write 1 to (re)start trajectory — self-clears |
| `reset_all` | uint8 R/W | Write 1 to clear ALL state (PIDs, traj, encoder, Kalman) — self-clears |
| `zero_encoder` | uint8 R/W | Write 1 to zero encoder position — self-clears |
| `apply_motor_params` | uint8 R/W | Write 1 to re-init Kalman4 + RefFF with current motor_* values — self-clears |

---

## 2 · Trajectory

| Variable | Type | Description |
|----------|------|-------------|
| `traj_target_deg` | float R/W | Target position (deg) — changing auto-starts trajectory |
| `max_velocity` | float R/W | Peak velocity (rad/s) |
| `max_accel` | float R/W | Peak acceleration (rad/s²) |
| `max_jerk` | float R/W | Peak jerk — S-curve only (rad/s³) |
| `t1_seg` | float R/W | S-curve: jerk segment duration (s) |
| `t2_seg` | float R/W | S-curve: constant-accel segment duration (s) |
| `t_acc_seg` | float R/W | Trapezoid: acceleration duration (s) |
| `t_cruise_seg` | float R/W | Trapezoid: cruise duration (s) · S-curve: read-only auto-computed |
| `t_dec_seg` | float R/W | Trapezoid: deceleration duration (s) |

---

## 3 · Position PID — TIM7 Outer Loop (500 Hz)

| Variable | Type | Description |
|----------|------|-------------|
| `kp_position` | float R/W | Proportional gain (rad/s per rad) |
| `ki_pos` | float R/W | Integral gain (rad/s per rad·s) |
| `kd_pos` | float R/W | Derivative gain — acts on −velocity (rad/s per rad/s) |
| `pos_deadband_deg` | float R/W | Position error threshold for hard stop (deg) |
| `pos_integral_live` | float R/O | Position integrator accumulator — watch for windup |

---

## 4 · Velocity PID — TIM6 Inner Loop (1000 Hz)

| Variable | Type | Description |
|----------|------|-------------|
| `kp_vel` | float R/W | Proportional gain (% per rad/s) |
| `ki_vel` | float R/W | Integral gain (% per rad) |
| `kd_vel` | float R/W | Derivative gain — keep 0 at 1 kHz (noisy) |
| `min_pwm_threshold` | float R/W | Dead-zone floor — raise to overcome static friction (%) |
| `use_kf4_vel` | uint8 R/W | Velocity source: **0** = raw encoder (`g_velocity_rad_s`) · **1** = Kalman4 (`g_kf4_velocity`) — applies to both velocity PID and position PID kd term |

---

## 5 · Motor Parameters

Change any value, then write `apply_motor_params = 1` to apply without resetting position.
`reset_all = 1` also applies current values.

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `motor_J_eq` | float R/W | 0.027 | Equivalent inertia (kg·m²) |
| `motor_b_eq` | float R/W | 0.5 | Equivalent viscous friction (N·m·s/rad) |
| `motor_N` | float R/W | 50.0 | Gear ratio |
| `motor_Kt` | float R/W | 0.00747 | Torque constant (N·m/A) |
| `motor_Ke` | float R/W | 0.0083 | Back-EMF constant (V·s/rad) |
| `motor_L` | float R/W | 0.0012794 | Armature inductance (H) |
| `motor_R_arm` | float R/W | 2.8 | Armature resistance (Ω) |

---

## 6 · Reference Feedforward

| Variable | Type | Description |
|----------|------|-------------|
| `refff_enabled` | uint8 R/W | 0 = off · 1 = active (adds model-based PWM on top of vel PID) |
| `V_supply` | float R/W | Motor bus voltage (V) — must match actual supply |
| `distff_enabled` | uint8 R/W | 0 = off · 1 = active (adds disturbance cancellation PWM) |
| `distff_tau` | float R/W | DistFF filter time constant (s) — apply with `apply_motor_params = 1` |
| `g_distff_pwm` | float R/O | DistFF PWM addend (%) — observe before enabling to check magnitude |

---

## 7 · Kalman4 — 4-State Motor Filter

| Variable | Type | Description |
|----------|------|-------------|
| `kf4_sigma_v2` | float R/W | σ_v² — voltage noise variance (drives Q second-order expansion; Q[2][2], Q[1][2], Q[1][1] non-zero) |
| `kf4_r_theta` | float R/W | σ_θ² — position measurement noise variance |
| `g_kf4_position` | float R/O | Kalman4 estimated position (rad) |
| `g_kf4_velocity` | float R/O | Kalman4 estimated velocity (rad/s) |
| `g_kf4_current` | float R/O | Kalman4 estimated current (A) — model only, no sensor |
| `g_kf4_tau_d_obs` | float R/O | Kalman4 estimated disturbance torque (N·m) |

> Kalman4 runs every TIM6 tick in all modes. After tuning noise params, write `reset_all = 1`.

---

## 8 · Mode 7 — Kalman4 Calibration

| Variable | Type | Description |
|----------|------|-------------|
| `kf_cal_pwm` | float R/W | DC offset PWM (−100 to +100 %) |
| `kf_sine_enabled` | uint8 R/W | 0 = constant `kf_cal_pwm` · 1 = sine wave active |
| `kf_sine_amp` | float R/W | Sine amplitude (%) |
| `kf_sine_freq` | float R/W | Sine frequency (Hz) |
| `kf_sine_dir` | float R/W | +1.0 = normal · −1.0 = reversed |

---

## 9 · Current Sensor — ADC1 (PA1)

| Variable | Type | Description |
|----------|------|-------------|
| `current_zero_counts` | float R/W | ADC count at zero current — trim until `g_current_A ≈ 0` at standstill |
| `current_counts_per_amp` | float R/W | ADC counts per ampere |
| `adc_raw_current` | uint32 R/O | Raw 12-bit ADC count (0–4095) |
| `g_current_A` | float R/O | Computed motor current (A) |

> Formula: `I = (adc_raw − current_zero_counts) / current_counts_per_amp`

---

## 10 · Observables — Read Only

| Variable | Units | Description |
|----------|-------|-------------|
| `ss_error_deg` | deg | Position error (ideal − actual) |
| `ss_error_rad` | rad | Position error (ideal − actual) |
| `ss_reached` | — | 1 = error within `pos_deadband_deg` |
| `ss_final_error_deg` | deg | Error frozen at moment hard stop fired |
| `vel_error_live` | rad/s | Current velocity error into vel PID |
| `i_term_live` | — | Velocity integrator accumulator |
| `g_traj_pos` | rad | Trajectory ideal position |
| `g_smooth_vel` | rad/s | Trajectory velocity |
| `g_smooth_accel` | rad/s² | Trajectory acceleration |
| `g_jerk` | rad/s³ | Trajectory jerk (0 for trapezoid) |
| `g_position_rad` | rad | Actual encoder position |
| `g_velocity_rad_s` | rad/s | Actual raw encoder velocity |
| `g_pwm_duty` | % | PWM output to motor |
| `g_motor_voltage` | V | Estimated motor voltage (used by Kalman4) |

---

## 11 · Hardware

| Variable | Type | Description |
|----------|------|-------------|
| `motor_dir_inverted` | uint8 R/W | 0 = normal · 1 = flip direction |
| `gripper_updown` | uint8 R/W | PB1: 0 = down · 1 = up |
| `gripper_openclose` | uint8 R/W | PB2: 0 = close · 1 = open |
| `emer_output` | uint8 R/W | PC11 relay: 0 = de-energise · 1 = energise |

---

## 12 · Sequence Mode

| Variable | Type | Description |
|----------|------|-------------|
| `seq_targets[0]` … `seq_targets[8]` | float R/W | Target positions (deg) — fill in order |
| `seq_count` | uint8 R/W | Number of active steps (1–9) |
| `seq_enabled` | uint8 R/W | Write 1 to start — self-clears when complete · write 0 to abort |
| `seq_index` | uint8 R/O | Current step (0-based) |
| `seq_step_delay` | float R/W | Dwell time between steps (s) — default 2.0 |

> Each step advances when trajectory is done AND `|pos_error| ≤ pos_deadband_deg`.
> `reset_all` aborts the sequence.

---

## 13 · Sweep Mode

Programmatic sweep: `0 → start → 0 → (start+step) → ... → 0 → stop`.
Generates the full alternating-zero sequence automatically — no array to fill.

| Variable | Type | Description |
|----------|------|-------------|
| `sweep_enabled` | uint8 R/W | Write 1 to start — self-clears when done · write 0 to abort |
| `sweep_start_deg` | float R/W | First non-zero target (deg) — default 5.0 |
| `sweep_stop_deg` | float R/W | Last non-zero target (deg) — default 360.0 |
| `sweep_step_deg` | float R/W | Increment per step (deg) — default 5.0 |
| `sweep_delay_s` | float R/W | Dwell time at each position (s) — default 2.0 |
| `sweep_index` | uint8 R/O | Current step index (0-based) |

> Example: `sweep_start_deg=5`, `sweep_stop_deg=360`, `sweep_step_deg=5` → 144 steps: 0→5→0→10→…→0→360.
> Each step advances when trajectory is done AND `|pos_error| ≤ pos_deadband_deg`.
> `reset_all` aborts the sweep.
