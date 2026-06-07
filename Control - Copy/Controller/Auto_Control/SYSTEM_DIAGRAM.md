# Auto_Control — System Architecture Diagram
`Core/Src/main.c` · `SCURVE.c` · `TRAPEZOID.c` · `ENCODER.c` · `Kalman4.c` · `DistFF.c`

---

## Hardware Overview

```
STM32G474RETx @ 170 MHz  (NUCLEO-G474RE)
│
├── TIM1   QEI encoder input    ARR=65535, TIM_ENCODERMODE_TI12  → 8192 counts/rev
├── TIM3   PWM output (PC9/CH4) Prescaler=169, ARR=999           → 1 kHz PWM
├── TIM6   1 kHz inner ISR      Prescaler=167, ARR=999            (priority 0)
├── TIM7   500 Hz outer ISR     Prescaler=169, ARR=1999           (priority 1)
├── LPUART1 TX                  PA2, 2 000 000 baud              → PC host
└── ADC1 IN2                    PA1, 12-bit single conversion     → current sensor
```

> **IRQ priorities:** TIM6 (0) > TIM7 (1) — TIM6 can preempt TIM7. Intentional so the
> velocity loop always fires on time.

---

## Full Signal Flow

```
╔══════════════════════════════════════════════════════════════════════════════════╗
║  LIVE EXPRESSIONS  (STM32CubeIDE debug — write any value, takes effect in RAM)  ║
║                                                                                  ║
║  traj_target_deg   control_mode   traj_type   start_move   reset_all            ║
║  pid_enabled       zero_encoder   apply_motor_params                             ║
║  motor_dir_inverted               min_pwm_threshold                             ║
║  max_velocity  max_accel  max_jerk                                               ║
║  t1_seg  t2_seg  t_acc_seg  t_cruise_seg  t_dec_seg                              ║
║    S-curve: t_cruise_seg auto-computed (read-only)                               ║
║    Trapezoid: t_cruise_seg user-writable directly                                ║
║  kp_vel  ki_vel  kd_vel   use_kf4_vel                                            ║
║  kp_position  ki_pos  kd_pos   pos_deadband_deg                                 ║
║  motor_J_eq  motor_b_eq  motor_N  motor_Kt  motor_Ke  motor_L  motor_R_arm      ║
║  refff_enabled    V_supply                                                       ║
║  distff_enabled   distff_tau   [g_distff_pwm R/O]                               ║
║  kf4_sigma_v2   kf4_r_theta                                                     ║
║  [g_kf4_position R/O]  [g_kf4_velocity R/O]                                    ║
║  [g_kf4_current R/O]   [g_kf4_tau_d_obs R/O]                                   ║
║  seq_targets[0..8]   seq_count   seq_enabled   seq_index   seq_step_delay        ║
║  sweep_enabled  sweep_start_deg  sweep_stop_deg  sweep_step_deg  sweep_delay_s  ║
║  [sweep_index R/O]                                                               ║
║  kf_cal_pwm   kf_sine_enabled   kf_sine_amp   kf_sine_freq   kf_sine_dir        ║
║  current_zero_counts   current_counts_per_amp                                   ║
║  [g_current_A R/O]   [adc_raw_current R/O]                                      ║
╚══════════════════════════════════════════════════════════════════════════════════╝
                │
                │  traj_target_deg changed   OR   start_move=1   OR   reset_all=1
                ▼
┌──────────────────────────────────────────────────────────────────────────────────┐
│  TIM7 ISR  @  500 Hz   (POS_DT = 0.002 s)                          main.c      │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │  Housekeeping (every tick before PID)                                   │    │
│  │  reset_all      → re-init all modules, zero encoder, reset PIDs        │    │
│  │  apply_motor_params → re-init RefFF, Kalman4, DistFF (position kept)   │    │
│  │  zero_encoder   → reset TIM1 counter + encoder struct + Kalman4        │    │
│  │  Mode/traj-type change → reset PIDs, Start_Trajectory                  │    │
│  │  seq/sweep state machine → advance targets, detect settle              │    │
│  │  traj_target_deg change → Start_Trajectory(disp)                       │    │
│  │  start_move trigger → Start_Trajectory(disp)                           │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │  Start_Trajectory(displacement)                                         │    │
│  │                                                                         │    │
│  │  traj_start_pos = my_encoder.position_rad   (absolute reference)        │    │
│  │  displacement   = traj_target_deg×DEG_TO_RAD − my_encoder.position_rad  │    │
│  │                                                                         │    │
│  │  traj_type == TRAJ_SCURVE  (0)                                          │    │
│  │    t_cruise = |d|/max_velocity − 2×t1_seg − t2_seg  (auto-computed)    │    │
│  │    t_cruise_seg = t_cruise  (overwritten — read-only for user)          │    │
│  │    SCurve_SetTarget_ByTime(&my_scurve, d, t1_seg, t2_seg, t_cruise)    │    │
│  │    Segments: [+j]–[0j]–[−j]–[cruise]–[−j]–[0j]–[+j]                  │    │
│  │                                                                         │    │
│  │  traj_type == TRAJ_TRAPEZOID  (1)                                       │    │
│  │    t_acc_seg, t_cruise_seg, t_dec_seg — used directly (user-written)    │    │
│  │    v_peak = |d| / (t_acc/2 + t_cruise + t_dec/2) — back-computed       │    │
│  │    v_peak clamped to max_velocity if exceeded; segment times unchanged  │    │
│  │    Trapezoid_SetTarget_ByTime(...)                                       │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  pid_enabled == 0?  →  vel_command=0, pwm_command=0, hard_stop_active=0, return │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  switch(control_mode)                                                    │   │
│  │                                                                           │   │
│  │  MODE 3  CASCADE_CTRL  ─────────────────────────────────────────         │   │
│  │    Reads trajectory globals written by TIM6:                             │   │
│  │      g_traj_pos (rad), g_smooth_vel (rad/s), g_traj_active              │   │
│  │    pos_error = g_traj_pos − my_encoder.position_rad                     │   │
│  │    vel_correct = Position_PID_Controller(pos_error,                      │   │
│  │                                          my_encoder.velocity_rad_s)     │   │
│  │    vel_correct clamped ± max_velocity                                   │   │
│  │    vel_cmd = g_smooth_vel + vel_correct                                  │   │
│  │    hard-stop check: !g_traj_active && ss_reached                        │   │
│  │      YES → vel_command = 0, hard_stop_active = 1  (TIM6 holds zero)    │   │
│  │      NO  → vel_command = vel_cmd, hard_stop_active = 0  (non-sticky)   │   │
│  │                                                                          │   │
│  │  MODE 7  KALMAN4_CAL  ──────────────────────────────────────────         │   │
│  │    kf_sine_enabled: generate sine excitation from kf_sine_amp/freq      │   │
│  │    pwm_command = kf_cal_pwm ± sine  (clamped ±100 %)                   │   │
│  │    vel_command = 0, hard_stop_active = 0                                │   │
│  │    g_traj_pos/vel/accel/jerk cleared to 0                               │   │
│  │      (TIM6 repurposes telemetry slots for Kalman4 state comparison)     │   │
│  │                                                                          │   │
│  │  default → vel_command=0, pwm_command=0, hard_stop_active=1             │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────────────┘
                    │                              │
              vel_command                    pwm_command          hard_stop_active
         (volatile float)               (volatile float)          (volatile uint8_t)
         Mode 3                          Mode 7
                    │                              │
                    └──────────── volatile inter-loop interface ──────────────────┘
                                  (TIM7 writes → TIM6 reads)
                    │
┌──────────────────────────────────────────────────────────────────────────────────┐
│  TIM6 ISR  @  1000 Hz   (LOOP_DT = 0.001 s)                        main.c      │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  Encoder_Update(&my_encoder, __HAL_TIM_GET_COUNTER(&htim1))             │   │
│  │   delta = (int16_t)(current − last)   ← handles 16-bit wrap-around      │   │
│  │   position_rad  += delta × (2π / 8192)                                  │   │
│  │   velocity_rad_s = Σ(delta[k..k-9]) × (2π / 8192) / (10 × LOOP_DT)    │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  ADC read (PA1 / ADC1_IN2):                                             │   │
│  │    HAL_ADC_Stop → HAL_ADC_Start → spin on ISR_EOC (~300 ns)            │   │
│  │    adc_raw_current = DR & 0x0FFF                                        │   │
│  │    g_current_A = (adc_raw_current − current_zero_counts)                │   │
│  │                / current_counts_per_amp                                  │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  Kalman4_Update(&my_kf4, position_rad, motor_voltage)                   │   │
│  │  States: x = [θ, ω, i, τ_d]ᵀ    Runs every tick in all modes           │   │
│  │  Q rebuilt each tick (2nd-order expansion: kf4_sigma_v2, motor params)  │   │
│  │  R_data[0] = kf4_r_theta                                                │   │
│  │  g_kf4_position  = x[0]    g_kf4_velocity  = x[1]                      │   │
│  │  g_kf4_current   = x[2]    g_kf4_tau_d_obs = x[3]                      │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  Trajectory Update (Mode 3 only — TIM6 at 1 kHz)                        │   │
│  │    SCurve_Update() OR Trapezoid_Update()                                 │   │
│  │    g_traj_pos, g_smooth_vel, g_smooth_accel, g_jerk, g_traj_active      │   │
│  │    written to volatile globals → TIM7 reads for position PID            │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  hard_stop_active || !pid_enabled  ──YES──► Velocity_PID_Reset()               │
│                                              Set_Motor_PWM(0.0f)                │
│                    │ NO                                                          │
│                    ▼                                                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  switch(control_mode)                                                    │   │
│  │                                                                           │   │
│  │  Mode 3  CASCADE_CTRL:                                                   │   │
│  │    fb_vel = use_kf4_vel ? g_kf4_velocity : encoder.velocity_rad_s      │   │
│  │    vel_error = vel_command − fb_vel                                     │   │
│  │    pwm = Velocity_PID_Controller(vel_error)                             │   │
│  │    if refff_enabled: pwm += RefFF_Update(vel_command)/V_supply×100 %   │   │
│  │    v_ff_d = DistFF_Update(&my_distff, g_kf4_tau_d_obs)                 │   │
│  │    g_distff_pwm = v_ff_d / V_supply × 100 %                            │   │
│  │    if distff_enabled: pwm += g_distff_pwm                              │   │
│  │    Set_Motor_PWM(clamp(pwm, ±max_pwm))                                 │   │
│  │                                                                          │   │
│  │  Mode 7  KALMAN4_CAL:                                                   │   │
│  │    Set_Motor_PWM(pwm_command)  (direct from TIM7)                       │   │
│  │    g_traj_pos = kf4_position, g_smooth_vel = kf4_vel, etc.             │   │
│  │    (repurposes telemetry channels for Kalman4 state comparison)         │   │
│  │                                                                          │   │
│  │  default → Set_Motor_PWM(0)                                             │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │  Set_Motor_PWM(pwm_duty)                                                 │   │
│  │   if (motor_dir_inverted) pwm_duty = −pwm_duty                          │   │
│  │   direction: PC7 (MOTOR_DIR_Pin) ← SET if ≥0, RESET if <0              │   │
│  │   dead-zone: if 0 < |pwm| < min_pwm_threshold → raise to threshold      │   │
│  │   clamp:  |pwm| ≤ 100 %                                                 │   │
│  │   TIM3 CCR4 = (uint32_t)(|pwm|/100 × 1000)                             │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                    │                                                             │
│  Telemetry (every tick, non-blocking):                                          │
│    Pack_Telemetry() → 68-byte packet → tx_pending=1                            │
│    while(1): if tx_pending → HAL_UART_Transmit (blocking, ~272 µs @ 2 Mbaud)  │
└──────────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼  PWM + direction
         ┌──────────────────────┐
         │   DC MOTOR           │  TIM3 CH4, 1 kHz, ARR=999
         │   + GEARBOX          │  PC7 = direction
         └──────────┬───────────┘
                    │ shaft rotation
                    ▼
         ┌──────────────────────┐
         │   QUADRATURE         │  2048 PPR × 4x QEI = 8192 counts/rev
         │   ENCODER (TIM1)     │  ARR=65535, TIM_ENCODERMODE_TI12
         └──────────────────────┘
                    │
                    └── feedback ──► TIM6: Encoder_Update()
```

---

## Kalman Filter

| File | States | F model | Measurements | Status |
|------|--------|---------|--------------|--------|
| `Kalman4.c` | 4 — θ, ω, i, τ_d | Full motor physics ZOH | position only (H=[1,0,0,0]) | **Active — all modes, every TIM6 tick** |
| `Kalman.c` | 2 — position, velocity | Constant-velocity | position only | **Present in project — not wired into main.c** |

**Kalman4 current configuration:**
- H = [1, 0, 0, 0] — position measurement only (1×4)
- R = [kf4_r_theta] — scalar position measurement noise
- F encodes motor physics (J, b, N, Kt, Ke, L, R) with ZOH for current state
- G = [0, 0, Ts/L, 0]ᵀ — voltage noise enters through current channel
- Q: second-order Van Loan expansion — `Qk ≈ Qc·Ts + (A·Qc+Qc·Aᵀ)·Ts²/2 + A·Qc·Aᵀ·Ts³/3`
  - Nonzero entries: Q[2][2], Q[1][2]=Q[2][1], Q[1][1]
  - Recomputed every tick from `kf4_sigma_v2` and motor parameters

ADC current (PA1) is read every tick but **not fed into Kalman4**. Future: expand to `MEAS_DIM=2`.

---

## Position PID (Mode 3 — TIM7)

```
                         Position_PID_Controller(pos_error, vel_feedback)
                         ┌──────────────────────────────────────────────────────────┐
  pos_error (rad) ──────►│ p_out = kp_position × pos_error                         │
                         │                                                           │
  pos_integral += pos_error × POS_DT   (clamped ± max_velocity/ki_pos)             │
                         │ i_out = ki_pos × pos_integral                            │
                         │                                                           │
  vel_feedback = my_encoder.velocity_rad_s ──────────────────────────────────────► │
                         │ d_out = −kd_pos × vel_feedback  (velocity-form D)       │
                         │                                                           │
                         │ return p_out + i_out + d_out   (rad/s)                  │
                         └──────────────────────────────────────────────────────────┘
```

---

## UART Telemetry Packet — 68 bytes @ 2 Mbaud

```
Byte  0– 1 : 0x7E 0x7E         — header
Byte  2– 5 : g_traj_pos         float32  trajectory ideal position    (rad)
Byte  6– 9 : g_smooth_vel       float32  trajectory velocity          (rad/s)
Byte 10–13 : g_smooth_accel     float32  trajectory acceleration      (rad/s²)
Byte 14–17 : g_jerk             float32  trajectory jerk              (rad/s³; 0 for trapezoid)
Byte 18–21 : g_position_rad     float32  actual encoder position      (rad)
Byte 22–25 : g_velocity_rad_s   float32  raw encoder velocity         (rad/s)
Byte 26–29 : g_kf4_velocity     float32  Kalman4 velocity estimate    (rad/s)  ← backwards-compat slot
Byte 30–33 : g_kf4_position     float32  Kalman4 position estimate    (rad)    ← backwards-compat slot
Byte 34–37 : ss_error_rad       float32  steady-state position error  (rad)
Byte 38–41 : g_current_A        float32  measured motor current       (A)
Byte 42–45 : g_kf4_position     float32  Kalman4 position estimate    (rad)   ← dedicated Kalman4 slot
Byte 46–49 : g_kf4_velocity     float32  Kalman4 velocity estimate    (rad/s) ← dedicated Kalman4 slot
Byte 50–53 : g_kf4_current      float32  Kalman4 current estimate     (A)
Byte 54–57 : g_kf4_tau_d_obs    float32  Kalman4 disturbance torque   (N·m)
Byte 58–61 : g_motor_voltage    float32  motor commanded voltage      (V)
Byte 62–65 : adc_raw_current    uint32   raw ADC count                (0–4095)
Byte 66–67 : 0x03 0x03          — footer
```

> Bytes 26–33 now carry Kalman4 data (formerly 2-state Kalman, removed). Bytes 42–49
> duplicate Kalman4 pos/vel in the dedicated Kalman4 slots. Simulink parser checks raw(67)
> and raw(68) for footer (1-based indexing).

Packed in `Pack_Telemetry()` by TIM6 ISR.
Transmitted in `while(1)` via `HAL_UART_Transmit()` (blocking, ~272 µs @ 2 Mbaud).

---

## Mode Summary

| `control_mode` | Name | Trajectory update | Position loop | Velocity loop | PWM source |
|:-:|---|:-:|:-:|:-:|---|
| 3 | CASCADE_CTRL | TIM6 @ 1 kHz | PID → vel_command | PID + RefFF + DistFF | vel_command |
| 7 | KALMAN4_CAL | — | — | — | `kf_cal_pwm` ± sine (±100 %) |
| other | — | — | — | — | 0 (default case) |

Sequence mode (`seq_enabled`) and sweep mode (`sweep_enabled`) both work in Mode 3.
They use TIM7 to manage target changes; trajectory and velocity PID run as normal.

---

## Trajectory Shape Parameters

| Profile | Shape params | Time segments | v_peak formula |
|---|---|---|---|
| S-curve | `t1_seg`, `t2_seg` | +j · 0j · −j · cruise · −j · 0j · +j | `t_cruise` auto = `\|d\|/v_max − 2×t1 − t2` |
| Trapezoid | `t_acc_seg`, `t_cruise_seg`, `t_dec_seg` | accel · cruise · decel | `\|d\| / (t_acc/2 + t_cruise + t_dec/2)` |

**S-curve:** `t_cruise_seg` is read-only auto-computed. `max_velocity` determines cruise time.
**Trapezoid:** all three segment times are user-writable. `v_peak` is back-computed. `max_velocity` is a safety clamp only.
