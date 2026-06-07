# Cascade Control Block Diagram — Mode 3 (MODE_CASCADE_CTRL)
`Core/Src/main.c`

---

```
  traj_target_deg  (Live Expressions — change triggers auto Start_Trajectory)
  start_move = 1   (manual retrigger)
                                   │
                                   ▼  TIM7 calls Start_Trajectory(disp) to arm profile
╔══════════════════════════════════════════════════════════════════════════════════╗
║  TIM6  @  1000 Hz   INNER LOOP   LOOP_DT = 0.001 s    (priority 0)            ║
║                                                                                  ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  ENCODER UPDATE   Encoder_Update()                                       │  ║
║   │   position_rad  = count_accum × 2π / 8192                               │  ║
║   │   velocity_rad_s = windowed delta × 2π / 8192 / LOOP_DT                 │  ║
║   └───────────┬──────────────────────────────────────────────────────────────┘  ║
║               │ position_rad, velocity_rad_s                                    ║
║               ▼                                                                  ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  ADC READ   PA1 / ADC1_IN2                                               │  ║
║   │   HAL_ADC_Stop → HAL_ADC_Start → spin on ISR_EOC                        │  ║
║   │   adc_raw_current = DR & 0x0FFF                                          │  ║
║   │   g_current_A = (adc_raw_current − current_zero_counts)                  │  ║
║   │               / current_counts_per_amp                                   │  ║
║   └──────────────────────────────────────────────────────────────────────────┘  ║
║               │                                                                  ║
║               ▼                                                                  ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  KALMAN4   Kalman4_Update(&my_kf4, position_rad, motor_voltage)          │  ║
║   │                                                                          │  ║
║   │  States: x = [θ, ω, i, τ_d]ᵀ                                            │  ║
║   │  F: full motor ZOH physics (J, b, N, Kt, Ke, L, R)                      │  ║
║   │  Q: second-order expansion of Qc·Ts + (A·Qc+Qc·Aᵀ)·Ts²/2 + A·Qc·Aᵀ·Ts³/3  ║
║   │     σ_v² enters only through current channel (G = [0,0,Ts/L,0]ᵀ)        │  ║
║   │     Nonzero: Q[2][2], Q[1][2]=Q[2][1], Q[1][1] (from motor dynamics)    │  ║
║   │  Tune: kf4_sigma_v2 (process noise), kf4_r_theta (meas noise)           │  ║
║   │                                                                          │  ║
║   │  g_kf4_position  = x[0]   (rad)                                         │  ║
║   │  g_kf4_velocity  = x[1]   (rad/s)                                       │  ║
║   │  g_kf4_current   = x[2]   (A)   — model-only, no sensor feedback        │  ║
║   │  g_kf4_tau_d_obs = x[3]   (N·m) — estimated disturbance torque          │  ║
║   └──────────────────────────────────────────────────────────────────────────┘  ║
║               │                                                                  ║
║               ▼                                                                  ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  TRAJECTORY UPDATE   (Mode 3 only, every TIM6 tick)                      │  ║
║   │                                                                          │  ║
║   │  SCurve_Update(&my_scurve)   OR   Trapezoid_Update(&my_trapezoid)        │  ║
║   │   — profile was armed by TIM7 calling Start_Trajectory()                 │  ║
║   │                                                                          │  ║
║   │  g_traj_pos    = traj_start_pos + p_current      (volatile, rad)        │  ║
║   │  g_smooth_vel  = v_current                        (volatile, rad/s)     │  ║
║   │  g_smooth_accel = a_current                       (volatile, rad/s²)    │  ║
║   │  g_jerk        = j_current  (0 for trapezoid)    (volatile, rad/s³)     │  ║
║   │  g_traj_active = is_active                        (volatile, uint8)     │  ║
║   └──────────────────────────────────────────────────────────────────────────┘  ║
║               │                                                                  ║
║   hard_stop_active || !pid_enabled  ──YES──► Velocity_PID_Reset()              ║
║                                               Set_Motor_PWM(0)                  ║
║               │ NO                                                               ║
║               ▼ vel_command (written by TIM7, read here)                        ║
║          ┌───┴───┐                                                              ║
║          │       │  fb_vel = use_kf4_vel==1 ? g_kf4_velocity                    ║
║          │   Σ   │                          : my_encoder.velocity_rad_s         ║
║          │ + │ − │◄─── fb_vel  (source: use_kf4_vel 0=encoder · 1=Kalman4)      ║
║          └───┬───┘  vel_error = vel_command − fb_vel  → vel_error_live          ║
║              │                                                                   ║
║              ▼ vel_error (rad/s)                                                 ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  VELOCITY PID   Velocity_PID_Controller()                                │  ║
║   │                                                                          │  ║
║   │   P  =  kp_vel × vel_error                                               │  ║
║   │                                                                          │  ║
║   │   vel_integral += vel_error × LOOP_DT                                   │  ║
║   │   anti-windup:  clamp vel_integral to ± max_pwm / ki_vel                │  ║
║   │   I  =  ki_vel × vel_integral    → i_term_live (Live Expr)              │  ║
║   │                                                                          │  ║
║   │   D  =  kd_vel × (vel_error − vel_prev_error) / LOOP_DT                │  ║
║   │                                                                          │  ║
║   │   output = P + I + D   clamped ± max_pwm (%)  →  pwm_out               │  ║
║   └──────────────────────────────┬───────────────────────────────────────────┘  ║
║                                  │ pwm_out (%)                                   ║
║                                  │                                               ║
║   ┌──────────────────────────────┴───────────────────────────────────────────┐  ║
║   │  REFERENCE FEEDFORWARD   RefFF_Update()   [REF_FEEDFORWARD.c]            │  ║
║   │                  (only when refff_enabled = 1)                            │  ║
║   │                                                                           │  ║
║   │   G_ff(s) = [J·L·s² + (J·R+b·L)·s + (b·R+N²·Kt·Ke)] / [N·Kt·(τ·s+1)²]│  ║
║   │   input : vel_command (rad/s)   output: V_ff (V)                         │  ║
║   │   pwm_ff = V_ff / V_supply × 100 %                                       │  ║
║   │   pwm_out += pwm_ff                                                       │  ║
║   └──────────────────────────────┬───────────────────────────────────────────┘  ║
║                                  │ pwm_out (%)                                   ║
║                                  │                                               ║
║   ┌──────────────────────────────┴───────────────────────────────────────────┐  ║
║   │  DISTURBANCE FEEDFORWARD   DistFF_Update()   [DistFF.c]                  │  ║
║   │                  (computed always; added when distff_enabled = 1)         │  ║
║   │                                                                           │  ║
║   │   G_dff(s) = (L·s + R) / (N·Kt·(τ·s + 1))                              │  ║
║   │   input : g_kf4_tau_d_obs (N·m)   output: v_ff_d (V)                    │  ║
║   │   g_distff_pwm = v_ff_d / V_supply × 100 %   (observe before enabling)  │  ║
║   │   if distff_enabled: pwm_out += g_distff_pwm                             │  ║
║   └──────────────────────────────┬───────────────────────────────────────────┘  ║
║                                  │ pwm_out (%) clamped ± max_pwm                 ║
║                                  ▼                                               ║
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║
║   │  Set_Motor_PWM(pwm_out)                                                  │  ║
║   │                                                                          │  ║
║   │  1. motor_dir_inverted → pwm_out = −pwm_out                             │  ║
║   │  2. sign → PC7 (MOTOR_DIR_Pin): SET if ≥ 0,  RESET if < 0              │  ║
║   │  3. dead-zone: 0 < |pwm| < min_pwm_threshold → raise to threshold       │  ║
║   │  4. clamp: |pwm| ≤ 100 %                                                │  ║
║   │  5. TIM3 CCR4 = (uint32_t)(|pwm| / 100 × 1000)    ARR = 999           │  ║
║   └──────────────────────────────────────────────────────────────────────────┘  ║
║               │                                                                  ║
║   Pack_Telemetry() every tick → tx_pending=1 → while(1) transmits 68 bytes     ║
╚══════════════════════════════════════════════════════════════════════════════════╝
               │ PWM voltage
               │  ┌─────────────────────────────────────────────────┐
               │  │  volatile inter-loop interface                   │
               │  │  TIM6 writes:  g_traj_pos, g_smooth_vel,        │
               │  │                g_smooth_accel, g_jerk,           │
               │  │                g_traj_active  (trajectory state) │
               │  │  TIM7 writes:  vel_command, hard_stop_active     │
               │  │  TIM6 reads:   vel_command, hard_stop_active     │
               │  │  TIM7 reads:   g_traj_pos, g_smooth_vel,        │
               │  │                g_traj_active, g_kf4_tau_d_obs   │
               │  └─────────────────────────────────────────────────┘
               │
╔══════════════════════════════════════════════════════════════════════════════════╗
║  TIM7  @  500 Hz   OUTER POSITION LOOP   POS_DT = 0.002 s   (priority 1)      ║
║                                                                                  ║
║   Controls: Start_Trajectory() arm/re-arm, sequence, sweep, pos PID             ║
║   Reads trajectory globals written by TIM6 (g_traj_pos, g_smooth_vel, etc.)    ║
║                                                                                  ║
║              g_traj_pos  (ideal position, rad — written by TIM6 @ 1 kHz)       ║
║              g_traj_active (1 while moving, 0 when done)                        ║
║              │                                                                   ║
║         ┌───┴───┐  pos_error = g_traj_pos − my_encoder.position_rad             ║
║         │   Σ   │                                                                ║
║         │ + │ − │◄──────────────────── my_encoder.position_rad (encoder) ───────╫──┐
║         └───┬───┘                                                                ║  │
║             │                                                                    ║  │
║             │ pos_error (rad)  → ss_error_rad / ss_error_deg (Live Expr)        ║  │
║             │                                                                    ║  │
║   ┌──────────────────────────────────────────────────────────────────────────┐  ║  │
║   │  POSITION PID   Position_PID_Controller()                                │  ║  │
║   │                                                                          │  ║  │
║   │   P  =  kp_position × pos_error                                         │  ║  │
║   │                                                                          │  ║  │
║   │   pos_integral += pos_error × POS_DT                                    │  ║  │
║   │   anti-windup:  clamp pos_integral to ± max_velocity / ki_pos           │  ║  │
║   │   I  =  ki_pos × pos_integral   → pos_integral_live (Live Expr)         │  ║  │
║   │                                                                          │  ║  │
║   │   D  =  −kd_pos × my_encoder.velocity_rad_s  (velocity-form D)         │◄─╫──┤
║   │                                                                          │  ║  │
║   │   output = P + I + D   →  vel_correct (rad/s)                           │  ║  │
║   └───────────────────────────────────┬──────────────────────────────────────┘  ║  │
║                                       │ vel_correct  (clamped ± max_velocity)   ║  │
║                                       │                                          ║  │
║                              ┌────────┴──────┐                                  ║  │
║                              │       Σ       │◄─── g_smooth_vel (feedforward)   ║  │
║                              │  +vel_correct │     (written by TIM6 traj)       ║  │
║                              │  +g_smooth_vel│                                  ║  │
║                              └───────┬───────┘                                  ║  │
║                                      │ vel_cmd = g_smooth_vel + vel_correct      ║  │
║                                      │ clamped ± max_velocity                   ║  │
║                                      │                                           ║  │
║                           ┌──────────┴────────────────────────────────────┐    ║  │
║                           │  HARD-STOP / SETTLE CHECK                      │    ║  │
║                           │                                                │    ║  │
║                           │  !g_traj_active                                │    ║  │
║                           │  && ss_reached (|pos_error| ≤ pos_deadband_deg)│    ║  │
║                           │                                                │    ║  │
║                           │  YES → vel_command = 0                         │    ║  │
║                           │         hard_stop_active = 1                   │    ║  │
║                           │         ss_final_error_deg = ss_error_deg      │    ║  │
║                           │                                                │    ║  │
║                           │  NO  → vel_command = vel_cmd                   │    ║  │
║                           │         hard_stop_active = 0   (non-sticky)    │    ║  │
║                           └──────────────────┬─────────────────────────────┘   ║  │
╚══════════════════════════════════════════════╪══════════════════════════════════╝  │
                                               │ vel_command (volatile)              │
                                               ▼                                     │
                              ─── back to TIM6 inner loop ───                        │
                                                                                     │
                    ┌──────────────────────────────┐                                │
                    │       DC MOTOR + GEARBOX      │   TIM3 CH4 PWM @ 1 kHz        │
                    │                               │   PC7 = direction              │
                    └───────────────┬───────────────┘                               │
                                    │ θ  (shaft angle)                              │
                                    ▼                                               │
                    ┌──────────────────────────────┐                                │
                    │    QUADRATURE ENCODER (TIM1)  │   8192 counts/rev             │
                    │    Encoder_Update()           │   ARR=65535, TIM_ENCODERMODE_TI12│
                    │    pos += delta × 2π/8192     │                                │
                    │    vel  = Σ(delta[k..k-9])    │                                │
                    │          × 2π/8192/LOOP_DT    │                                │
                    └──────────┬────────────────────┘                               │
                               │                                                     │
              ┌────────────────┴────────────────────────────────┐                   │
              │                                                  │                   │
        position_rad  ──► Kalman4_Update()            velocity_rad_s                │
        (TIM6 every tick)   (& TIM7 via encoder struct)                             │
              │                                                  │                   │
              └──────────────────────────────────────────────────┼───────────────────┘
                                                                  └───────────────────┘
```

---

## Signal Summary

| Signal | Variable | Units | Written by | Read by |
|--------|----------|-------|------------|---------|
| Trajectory ideal position | `g_traj_pos` | rad | TIM6 traj update | TIM7 Σ |
| Trajectory velocity | `g_smooth_vel` | rad/s | TIM6 traj update | TIM7 Σ |
| Trajectory acceleration | `g_smooth_accel` | rad/s² | TIM6 traj update | TIM7 |
| Trajectory jerk | `g_jerk` | rad/s³ | TIM6 traj update | UART only |
| Trajectory active flag | `g_traj_active` | — | TIM6 traj update | TIM7 hard-stop |
| Position error | `pos_error` → `ss_error_rad` | rad | TIM7 | TIM7 PID |
| Velocity correction | `vel_correct` | rad/s | TIM7 pos PID | TIM7 Σ |
| **Velocity setpoint** | **`vel_command`** | rad/s | **TIM7** | **TIM6** |
| **Hard-stop flag** | **`hard_stop_active`** | — | **TIM7** | **TIM6** |
| Kalman4 disturbance | `g_kf4_tau_d_obs` | N·m | TIM6 Kalman4 | TIM6 DistFF |
| DistFF addend | `g_distff_pwm` | % | TIM6 DistFF | observable |
| Velocity error | `vel_error` → `vel_error_live` | rad/s | TIM6 (fb_vel source = `use_kf4_vel`) | TIM6 vel PID |
| Feedforward PWM | `pwm_ff` | % | TIM6 RefFF | TIM6 Σ→PWM |
| PWM output | `pwm_out` / `g_pwm_duty` | % | TIM6 | `Set_Motor_PWM` |
| Actual position | `my_encoder.position_rad` | rad | TIM6 encoder | TIM7 Σ, Kalman4 |
| Actual velocity | `my_encoder.velocity_rad_s` | rad/s | TIM6 encoder | TIM7 pos D, TIM6 vel Σ |

---

## Tunable Gains (Live Expressions)

| Gain | Variable | Loop | Output units | Default |
|------|----------|------|-------------|---------|
| Position P | `kp_position` | Outer (TIM7) | rad/s per rad | 1.0 |
| Position I | `ki_pos` | Outer (TIM7) | rad/s per rad·s | 0.0 |
| Position D | `kd_pos` | Outer (TIM7) | rad/s per rad/s | 0.0 |
| Velocity P | `kp_vel` | Inner (TIM6) | % per rad/s | 30.0 |
| Velocity I | `ki_vel` | Inner (TIM6) | % per rad | 0.1 |
| Velocity D | `kd_vel` | Inner (TIM6) | % per rad/s² | 0.0 |
| Dead zone | `min_pwm_threshold` | Inner (TIM6) | % | 5.0 |
| Vel source | `use_kf4_vel` | Inner (TIM6) + kd_pos | — | 0 |
| Settle deadband | `pos_deadband_deg` | Hard-stop check | deg | 2.0 |

> `pos_deadband_deg` determines the hard-stop settle condition
> (`!g_traj_active && ss_reached`). Set ≥ gear backlash amplitude to prevent
> DistFF oscillation in the backlash range.

---

## Kalman4 Parameters (Live Expressions)

| Variable | Role | Default | Effect |
|----------|------|---------|--------|
| `kf4_sigma_v2` | σ_v² — voltage noise variance driving Q | 1e-3 | Higher → faster ω/τ_d response, noisier |
| `kf4_r_theta` | R[0] — position measurement noise variance | 1e-6 | Higher → smoother estimate, more lag |
| `g_kf4_velocity` | Read-only Kalman4 velocity output | — | Compare with `g_velocity_rad_s` |
| `g_kf4_position` | Read-only Kalman4 position output | — | Compare with `g_position_rad` |
| `g_kf4_current` | Read-only Kalman4 current estimate | — | Model-driven, no current sensor feedback |
| `g_kf4_tau_d_obs` | Read-only disturbance torque estimate | — | Feeds DistFF; observe in Mode 7 |

> After changing `kf4_sigma_v2` or `kf4_r_theta`, write `apply_motor_params = 1`
> or `reset_all = 1` to re-initialise the filter with the new noise parameters.

---

## Reference Feedforward Parameters (Live Expressions)

| Variable | Role | Default |
|----------|------|---------|
| `refff_enabled` | 0 = off, 1 = active | 0 |
| `V_supply` | Motor bus voltage (V) | 24.0 |

---

## Disturbance Feedforward Parameters (Live Expressions)

| Variable | Role | Default |
|----------|------|---------|
| `distff_enabled` | 0 = off (observe g_distff_pwm only), 1 = active | 1 |
| `distff_tau` | Filter time constant (s) — apply with `apply_motor_params = 1` | 0.02 |
| `g_distff_pwm` | Read-only DistFF PWM addend (%) | — |

> Check `g_distff_pwm` magnitude before enabling. If DistFF causes oscillation
> near zero speed (gear backlash), set `pos_deadband_deg` ≥ backlash amplitude.

---

## Anti-windup Limits

| Integrator | Variable | Clamped to |
|-----------|----------|-----------|
| Position I | `pos_integral` | `± max_velocity / ki_pos` |
| Velocity I | `vel_integral` | `± max_pwm / ki_vel` |
