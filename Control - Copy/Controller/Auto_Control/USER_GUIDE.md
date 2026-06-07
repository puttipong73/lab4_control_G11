# Auto_Control — User Guide
**STM32G474RE Motor Controller with S-Curve & Trapezoidal Trajectory Profiles**

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Hardware Setup](#2-hardware-setup)
3. [Software Requirements](#3-software-requirements)
4. [Quick Start](#4-quick-start)
5. [Control Modes](#5-control-modes)
6. [Trajectory Profiles](#6-trajectory-profiles)
7. [Live Expressions — Runtime Control](#7-live-expressions--runtime-control)
   - [Master Control](#master-control)
   - [Sequence Mode](#sequence-mode)
   - [Trajectory Target & Profile](#trajectory-target--profile)
   - [Position PID](#position-pid--tim7-outer-loop-500-hz)
   - [Velocity PID](#velocity-pid--tim6-inner-loop-1000-hz)
   - [Kalman Filter](#kalman-filter-1)
   - [Kalman4 Calibration (Mode 7)](#kalman4-calibration-mode-7)
   - [Current Sensor — ADC1 IN2 (PA1)](#current-sensor--adc1-in2-pa1)
   - [Reference Feedforward](#reference-feedforward-1)
   - [Motor Setup](#motor-setup)
   - [Observables — Read Only](#observables--read-only)
   - [Encoder Test — Mode 4 Only](#encoder-test--mode-4-only)
8. [UART Telemetry & Simulink](#8-uart-telemetry--simulink)
9. [Kalman Filter](#9-kalman-filter)
   - [9a. Kalman Filter — 4-State Motor Model (Kalman4)](#9a-kalman-filter--4-state-motor-model-kalman4)
10. [Reference Feedforward](#10-reference-feedforward)
11. [Parameter Tuning](#11-parameter-tuning)
12. [Time-Based Segment Control](#12-time-based-segment-control)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. Project Overview

This firmware implements a real-time cascade motor motion controller on the **STM32G474RE** microcontroller. It provides two velocity profile trajectory generators, a 4-state motor Kalman filter (Kalman4), model-based reference feedforward, disturbance feedforward, and sweep/sequence automation — all tunable live during a debug session without reflashing.

### Key Features

| Feature | Detail |
|---------|--------|
| Inner loop (velocity) | 1 kHz (TIM6, 1 ms) — encoder + Kalman4 + trajectory update + vel PID + RefFF + DistFF |
| Outer loop (position) | 500 Hz (TIM7, 2 ms) — position PID + sequence/sweep control |
| Trajectory profiles | S-Curve (7-segment, jerk-limited) and Trapezoidal (3-segment) |
| Active modes | **Mode 3** (full cascade) and **Mode 7** (Kalman4 calibration) |
| Sequence mode | 9-target array sequencer with configurable inter-step delay |
| Sweep mode | Programmatic 0→T→0→… sweep — generates targets automatically |
| Kalman4 filter | 4-state motor model [θ, ω, i, τ_d] — position measurement, physics-based |
| DistFF | Disturbance feedforward from Kalman4 τ_d → compensating voltage |
| Reference feedforward | Model-based Tustin-discretised feed-forward voltage term |
| Current sensor | ADC1 IN2 (PA1) — 12-bit current reading with conversion to Amperes |
| Telemetry | **72-byte** binary packet at 2 Mbaud over LPUART1 → Simulink |
| Encoder | Quadrature, 2048 PPR × 4x = 8192 counts/rev, 10-tick windowed velocity |

### System Block Diagram

```
 traj_target_deg (change auto-triggers Start_Trajectory)
                      │
                      ▼  TIM7 @ 500 Hz arms profile
             ┌─────────────────────────────────────────┐
             │  TIM6 @ 1000 Hz — trajectory Update     │
             │  SCurve_Update / Trapezoid_Update        │
             │  → g_traj_pos, g_smooth_vel (volatile)  │
             └──────────────────┬──────────────────────┘
                                │ TIM7 reads g_traj_pos
                                ▼
             ┌─────────────────────────────────────────┐
             │  TIM7 @ 500 Hz — Position PID           │
             │  pos_error = g_traj_pos − act_pos        │
             │  vel_correct = PID(pos_error)            │
             │  vel_command = g_smooth_vel + vel_correct│
             └──────────────────┬──────────────────────┘
                                │ vel_command (volatile)
                                ▼
             ┌─────────────────────────────────────────┐   TIM6 @ 1000 Hz
             │  Velocity PID                           │   (inner velocity loop)
             │  fb_vel = use_kf4_vel ? kf4_vel         │
             │                       : encoder.vel     │
             │  vel_error = vel_command − fb_vel        │
             │  kp_vel, ki_vel, kd_vel                  │
             └──────────────────┬──────────────────────┘
                                │ pwm_out
                                ▼
             ┌─────────────────────────────────────────┐
             │  + RefFF (if refff_enabled)             │
             │  + DistFF (if distff_enabled)           │
             │    uses Kalman4 τ_d estimate             │
             └──────────────────┬──────────────────────┘
                                │ total PWM
                                ▼
                          [ MOTOR + ENCODER ]
                                │
                      UART telemetry → Simulink
                      72 bytes @ 2 Mbaud
```

---

## 2. Hardware Setup

### Required Components

| Component | Specification |
|-----------|--------------|
| MCU Board | NUCLEO-G474RE |
| Motor driver | PWM + direction input (e.g. L298N, DRV8833) |
| DC Motor | With quadrature encoder, 2048 PPR |
| PC connection | USB-A to Mini-B (ST-LINK, also provides virtual COM port) |

### Pin Connections

| STM32 Pin | Signal | Connect to |
|-----------|--------|-----------|
| PA0 | `MOTOR_DIR` | Motor driver direction input |
| PA1 (ADC1_IN2) | Current sensor | Analog current sense (0–3.3 V) |
| PA8 (TIM1_CH1) | Encoder channel A | Encoder A output |
| PA9 (TIM1_CH2) | Encoder channel B | Encoder B output |
| PC9 (TIM3_CH4) | PWM output | Motor driver PWM input |
| PA2 (LPUART1_TX) | UART TX | ST-LINK (virtual COM — no wiring needed) |
| PA3 (LPUART1_RX) | UART RX | ST-LINK (virtual COM — no wiring needed) |
| PC13 | User button B1 | Onboard (no wiring needed) |
| PA5 | LED LD2 | Onboard (no wiring needed) |

> **Note:** LPUART1 TX/RX are routed through the ST-LINK chip on the NUCLEO board. You only need the USB cable — no separate USB-serial adapter required.

---

## 3. Software Requirements

| Software | Version / Notes |
|----------|----------------|
| STM32CubeIDE | 1.13 or later |
| Simulink (optional) | For data visualization via UART |
| Instrument Control Toolbox | Required for Simulink serial receive |

---

## 4. Quick Start

### Step 1 — Open the project
```
File → Open Projects from File System
→ Select: C:\Studio_2.68_G11\Control_by_due\Control\Controller\Auto_Control
```

### Step 2 — Build
```
Project → Build All   (Ctrl + B)
```
Expected: 0 errors, 0 warnings.

### Step 3 — Flash and debug
```
Run → Debug As → STM32 Cortex-M C/C++ Application   (F11)
```
The debugger will halt at `main()`. Press **Resume** (F8) to start the firmware.

### Step 4 — Open Live Expressions
```
Window → Show View → Live Expressions
```
The Live Expressions panel is pre-populated from the `.launch` file. All variables below should appear automatically when debugging starts.

### Step 5 — Run your first trajectory test
1. Confirm `control_mode = 1` (trajectory test mode)
2. Confirm `traj_type = 0` (S-Curve)
3. Set `traj_target_deg = 180` and watch `g_smooth_vel` on the Live Expressions — the trajectory runs automatically

---

## 5. Control Modes

Change `control_mode` in Live Expressions. The firmware resets all state and restarts cleanly on every mode change.

---

### Mode 1 — Trajectory Test (`control_mode = 1`)

**Purpose:** Observe the raw shape of the velocity profile with no closed-loop feedback.

```
Trajectory Generator ──► v_current ──► PWM (open-loop, proportional to velocity)
Encoder reads velocity (telemetry only, not fed back)
```

**When to use:** First test after changing trajectory parameters, or when comparing S-curve vs trapezoidal shape.

---

### Mode 2 — Trajectory + Position PD (`control_mode = 2`)

**Purpose:** Trajectory generates ideal position; position error drives PWM directly.

```
Trajectory → p_current → pos_error = ideal_pos − actual_pos → kp_pos_pwm × pos_error → PWM
```

**When to use:** Step-response testing without velocity PID.

---

### Mode 3 — Full Cascade Control (`control_mode = 3`)

**Purpose:** Full cascade — trajectory + position PID outer loop + velocity PID inner loop.

```
Trajectory → v_current, p_current
  └─ Position PID → vel_correct
       └─ vel_command = v_current + vel_correct → Velocity PID → PWM
```

**When to use:** Normal production operation and final validation.

---

### Mode 4 — Encoder Test (`control_mode = 4`)

**Purpose:** Motor off — observe encoder position and velocity in Live Expressions and telemetry.

```
Motor PWM = 0 — encoder values visible in enc_position_deg, enc_velocity_rpm
```

**When to use:** Verifying encoder wiring and direction before running control.

---

### Mode 5 — Position+Velocity Control, No Trajectory (`control_mode = 5`)

**Purpose:** Cascade position→velocity PID aimed directly at `traj_target_deg`, no trajectory generator.

```
traj_target_deg → pos_error → Position PID → vel_command → Velocity PID → PWM
```

**When to use:** Comparing step response with vs without trajectory smoothing.

---

### Mode 6 — Direct Position PID → PWM (`control_mode = 6`)

**Purpose:** Position error drives PWM directly (no velocity PID, no trajectory).

```
traj_target_deg → pos_error → kp_pos_pwm × error + ki_pos_pwm × integral − kd_pos_pwm × vel → PWM
```

**When to use:** Simple position hold with integral action.

---

### Mode 7 — Kalman4 Calibration (`control_mode = 7`)

**Purpose:** Drive the motor with a known open-loop PWM while observing Kalman4 estimates to tune the filter noise parameters.

```
kf_cal_pwm (Live Expressions -100..+100 %) → Set_Motor_PWM directly
Kalman4_Update(encoder position, estimated voltage) → g_kf4_velocity, g_kf4_current, g_kf4_tau_d_obs
```

**UART packet channels (Mode 7):**
- Bytes 6–9 → Kalman4 velocity estimate
- Bytes 10–13 → Kalman4 current estimate (model-only)
- Bytes 14–17 → Kalman4 disturbance torque estimate
- Bytes 22–25 → actual raw encoder velocity
- Bytes 26–29 → Kalman4 velocity (same, in standard Kalman slot)

**Calibration workflow:**
1. Set `control_mode = 7`
2. Set `kf_cal_pwm` to a step or ramp value (e.g. 20–40 %)
3. In Simulink scope: compare bytes 22–25 (actual velocity) vs 26–29 (Kalman4 velocity)
4. Increase `kf4_q_vel` if Kalman4 lags; increase `kf4_r_theta` if it is too noisy
5. Write `reset_all = 1` to reinitialise with new noise values — self-clears
6. Repeat until tracks well, then use `g_kf4_velocity` as velocity feedback in other modes

**When to use:** Before switching to a mode that uses Kalman4 as velocity feedback.

---

## 6. Trajectory Profiles

Change `traj_type` in Live Expressions at any time.

---

### Trapezoidal Profile (`traj_type = 1`) — 3 segments

```
Velocity
v_peak │      ___________
       │     /           \
       │    /             \
     0 │___/_______________\___
       │  Seg1   Seg2   Seg3
       │  Accel  Cruise  Decel
```

| Segment | Duration | Acceleration | Velocity |
|---------|----------|-------------|---------|
| 1 — Accel | `t_acc_seg` | `+a_acc` | `0 → v_peak` |
| 2 — Cruise | `t_cruise_seg` | `0` | `v_peak` (constant) |
| 3 — Decel | `t_dec_seg` | `−a_dec` | `v_peak → 0` |

**Asymmetric area identity:** `displacement = v_peak × (t_acc/2 + t_cruise + t_dec/2)`

`a_acc = v_peak / t_acc`,  `a_dec = v_peak / t_dec`  — may differ if `t_acc ≠ t_dec`

**v_peak back-computed:** `v_peak = |displacement| / (t_acc/2 + t_cruise + t_dec/2)`
`max_velocity` is a safety ceiling only. If `v_peak > max_velocity`, `v_peak` is clamped but `t_cruise` stays as set.

---

### S-Curve Profile (`traj_type = 0`) — 7 segments

```
Velocity
v_peak │      .──────────.
       │    .´            `.
       │   /                \
     0 │__/                  \__
       │ 1  2  3    4    5  6  7

Accel  │   /‾\_________/‾\
     0 │__/             \__

Jerk   │_   _           _   _
     0 │ |_| |_________|  | |_
       │+  - 0    0    -  + 0
```

| Seg | Jerk | Duration |
|-----|------|---------|
| 1 | `+j_max` | `t1_seg` |
| 2 | `0` | `t2_seg` (0 in constraint-based mode) |
| 3 | `−j_max` | `t1_seg` |
| 4 | `0` (cruise) | `t_cruise_seg` |
| 5 | `−j_max` | `t1_seg` |
| 6 | `0` | `t2_seg` |
| 7 | `+j_max` | `t1_seg` |

**Area identity:** `displacement = v_peak × (2×t1 + t2 + t_cruise)`

**Advantage over trapezoidal:** Jerk is always finite → no sudden torque spikes → less mechanical wear and vibration.

---

### Profile Comparison

| Property | Trapezoidal | S-Curve |
|----------|-------------|---------|
| Jerk | Infinite at transitions | Bounded at `j_max` |
| Move time | Shorter | ~12% longer |
| Mechanical stress | Higher | Lower |
| Segments | 3 | 7 |
| Best for | Speed | Precision / long life |

---

## 7. Live Expressions — Runtime Control

All variables can be read and written live during a debug session without stopping the CPU or reflashing. The panel is pre-populated automatically from the `.launch` file.

### Master Control

| Variable | Type | Description |
|----------|------|-------------|
| `control_mode` | uint8 R/W | 1=TrajTest 2=PosCtrl 3=Cascade 4=EncTest 5=PosVel 6=DirectPos 7=Kalman4Cal |
| `traj_type` | uint8 R/W | 0=S-Curve 1=Trapezoid |
| `pid_enabled` | uint8 R/W | 0=freeze all PIDs and hold PWM=0, 1=run |
| `start_move` | uint8 R/W | Write 1 to (re)start trajectory — self-clears |
| `reset_all` | uint8 R/W | Write 1 to clear ALL state (PIDs, traj, encoder) — self-clears |
| `zero_encoder` | uint8 R/W | Write 1 to zero encoder position — self-clears |

### Sequence Mode

| Variable | Type | Description |
|----------|------|-------------|
| `seq_targets[0]` … `seq_targets[8]` | float R/W | Target positions (deg) — fill in order, unused entries ignored |
| `seq_count` | uint8 R/W | Number of active steps (1–9) |
| `seq_enabled` | uint8 R/W | Write 1 to start sequence — self-clears when done; write 0 to abort |
| `seq_index` | uint8 R/O | Current step being executed (0-based) |
| `seq_step_delay` | float R/W | Dwell time between steps (s) — default 2.0 s; set 0 for immediate advance |

> Works in Modes 1, 2, 3. Each step advances when trajectory is complete **and** `|pos_error| ≤ pos_deadband_deg`. Set `pos_deadband_deg` to a non-zero value or the sequence will never advance.

### Trajectory Target & Profile

| Variable | Type | Description |
|----------|------|-------------|
| `traj_target_deg` | float R/W | Target position (deg) — changing auto-starts trajectory |
| `max_velocity` | float R/W | Peak velocity limit (rad/s) |
| `max_accel` | float R/W | Peak acceleration limit (rad/s²) |
| `max_jerk` | float R/W | Peak jerk limit — S-curve only (rad/s³) |
| `t1_seg` | float R/W | S-curve: jerk segment duration (s) |
| `t2_seg` | float R/W | S-curve: constant-accel segment duration (s) |
| `t_acc_seg` | float R/W | Trapezoid: acceleration duration (s) |
| `t_cruise_seg` | float R/W | Trapezoid: cruise duration (s) — writable \| S-curve: read-only auto-computed |
| `t_dec_seg` | float R/W | Trapezoid: deceleration duration (s) — may differ from t_acc_seg |

### Position PID — TIM7 Outer Loop (500 Hz)

| Variable | Type | Description |
|----------|------|-------------|
| `kp_position` | float R/W | Proportional gain (rad/s per rad) |
| `ki_pos` | float R/W | Integral gain (rad/s per rad·s) |
| `kd_pos` | float R/W | Derivative gain — acts on −velocity (rad/s per rad/s) |
| `pos_deadband_deg` | float R/W | Hard-stop settle threshold (deg) |

### Velocity PID — TIM6 Inner Loop (1000 Hz)

| Variable | Type | Description |
|----------|------|-------------|
| `kp_vel` | float R/W | Proportional gain (% per rad/s) |
| `ki_vel` | float R/W | Integral gain (% per rad) |
| `kd_vel` | float R/W | Derivative gain (% per rad/s²) |
| `min_pwm_threshold` | float R/W | Dead-zone threshold — raise to overcome static friction (%) |
| `use_kf4_vel` | uint8 R/W | Velocity feedback source: **0** = raw encoder · **1** = Kalman4 estimate — applies to both velocity PID and position PID kd term |

### Kalman4 — 4-State Motor Filter

| Variable | Type | Description |
|----------|------|-------------|
| `kf4_sigma_v2` | float R/W | σ_v² — voltage noise variance; higher = faster ω/τ_d response |
| `kf4_r_theta` | float R/W | σ_θ² — position measurement noise; higher = smoother, more lag |
| `g_kf4_velocity` | float R/O | Kalman4 estimated velocity (rad/s) |
| `g_kf4_position` | float R/O | Kalman4 estimated position (rad) |
| `g_kf4_current` | float R/O | Kalman4 estimated current (A) — model-driven, no sensor |
| `g_kf4_tau_d_obs` | float R/O | Kalman4 estimated disturbance torque (N·m) |

> Apply changes: write `apply_motor_params = 1` (position preserved) or `reset_all = 1`.

### Kalman4 Calibration (Mode 7)

| Variable | Type | Description |
|----------|------|-------------|
| `kf_cal_pwm` | float R/W | DC offset PWM for Mode 7 (−100 to +100 %) |
| `kf_sine_enabled` | uint8 R/W | 0=constant `kf_cal_pwm`, 1=sine wave excitation |
| `kf_sine_amp` | float R/W | Sine amplitude (%) |
| `kf_sine_freq` | float R/W | Sine frequency (Hz) |
| `kf_sine_dir` | float R/W | +1.0=normal, −1.0=reversed |

### Disturbance Feedforward

| Variable | Type | Description |
|----------|------|-------------|
| `distff_enabled` | uint8 R/W | 0=observe only, 1=active (adds DistFF PWM to vel PID output) |
| `distff_tau` | float R/W | Filter time constant (s) — apply with `apply_motor_params=1` |
| `g_distff_pwm` | float R/O | DistFF PWM addend (%) — observe before enabling |

> If DistFF causes oscillation at zero speed (gear backlash), set `pos_deadband_deg` ≥ backlash amplitude.

### Current Sensor — ADC1 IN2 (PA1)

| Variable | Type | Description |
|----------|------|-------------|
| `current_zero_counts` | float R/W | ADC count at zero current (default 2892.7) — trim until `g_current_A ≈ 0` with motor stopped |
| `current_counts_per_amp` | float R/W | ADC counts per ampere (default 86.57) |
| `adc_raw_current` | uint32 R/O | Raw 12-bit ADC count (0–4095) |
| `g_current_A` | float R/O | Computed motor current (A) |

> Formula: `I = (adc_raw_current − current_zero_counts) / current_counts_per_amp`

### Reference Feedforward

| Variable | Type | Description |
|----------|------|-------------|
| `refff_enabled` | uint8 R/W | 0=off, 1=active — adds model-based PWM on top of vel PID |
| `V_supply` | float R/W | Motor bus voltage (V) — must match actual supply |

### Motor Setup

| Variable | Type | Description |
|----------|------|-------------|
| `motor_dir_inverted` | uint8 R/W | 0=normal, 1=flip direction — takes effect immediately |
| `motor_J_eq` | float R/W | Equivalent inertia (kg·m²) — default 0.027 |
| `motor_b_eq` | float R/W | Equivalent viscous friction (N·m·s/rad) — default 0.5 |
| `motor_N` | float R/W | Gear ratio — default 50.0 |
| `motor_Kt` | float R/W | Torque constant (N·m/A) — default 0.00747 |
| `motor_Ke` | float R/W | Back-EMF constant (V·s/rad) — default 0.0083 |
| `motor_L` | float R/W | Armature inductance (H) — default 0.0012794 |
| `motor_R_arm` | float R/W | Armature resistance (Ω) — default 2.8 |
| `apply_motor_params` | uint8 R/W | Write 1 to apply motor param changes → re-inits RefFF, Kalman4, DistFF without zeroing encoder |

### Observables — Read Only

| Variable | Units | Description |
|----------|-------|-------------|
| `ss_error_deg` | deg | Steady-state position error |
| `ss_error_rad` | rad | Steady-state position error |
| `vel_error_live` | rad/s | Current velocity error into PID |
| `i_term_live` | — | Velocity integrator accumulator |
| `g_traj_pos` | rad | Trajectory ideal position |
| `g_smooth_vel` | rad/s | Trajectory velocity |
| `g_smooth_accel` | rad/s² | Trajectory acceleration |
| `g_jerk` | rad/s³ | Trajectory jerk (0 for trapezoid) |
| `g_position_rad` | rad | Actual encoder position |
| `g_velocity_rad_s` | rad/s | Actual raw encoder velocity |
| `g_pwm_duty` | % | PWM output to motor |

### Sweep Mode

| Variable | Type | Description |
|----------|------|-------------|
| `sweep_enabled` | uint8 R/W | Write 1 to start, 0 to abort — self-clears when complete |
| `sweep_start_deg` | float R/W | First non-zero target (deg) — default 5.0 |
| `sweep_stop_deg` | float R/W | Last non-zero target (deg) — default 360.0 |
| `sweep_step_deg` | float R/W | Increment per step (deg) — default 5.0 |
| `sweep_delay_s` | float R/W | Dwell time at each position (s) — default 2.0 |
| `sweep_index` | uint8 R/O | Current step index (0-based) |

> Example: `start=5, stop=360, step=5` → 144 moves: 0→5→0→10→…→0→360.
> Each step advances when trajectory done AND `|pos_error| ≤ pos_deadband_deg`.
> `reset_all` aborts the sweep. Works in Mode 3 only.

---

## 8. UART Telemetry & Simulink

### Port Settings

| Setting | Value |
|---------|-------|
| Port | COMx (check Device Manager → Ports → STMicroelectronics STLink) |
| Baud rate | **2,000,000** |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |

### Packet Format (72 bytes, sent every 1 ms)

```
Byte  0– 1 : 0x7E 0x7E        header
Byte  2– 5 : traj_pos          float  trajectory ideal position   (rad)
Byte  6– 9 : traj_vel          float  trajectory velocity         (rad/s)
Byte 10–13 : traj_accel        float  trajectory acceleration     (rad/s²)
Byte 14–17 : traj_jerk         float  trajectory jerk             (rad/s³)
Byte 18–21 : act_pos           float  encoder position            (rad)
Byte 22–25 : act_vel           float  raw encoder velocity        (rad/s)
Byte 26–29 : kf4_vel_compat    float  Kalman4 velocity (compat slot) (rad/s)
Byte 30–33 : kf4_pos_compat    float  Kalman4 position (compat slot) (rad)
Byte 34–37 : ss_error          float  steady-state position error (rad)
Byte 38–41 : current           float  measured motor current      (A)
Byte 42–45 : kf4_pos           float  Kalman4 estimated position  (rad)
Byte 46–49 : kf4_vel           float  Kalman4 estimated velocity  (rad/s)
Byte 50–53 : kf4_current       float  Kalman4 estimated current   (A)
Byte 54–57 : kf4_tau_d         float  Kalman4 disturbance torque  (N·m)
Byte 58–61 : voltage           float  motor commanded voltage     (V)
Byte 62–65 : adc_raw           uint32 raw ADC count               (0–4095)
Byte 66–69 : i_term_live       float  velocity integrator accumulator
Byte 70–71 : 0x03 0x03        footer
```

No checksum. Validate by checking header (0x7E 0x7E) and footer (0x03 0x03).

### Simulink Setup

1. Add a **Serial Configuration** block → set port and baud rate to 2000000
2. Add a **Serial Receive** block → Data size: **72** bytes, Data type: uint8
3. Add a **MATLAB Function** block to parse the packet:

```matlab
function [traj_pos, traj_vel, traj_accel, traj_jerk, ...
          act_pos, act_vel, kf_vel, kf_pos, ss_error, ...
          current, kf4_pos, kf4_vel, kf4_current, kf4_tau_d, ...
          voltage, adc_raw, i_term, valid] = parse_packet(raw)
    valid = (raw(1) == hex2dec('7E')) && (raw(2) == hex2dec('7E')) && ...
            (raw(71) == hex2dec('03')) && (raw(72) == hex2dec('03'));
    traj_pos    = typecast(uint8(raw(3:6)),   'single');
    traj_vel    = typecast(uint8(raw(7:10)),  'single');
    traj_accel  = typecast(uint8(raw(11:14)), 'single');
    traj_jerk   = typecast(uint8(raw(15:18)), 'single');
    act_pos     = typecast(uint8(raw(19:22)), 'single');
    act_vel     = typecast(uint8(raw(23:26)), 'single');
    kf_vel      = typecast(uint8(raw(27:30)), 'single');
    kf_pos      = typecast(uint8(raw(31:34)), 'single');
    ss_error    = typecast(uint8(raw(35:38)), 'single');
    current     = typecast(uint8(raw(39:42)), 'single');
    kf4_pos     = typecast(uint8(raw(43:46)), 'single');
    kf4_vel     = typecast(uint8(raw(47:50)), 'single');
    kf4_current = typecast(uint8(raw(51:54)), 'single');
    kf4_tau_d   = typecast(uint8(raw(55:58)), 'single');
    voltage     = typecast(uint8(raw(59:62)), 'single');
    adc_raw     = typecast(uint8(raw(63:66)), 'uint32');
    i_term      = typecast(uint8(raw(67:70)), 'single');
end
```

4. Connect outputs to **Scope** blocks with sample time = 0.001 s

### Recommended Scope Channels

| Channel | Signal | What to look for |
|---------|--------|-----------------|
| 1 | `traj_vel` | Trajectory velocity shape (S-curve or trapezoid) |
| 2 | `act_vel` | Raw encoder velocity tracking |
| 3 | `kf_vel` | Kalman estimated velocity (smoother than act_vel) |
| 4 | `traj_pos` vs `act_pos` | Position tracking error |
| 5 | `ss_error` | Steady-state position error (should converge to 0) |
| 6 | `current` | Motor current draw during motion |

---

## 9. Kalman4 Filter

Kalman4 is a 4-state physics-based motor Kalman filter that runs every TIM6 tick at 1 kHz.
States: **θ** (position), **ω** (velocity), **i** (armature current), **τ_d** (disturbance torque).

### What it estimates

| Output | Variable | Use |
|--------|----------|-----|
| Position | `g_kf4_position` | Compare with `g_position_rad` |
| Velocity | `g_kf4_velocity` | Compare with `g_velocity_rad_s` |
| Current | `g_kf4_current` | Model-only — no current sensor feedback |
| Disturbance | `g_kf4_tau_d_obs` | Input to DistFF — cancels load disturbances |

### Tuning

| Parameter | Effect |
|-----------|--------|
| `kf4_sigma_v2` (higher) | Faster ω and τ_d tracking, noisier |
| `kf4_r_theta` (higher) | Smoother position estimate, more lag |

After changing either, write `apply_motor_params = 1` to re-init without resetting position.

### Motor Model Parameters

Kalman4 uses the motor model for its prediction step. If the model is inaccurate, the
filter will be biased. Tune motor params in Live Expressions then `apply_motor_params = 1`.

**Tuning workflow (Mode 7):**
1. Set `control_mode = 7`, apply a sine: `kf_sine_enabled=1`, `kf_sine_amp=10`, `kf_sine_freq=1`
2. Observe `g_kf4_velocity` vs `g_velocity_rad_s` on Simulink (bytes 46–49 vs 22–25)
3. Increase `kf4_sigma_v2` if the estimate lags; increase `kf4_r_theta` if it is too noisy
4. Apply: `apply_motor_params = 1`

---

## 10. Reference Feedforward

The reference feedforward computes a model-based voltage term from the velocity reference, adding it on top of the velocity PID output. This reduces the PID's tracking error during ramp phases.

### Control Law

```
pwm_ff = RefFF_Update(vel_command) / V_supply × 100 %
pwm_total = Velocity_PID(vel_error) + pwm_ff
```

### When to Use

Enable feedforward after the PIDs are tuned:
1. Set `V_supply` to match your actual motor supply voltage (e.g. 24.0)
2. Set `refff_enabled = 1`
3. Observe `g_velocity_rad_s` vs `g_smooth_vel` during ramp — the lag should decrease

> Motor model parameters (`motor_J_eq`, `motor_b_eq`, `motor_N`, `motor_Kt`, `motor_Ke`, `motor_L`, `motor_R_arm`) are live-tunable via Live Expressions. Change any value then write `apply_motor_params = 1` to re-initialise RefFF, Kalman4, and DistFF without resetting position.

---

## 11. Parameter Tuning

### Current Default Parameters

All trajectory and PID parameters are writable live via Live Expressions. Hard-coded limits in `main.c`:
- `max_pwm = 100.0 %` — maximum PWM to motor
- `LOOP_DT = 0.001 s` — inner loop period (TIM6)
- `POS_DT = 0.002 s` — outer loop period (TIM7)

### Velocity PID Tuning Procedure

**Step 1 — Set `control_mode = 3`** and `pid_enabled = 1`. Set a small target (`traj_target_deg = 30`) and observe `g_velocity_rad_s` vs `g_smooth_vel`.

**Step 2 — Tune Kp first:**
- Increase `kp_vel` until the motor tracks the trajectory
- If it oscillates: reduce `kp_vel`

**Step 3 — Add Ki to eliminate steady-state error:**
- Increase `ki_vel` slowly
- If it winds up: reduce `ki_vel`

**Step 4 — Tune Kalman4 noise parameters (optional):**
- In Mode 7, apply a sine excitation (`kf_sine_enabled = 1`) and observe `g_kf4_velocity` (bytes 46–49 in telemetry)
- Reduce `kf4_sigma_v2` for smoother velocity estimate; increase if the filter lags
- Reduce `kf4_r_theta` to trust the encoder more; increase if encoder noise is high
- Write `reset_all = 1` after changing noise parameters

**Step 4b — Switch velocity feedback to Kalman4 (optional):**
- Once Kalman4 is tuned, set `use_kf4_vel = 1`
- Both velocity PID error and position PID kd term will now use `g_kf4_velocity`
- If PID becomes unstable → reduce `kf4_sigma_v2` or revert with `use_kf4_vel = 0`
- Compare `vel_error_live` before and after switching — it should become smoother

**Step 5 — Add feedforward (optional):**
- Set `refff_enabled = 1`, verify `V_supply` matches your bus voltage
- Ramp tracking should improve without affecting PID stability

**Step 6 — Add disturbance feedforward (optional):**
- Observe `g_distff_pwm` in Live Expressions before enabling to check magnitude
- Set `distff_enabled = 1` to activate; adjust `distff_tau` then write `apply_motor_params = 1`

### Typical Starting Point for Small DC Motors

| Gain | Start | Increase if | Decrease if |
|------|-------|-------------|-------------|
| `kp_vel` | 20.0 | Motor too slow to follow | Oscillation |
| `ki_vel` | 0.1 | Steady-state velocity error remains | Windup / overshoot |
| `kp_position` | 1.0 | Position error remains after trajectory | Oscillation |
| `ki_pos` | 0.0 | Steady-state position error | Windup |

---

## 12. Time-Based Segment Control

Segment durations are specified directly via Live Expressions. The firmware back-computes `v_peak` from your specified times and the target displacement, then auto-computes `t_cruise` to hit `max_velocity`.

### How It Works

**Trapezoidal:**
```
v_peak   = |displacement| / (t_acc_seg/2 + t_cruise_seg + t_dec_seg/2)
v_peak   = min(v_peak, max_velocity)   [clamp only — t_cruise unchanged]
a_acc    = v_peak / t_acc_seg
a_dec    = v_peak / t_dec_seg
```

**S-Curve:**
```
v_peak   = max_velocity
t_cruise = |displacement| / max_velocity − 2×t1_seg − t2_seg   (clamped ≥ 0)
```

`t_cruise_seg` in Live Expressions shows the computed cruise time — read-only.

### Step-by-Step Usage

1. Set your desired segment shape:

```
For S-Curve:
  t1_seg       = 0.1   ← jerk transition time (segs 1,3,5,7 each)
  t2_seg       = 0.1   ← constant-acceleration time (segs 2,6 each)
  (t_cruise_seg is auto-computed — read-only)

For Trapezoidal:
  t_acc_seg    = 0.3   ← acceleration time (seg 1)
  t_cruise_seg = 0.2   ← cruise time       (seg 2) — writable
  t_dec_seg    = 0.3   ← deceleration time (seg 3) — may differ from t_acc
```

2. Set `traj_target_deg` → profile starts automatically with your timing

### Minimum Displacement for Cruise Phase

| Profile | Cruise exists when |
|---------|-------------------|
| Trapezoidal | `t_cruise_seg > 0` (user-set directly) |
| S-Curve | `|disp| > max_velocity × (2×t1 + t2)` |

For Trapezoidal: if `t_cruise_seg = 0`, the profile is a triangle (accel + decel only). `v_peak` is always back-computed from times — `max_velocity` is a ceiling, not a target.

---

## 13. Troubleshooting

### Motor does not move

| Check | Action |
|-------|--------|
| `control_mode` value | Set to 3 (Mode 3 — Cascade) |
| `traj_target_deg` | Must be non-zero (≥ 1 deg) |
| `pid_enabled` | Must be 1 |
| `g_pwm_duty` | Check if it is non-zero in Live Expressions |
| `min_pwm_threshold` | Raise if motor won't start from rest |

### Motor runs in only one direction

| Check | Action |
|-------|--------|
| `motor_dir_inverted` | Toggle between 0 and 1 |
| Direction pin | Confirm PA0 is wired to driver DIR input |

### No UART data in Simulink

| Check | Action |
|-------|--------|
| COM port | Check Device Manager → correct COMx selected |
| Baud rate | Must be exactly **2,000,000** |
| Data size | Must be **72** bytes in Serial Receive block |
| Sync bytes | First two bytes must be `0x7E 0x7E`, last two `0x03 0x03` |

### Profile looks like triangle / parabola (no cruise phase)

| Check | Action |
|-------|--------|
| `traj_target_deg` too small | Increase displacement above minimum (see §12) |
| `t_cruise_seg` | Read value — if 0, displacement too small for cruise at current params |

### S-Curve looks like a trapezoid (no visible rounded corners)

`t1_seg` is too small → jerk segments are too short to see.
- Increase `t1_seg` (e.g. 0.5 or 1.0 s)
- Or reduce `max_jerk` in constraint-based code

### PID oscillation in Mode 3

- Reduce `kp_vel` first
- If that does not help, reduce `ki_vel` or `kp_position`
- If `use_kf4_vel = 1`: reduce `kf4_sigma_v2` for a smoother Kalman4 estimate, or revert to `use_kf4_vel = 0` (raw encoder) to isolate cause

### Kalman4 velocity drifts or diverges

- Write `reset_all = 1` to reinitialise the filter
- Raise `kf4_r_theta` to trust the encoder position measurement more
- Lower `kf4_sigma_v2` if the velocity estimate is noisy

---

## Appendix A — Complete Source File List

| File | Role |
|------|------|
| `Core/Src/main.c` | Entry point, PID controllers, PWM output, UART telemetry, ISR control loops |
| `Core/Src/SCURVE.c` | 7-segment S-curve profile generator |
| `Core/Src/TRAPEZOID.c` | 3-segment trapezoidal profile generator |
| `Core/Src/ENCODER.c` | Quadrature encoder decoding, windowed velocity |
| `Core/Src/REF_FEEDFORWARD.c` | Tustin-discretised model-based reference feedforward |
| `Core/Inc/Kalman4.h` | 4-state motor Kalman filter — struct and API declarations |
| `Core/Src/Kalman4.c` | 4-state motor Kalman filter (position, velocity, current, disturbance torque) |
| `Core/Inc/DistFF.h` | Disturbance feedforward — struct and API declarations |
| `Core/Src/DistFF.c` | Disturbance feedforward filter (τ_d estimate → compensating voltage) |
| `Core/Src/stm32g4xx_it.c` | Interrupt handlers |
| `Core/Inc/main.h` | GPIO / pin macros |
| `Core/Inc/SCURVE.h` | SCurve_t struct and function declarations |
| `Core/Inc/TRAPEZOID.h` | Trapezoid_t struct and function declarations |
| `Core/Inc/ENCODER.h` | Encoder_t struct, macros (PPR, CPR, COUNTS_TO_RAD) |
| `Core/Inc/REF_FEEDFORWARD.h` | RefFF_t struct and function declarations |

---

## Appendix B — DC Motor State-Space Model

For observer / controller design, the geared DC motor with disturbance state:

**States:** `x = [θ_l, θ̇_l, i, τ_d]ᵀ`

```
⎡ θ̇_l ⎤   ⎡  0       1       0      0  ⎤ ⎡θ_l ⎤   ⎡  0  ⎤
⎢ θ̈_l ⎥ = ⎢  0     -B/J   NKt/J  -1/J ⎥ ⎢θ̇_l ⎥ + ⎢  0  ⎥ · V
⎢  i̇  ⎥   ⎢  0  -NKe/L   -R/L    0   ⎥ ⎢ i  ⎥   ⎢ 1/L ⎥
⎣ τ̇_d ⎦   ⎣  0       0       0      0  ⎦ ⎣τ_d ⎦   ⎣  0  ⎦

y = [1  0  0  0] · x
```

| Symbol | Meaning |
|--------|---------|
| `θ_l` | Load shaft angle (rad) |
| `θ̇_l` | Load shaft velocity (rad/s) |
| `i` | Armature current (A) |
| `τ_d` | Disturbance torque (Nm) |
| `J` | Total inertia = `Jm·N² + Jl` |
| `N` | Gear ratio (motor → load) |
| `Kt` | Torque constant (Nm/A) |
| `Ke` | Back-EMF constant (V·s/rad) |
| `B` | Viscous damping (Nm·s/rad) |
| `R` | Armature resistance (Ω) |
| `L` | Armature inductance (H) |

---

*Generated for Auto_Control firmware — STM32G474RE NUCLEO board*
