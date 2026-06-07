# Auto_Control — Developer Handoff Document
**STM32G474RE Motor Controller with Trajectory Generation & Cascade PID**
**Version: June 2026 rev7 | Board: NUCLEO-G474RE**

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Hardware Setup](#2-hardware-setup)
3. [File Structure](#3-file-structure)
4. [Library APIs](#4-library-apis)
   - [4.1 ENCODER](#41-encoder)
   - [4.2 SCURVE (7-Segment)](#42-scurve-7-segment)
   - [4.3 TRAPEZOID (3-Segment)](#43-trapezoid-3-segment)
   - [4.4 KALMAN4 (4-State Motor Filter)](#44-kalman4-4-state-motor-filter)
   - [4.5 DISTFF — Disturbance Feedforward](#45-distff--disturbance-feedforward)
5. [main.c Architecture](#5-mainc-architecture)
   - [5.1 Startup Sequence](#51-startup-sequence-user-code-begin-2)
   - [5.2 Main Loop](#52-main-loop-while1)
   - [5.3 ISR Control Loops](#53-isr-control-loops-hal_tim_periodelapsedcallback)
   - [5.4 Start_Trajectory Helper](#54-start_trajectory-helper)
   - [5.5 Set_Motor_PWM](#55-set_motor_pwm)
6. [Control Modes](#6-control-modes)
7. [Live Expressions Variables](#7-live-expressions-variables)
   - [7.1 Control & Mode](#71-control--mode)
   - [7.2 Trajectory Profile Limits](#72-trajectory-profile-limits)
   - [7.3 Trajectory Shape Parameters](#73-trajectory-shape-parameters)
   - [7.3a Inter-Loop Interface](#73a-inter-loop-interface-read-only-observables)
   - [7.4 Velocity PID (Inner Loop)](#74-velocity-pid-inner-loop)
   - [7.4a Kalman Filter](#74a-kalman-filter)
   - [7.4b Sequence Mode](#74b-sequence-mode)
   - [7.4c Current Sensor — ADC1 IN2 (PA1)](#74c-current-sensor--adc1-in2-pa1)
   - [7.4d Kalman4 — Mode 7 Calibration](#74d-kalman4--mode-7-calibration)
   - [7.4e Reference Feedforward](#74e-reference-feedforward)
   - [7.5 Position PID (Outer Loop)](#75-position-pid-outer-loop)
   - [7.6 Motor Output](#76-motor-output)
   - [7.7 Debug Observables (Read-only)](#77-debug-observables-read-only)
8. [UART Telemetry → Simulink](#8-uart-telemetry--simulink)
9. [Trajectory Generation](#9-trajectory-generation)
10. [PID Controllers](#10-pid-controllers)
    - [10.1 Velocity PID](#101-velocity-pid-inner-loop)
    - [10.2 Position PID](#102-position-pid-outer-loop)
    - [10.3 PID Resets](#103-pid-resets)
11. [Bugs Fixed in This Session](#11-bugs-fixed-in-this-session)
12. [Tuning Guide](#12-tuning-guide)
13. [DC Motor State-Space Model](#13-dc-motor-state-space-model)
14. [Known Limitations & Future Work](#14-known-limitations--future-work)

---

## 1. Project Overview

ระบบ real-time motor motion controller สำหรับ DC Motor พร้อม Encoder บน STM32G474RE

### Key Features
| Feature | Detail |
|---------|--------|
| Inner loop (velocity) | **1 kHz** (TIM6 interrupt, 1 ms) — encoder + ADC + Kalman4 + trajectory update + velocity PID + RefFF + DistFF + telemetry |
| Outer loop (position) | **500 Hz** (TIM7 interrupt, 2 ms) — position PID, seq/sweep control, Start_Trajectory arming |
| Trajectory profiles | **S-Curve 7-segment** (jerk-limited) and **Trapezoidal 3-segment** |
| Trajectory mode | Always **time-based** — S-curve: t_cruise auto-computed from max_velocity \| Trapezoid: all times user-set directly |
| Active control modes | **Mode 3** (Cascade PID) and **Mode 7** (Kalman4 calibration) |
| Live tuning | All parameters changeable via STM32CubeIDE Live Expressions — motor params via `apply_motor_params=1` |
| Kalman4 | 4-state motor filter [θ, ω, i, τ_d] — position measurement, full ZOH physics model |
| DistFF | Disturbance feedforward — Kalman4 τ_d estimate → compensating voltage addend |
| Sweep mode | Programmatic 0→target→0→… sweep, generates targets automatically |
| Telemetry | Binary packet **72 bytes** @ 2 Mbaud → Simulink (packed in TIM6) |
| Encoder | Quadrature, 2048 PPR × 4x = 8192 counts/rev, 10-tick windowed velocity |

### System Architecture Overview
```
traj_target_deg (Live Expressions)
        │
        ▼  TIM7 @ 500 Hz  arms trajectory via Start_Trajectory(disp)
        │
        │  TIM6 @ 1 kHz  updates trajectory: SCurve_Update / Trapezoid_Update
        │    → writes g_traj_pos, g_smooth_vel, g_traj_active to volatile globals
        │
        └──[Mode 3: Cascade] ─► TIM7 Position PID (reads g_traj_pos) ─► vel_command
                                                                            │
        ┌───────────────────────────────────────────────────────────────────┘
        │  TIM6 @ 1 kHz  (inner velocity loop)
        ▼
[Velocity PID] ──► + RefFF_Update (if refff_enabled)
                   + DistFF_Update (if distff_enabled, uses g_kf4_tau_d_obs)
                   ──► PWM ─► Motor
        ▲
 my_encoder.velocity_rad_s  (10-tick windowed velocity)
 Kalman4 runs unconditionally every tick: g_kf4_position, g_kf4_velocity,
   g_kf4_current, g_kf4_tau_d_obs  (feeds DistFF)
```

**Inter-loop interface (volatile globals)**
| Variable | Direction | Used by |
|----------|-----------|---------|
| `vel_command` | TIM7 → TIM6 | Mode 3 — velocity set-point |
| `pwm_command` | TIM7 → TIM6 | Mode 7 — direct PWM |
| `hard_stop_active` | TIM7 → TIM6 | All — 1 = hold zero, reset vel PID |
| `g_traj_pos` | TIM6 → TIM7 | Mode 3 — ideal position for pos PID |
| `g_smooth_vel` | TIM6 → TIM7 | Mode 3 — velocity feedforward |
| `g_traj_active` | TIM6 → TIM7 | Mode 3 — hard-stop condition |

---

## 2. Hardware Setup

### MCU Specs
| Item | Value |
|------|-------|
| MCU | STM32G474RETx |
| Core | ARM Cortex-M4 @ 170 MHz |
| Flash | 512 KB |
| SRAM | 128 KB |
| Board | NUCLEO-G474RE |
| Clock | HSI 16 MHz → PLL → 170 MHz (boost mode) |

### Pin Connections
| STM32 Pin | Signal | Connect to |
|-----------|--------|-----------|
| **PA0** | `MOTOR_DIR_Pin` | Motor driver direction input |
| **PA1** (ADC1_IN2) | Current sensor | Analog current sense input (0–3.3 V) |
| **PA8** (TIM1_CH1) | Encoder A | Encoder channel A |
| **PA9** (TIM1_CH2) | Encoder B | Encoder channel B |
| **PC9** (TIM3_CH4) | PWM output | Motor driver PWM input |
| **PA2** (LPUART1_TX) | UART TX | ST-LINK (virtual COM — USB only) |
| **PA3** (LPUART1_RX) | UART RX | ST-LINK (virtual COM — USB only) |
| **PC13** | Button B1 | Onboard (no wiring needed) |
| **PA5** | LED LD2 | Onboard (no wiring needed) |

### Timer Assignments
| Timer | Role | Configuration |
|-------|------|--------------|
| **TIM1** | Encoder QEI | ARR=65535 (full 16-bit), TIM_ENCODERMODE_TI12 |
| **TIM3** | Motor PWM | Prescaler=169, ARR=999 → 1 kHz PWM, CCR1=duty |
| **TIM6** | Inner loop ISR (velocity) | Prescaler=167, ARR=999 → ~1 kHz interrupt, priority 0 |
| **TIM7** | Outer loop ISR (position) | Prescaler=169, ARR=1999 → 500 Hz interrupt, priority 1 |
| **LPUART1** | UART telemetry | 2,000,000 baud, 8N1 |

> **Important:** TIM1 ARR must be **65535** (not 8191). ARR=65535 ทำให้ signed 16-bit delta cast จัดการ wrap-around ได้ถูกต้อง

> **IRQ Priorities:** TIM6 (priority 0) > TIM7 (priority 1). TIM6 can preempt TIM7 mid-execution — this is intentional so the velocity loop always fires on time.

---

## 3. File Structure

```
Auto_Control/
├── Core/
│   ├── Inc/
│   │   ├── main.h              ← GPIO/pin macros (MOTOR_DIR_Pin_Pin, LD2_Pin, B1_Pin)
│   │   ├── ENCODER.h           ← Encoder_t struct, VEL_WINDOW=10, API declarations
│   │   ├── SCURVE.h            ← SCurve_t struct, 7-segment API
│   │   ├── TRAPEZOID.h         ← Trapezoid_t struct, 3-segment API
│   │   ├── REF_FEEDFORWARD.h   ← RefFF_t struct, feedforward API
│   │   ├── Kalman4.h           ← KalmanFilter4_t struct, 4-state motor filter API
│   │   └── DistFF.h            ← DistFF_t struct, disturbance feedforward API
│   └── Src/
│       ├── main.c              ← Entry point, all control logic, PID, UART
│       ├── ENCODER.c           ← Windowed velocity encoder implementation
│       ├── SCURVE.c            ← 7-segment S-curve trajectory generator
│       ├── TRAPEZOID.c         ← 3-segment trapezoidal trajectory generator
│       ├── REF_FEEDFORWARD.c   ← Tustin-discretised model-based feedforward
│       ├── Kalman4.c           ← 4-state motor Kalman filter (pos, vel, current, τ_d)
│       └── DistFF.c            ← disturbance feedforward (τ_d estimate → voltage)
├── Drivers/
│   ├── CMSIS/              ← ARM Cortex-M4 core support
│   └── STM32G4xx_HAL_Driver/
├── DEVELOPER_HANDOFF.md    ← This file
└── USER_GUIDE.md           ← End-user guide (Thai + English)
```

### IDE / Toolchain Note

> **Clang false-positive diagnostics (normal — not real errors):**
> STM32CubeIDE uses **arm-none-eabi-gcc** to compile this project. If your editor runs a host Clang language server (clangd/IntelliSense), it will report errors such as `'main.h' file not found`, `Unknown type name 'TIM_HandleTypeDef'`, `Unknown type name 'uint8_t'`, etc.
> These appear because the STM32 HAL and CMSIS headers are not on the host Clang include path. **They are not real build errors.** The project compiles and links correctly in STM32CubeIDE. Ignore these diagnostics; fix only errors reported by the IDE's actual build output.

---

## 4. Library APIs

### 4.1 ENCODER

#### Struct
```c
#define VEL_WINDOW  10  // velocity averaging window (10 ticks = 10ms)

typedef struct {
    uint16_t last_counter_value;    // raw TIM1 counter (previous tick)
    int32_t  count_accum;           // 32-bit accumulated counts (unlimited turns)
    int32_t  vel_buf[VEL_WINDOW];   // ring buffer for windowed velocity
    uint8_t  vel_idx;               // next write index
    uint8_t  vel_full;              // 1 when buffer filled once
    float    position_rad;          // absolute position (rad)
    float    velocity_rad_s;        // windowed velocity (rad/s) — 10x less noise
    float    dt;                    // time step (s)
} Encoder_t;
```

#### API
```c
void Encoder_Init  (Encoder_t *enc, float dt);
void Encoder_Update(Encoder_t *enc, uint16_t current_timer_value);
```

#### Windowed Velocity Logic
```
Single-tick velocity:  resolution = 0.767 rad/s per count  (noisy)
10-tick window:        resolution = 0.077 rad/s per count  (10x better)
Lag introduced:        VEL_WINDOW × dt / 2 = 5 ms

velocity = (count_accum[k] - count_accum[k-10]) / (10 × dt)
```

#### Wrap-around Handling
```c
// TIM1 ARR=65535, uint16 subtraction wraps at 65536
int16_t delta = (int16_t)(current_cnt - last_cnt);
// int16_t cast: values > 32767 become negative → correct signed delta
```

---

### 4.2 SCURVE (7-Segment)

#### Struct
```c
typedef struct {
    float v_max, a_max, j_max, dt;  // configuration
    float T[8];       // T[0]=0, T[1..7] = cumulative segment end times
    float v_peak;     // actual peak velocity achieved
    float a_peak;     // actual peak acceleration used
    float dir;        // +1.0 or -1.0 (direction)
    float t_now;      // elapsed time since move start
    float v_current;  // output: velocity (rad/s)
    float a_current;  // output: acceleration (rad/s²)
    float j_current;  // output: jerk (rad/s³)
    float p_current;  // output: integrated ideal position (rad, relative)
    uint8_t is_active; // 1 while running, 0 when done
} SCurve_t;
```

#### 7-Segment Jerk Schedule
```
Seg 1: jerk = +j_max  → a: 0 → a_peak      (duration t1)
Seg 2: jerk =  0      → a: a_peak           (duration t2, may=0)
Seg 3: jerk = -j_max  → a: a_peak → 0      (duration t1)
Seg 4: jerk =  0      → cruise at v_peak    (duration t_cruise, may=0)
Seg 5: jerk = -j_max  → a: 0 → -a_peak     (duration t1)
Seg 6: jerk =  0      → a: -a_peak         (duration t2, may=0)
Seg 7: jerk = +j_max  → a: -a_peak → 0    (duration t1)
```

#### API
```c
void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max, float dt);

// Constraint-based: computes times from v_max, a_max, j_max + displacement
void SCurve_SetTarget(SCurve_t *sc, float displacement);

// Time-based: you specify each segment duration; profile covers exactly displacement
// t1=jerk duration, t2=const-accel duration, t_cruise=cruise duration
void SCurve_SetTarget_ByTime(SCurve_t *sc, float displacement,
                              float t1, float t2, float t_cruise);

void SCurve_Update(SCurve_t *sc);   // call every dt
```

#### Key Math
```
v_peak  = displacement / (2*t1 + t2 + t_cruise)   [time-based]
a_peak  = v_peak / (t1 + t2)
j_used  = a_peak / t1   (derived in Update from a_peak/T[1])

Cruise phase exists when: displacement > v_max × (2*t1 + t2)   [ByTime: t_cruise > 0]
```

> When `t2 = 0` the profile degenerates to a pure constant-jerk triangle (ramp up, immediately ramp down — no flat-top acceleration). Set `t2_seg > 0` to get a true constant-acceleration plateau between the two jerk segments.

---

### 4.3 TRAPEZOID (3-Segment)

#### Struct
```c
typedef struct {
    float v_max, a_max, dt;
    float v_peak, T1, T2, T3, dir;
    float t_now;
    float v_current, a_current, p_current;
    uint8_t is_active;
} Trapezoid_t;
```

#### 3-Segment Schedule (asymmetric)
```
Seg 1: a = +a_acc  → v: 0      → v_peak   (duration t_acc)
Seg 2: a =  0      → v: v_peak            (duration t_cruise, may=0)
Seg 3: a = -a_dec  → v: v_peak → 0       (duration t_dec)
```
`a_acc` and `a_dec` are independent — asymmetric acceleration/deceleration supported.

#### API
```c
void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt);
void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement);

// t_acc, t_cruise, t_dec — all user-specified directly (no auto-compute from v_max)
// v_peak clamped to v_max only if computed value exceeds it; times unchanged.
void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement,
                                 float t_acc, float t_cruise, float t_dec);
void Trapezoid_Update(Trapezoid_t *tr);
```

#### Key Math
```
v_peak = |d| / (t_acc/2 + t_cruise + t_dec/2)   [time-based, asymmetric]
v_peak = min(v_max, sqrt(a_max × |d|))           [constraint-based, symmetric]

a_acc = v_peak / t_acc    (acceleration rate, segment 1)
a_dec = v_peak / t_dec    (deceleration rate, segment 3)

Clamp:  if v_peak > v_max → v_peak = v_max
        t_cruise extended: t_cruise = d/v_max − t_acc/2 − t_dec/2  (ensures full displacement)
        a_acc/a_dec recomputed from clamped v_peak
        t_cruise is only extended in this fallback — user's value is used when no clamping occurs
```

---

### 4.4 KALMAN4 (4-State Motor Filter)

Physics-based Kalman filter that estimates the full motor state from position measurement only. Uses zero-order hold (ZOH) discretisation for the current state transition (F[2][2]).

#### State vector
```
x = [θ_l,  ω_l,  i,  τ_d]ᵀ
     pos   vel  curr  disturbance torque
```

#### Matrices (set in `Kalman4_Init`)

```
F = [[1,  Ts,                    0,            0       ],
     [0,  1-(b/J)*Ts,       (N*Kt/J)*Ts, -(1/J)*Ts    ],
     [0, -(Ke*N/L)*Ts,  exp(-(R/L)*Ts),   0            ],  ← ZOH (NOT Euler)
     [0,  0,                    0,            1       ]]

G = [0, 0, Ts/L, 0]ᵀ      ← voltage noise enters through current channel

H = [1, 0, 0, 0]           ← 1×4, position only (no current sensor)

Q  rebuilt every TIM6 tick via second-order Van Loan expansion:
   Qk ≈ Qc·Ts + (A·Qc + Qc·Aᵀ)·Ts²/2 + A·Qc·Aᵀ·Ts³/3
   Qc[2][2] = kf4_sigma_v2 / L²   (only nonzero entry in Qc)
   Result: Q[2][2], Q[1][2]=Q[2][1], Q[1][1] nonzero; rest are zero.

R = [kf4_r_theta]          ← 1×1 scalar
```

> **ZOH for F[2][2]:** τ_e = L/R ≈ 0.46 ms < Ts = 1 ms, so Euler gives F[2][2] ≈ −1.19 (unstable).
> ZOH: F[2][2] = exp(−(R/L)·Ts) ≈ 0.111 (stable).
>
> **Q second-order expansion:** because RL = R/L ≈ 2188 s⁻¹ is large relative to Ts, the
> first-order approximation Q[2][2] = σ_v²·Ts²/L² is significantly under-estimated.
> Recomputing Q each tick from `kf4_sigma_v2` and motor parameters adds negligible overhead.

#### Struct
```c
typedef struct {
    arm_matrix_instance_f32 x, P, F, G, Q, H, R, K, S, S_inv, I;
    float x_data[4];        // state: [theta, omega, i, tau_d]
    float P_data[16];       // 4x4 error covariance
    float F_data[16];       // 4x4 physics transition
    float G_data[4];        // 4x1 control input
    float Q_data[16];       // 4x4 process noise
    float H_data[8];        // 2x4 observation
    float R_data[4];        // 2x2 measurement noise
    float K_data[8];        // 4x2 Kalman gain
    float S_data[4];        // 2x2 innovation covariance
    float S_inv_data[4];    // 2x2 S inverse (analytical)
    float I_data[16];       // 4x4 identity
    float Ts;
} KalmanFilter4_t;
```

#### API
```c
// Build all matrices from motor + noise parameters
void Kalman4_Init(KalmanFilter4_t *kf,
                  float theta0, float omega0, float current0,
                  float Ts,
                  float J_eq, float b_eq, float N,
                  float Kt, float Ke, float L, float R_arm,
                  float sigma_v2,  // kf4_sigma_v2 — voltage noise var → Q 2nd-order expansion
                  float r_theta);  // kf4_r_theta  — position measurement noise variance

// Reset state to [theta0, omega0, current0, 0] and P to 100·I
// Keeps F, G, Q, H, R unchanged — call Init instead if noise values changed
void Kalman4_Reset(KalmanFilter4_t *kf,
                   float theta0, float omega0, float current0);

// One predict + correct cycle. Returns x_data[4]
float* Kalman4_Update(KalmanFilter4_t *kf,
                      float theta_meas, // encoder position (rad)
                      float voltage);   // applied voltage (V) = pwm%/100 × V_supply
```

#### Noise Parameter Guide

| Parameter | Symbol | Role | Effect of increasing |
|-----------|--------|------|----------------------|
| `kf4_sigma_v2` | σ_v² | voltage noise driving Q | faster ω and τ_d response, noisier |
| `kf4_r_theta` | σ_θ² | position meas. noise | smoother θ estimate, more lag |

> Only two tuning variables. Q is computed automatically from σ_v², Ts, and motor model params.
> After changing either, write `apply_motor_params = 1` (position-preserving) or `reset_all = 1`.

> **Q[1][1] risk:** With Q_c having no ω noise term, the filter has 100% confidence in the
> F-matrix ω prediction. Any J/b/N/Kt model error causes permanent ω bias. If observed,
> add a small direct Q[1][1] term or adjust motor model parameters.

---

### 4.5 DISTFF — Disturbance Feedforward

Converts the Kalman4 disturbance torque estimate `τ̂_d` (x[3]) into a compensating voltage, which is then added to the velocity PID PWM output.

#### Continuous transfer function
```
                n1·s + n0         L·s + R
G_ff(s)  =  ─────────────────  =  ─────────────────
                d1·s + d0        N·Kt·τ·s + N·Kt
```

#### Tustin discretisation (c = 2/Ts)
```
b0 =  n1·c + n0    b1 = -n1·c + n0
a0 =  d1·c + d0    a1 = -d1·c + d0
```

#### Difference equation
```
y[k] = ( b0·u[k]  +  b1·u[k-1]  -  a1·y[k-1] ) / a0

u[k]  — tau_d_hat    Kalman4 disturbance estimate (N·m)  — x_data[3]
y[k]  — V_ff_d       feedforward voltage output   (V)
```

#### Struct
```c
typedef struct {
    float L, R, N, Kt, tau, Ts;  // motor parameters
    float b0, b1;                // discrete numerator
    float a0, a1;                // discrete denominator
    float u1;                    // u[k-1]
    float y1;                    // y[k-1]
} DistFF_t;
```

#### API
```c
// Compute discrete coefficients — call once at startup
// tau: filter time constant in the denominator (s) — NOT the disturbance torque
void DistFF_Init(DistFF_t *ff, float L, float R, float N, float Kt, float tau, float Ts);

// Zero delay-line state — call on reset_all, mode change, motor stop
void DistFF_Reset(DistFF_t *ff);

// One difference equation step — call at inner-loop rate (TIM6, 1 kHz)
// tau_d_hat : Kalman4 x_data[3]  (N·m)
// returns   : V_ff_d (V)  → convert to PWM: pwm_ff_d = V_ff_d / V_supply * 100.0f
float DistFF_Update(DistFF_t *ff, float tau_d_hat);
```

> **Status: Active in `main.c` (Mode 3 TIM6 ISR).** Works alongside `refff_enabled`.
> DistFF runs every tick; the addend is always written to `g_distff_pwm` for observation.
> Gate: `if (distff_enabled) pwm_out += g_distff_pwm;`
>
> **Backlash warning:** if DistFF causes oscillation at zero speed, Kalman4 is
> misattributing gear backlash motion as τ_d and DistFF overcorrects. Fix:
> set `pos_deadband_deg` ≥ backlash amplitude so the hard stop fires before the
> correction cycle builds up. Also check `distff_tau` — increase to slow DistFF response.

---

## 5. main.c Architecture

### 5.1 Startup Sequence (USER CODE BEGIN 2)
```c
// TRAJ_DT = 0.001f (trajectory Update runs in TIM6 at 1 kHz)
SCurve_Init(&my_scurve, max_velocity, max_accel, max_jerk, TRAJ_DT);
Trapezoid_Init(&my_trapezoid, max_velocity, max_accel, TRAJ_DT);

HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
__HAL_TIM_SET_COUNTER(&htim1, 0);    // zero counter FIRST
Encoder_Init(&my_encoder, LOOP_DT);  // then init struct (last_cnt=0 matches counter)
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

// Tustin-discretised reference feedforward
RefFF_Init(&my_refff, motor_J_eq, motor_L, motor_R_arm, motor_b_eq,
           motor_N, motor_Kt, motor_Ke, 0.02f, LOOP_DT);

// Disturbance feedforward — converts Kalman4 τ_d → compensating voltage
DistFF_Init(&my_distff, motor_L, motor_R_arm, motor_N, motor_Kt, distff_tau, LOOP_DT);

// 4-state motor Kalman filter
Kalman4_Init(&my_kf4, 0.0f, 0.0f, 0.0f, LOOP_DT,
             motor_J_eq, motor_b_eq, motor_N, motor_Kt, motor_Ke,
             motor_L, motor_R_arm, kf4_sigma_v2, kf4_r_theta);

HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED, ADC_CALIB_OFFSET);
HAL_ADC_Start(&hadc1);

MX_TIM7_Init();
HAL_TIM_Base_Start_IT(&htim7);
HAL_TIM_Base_Start_IT(&htim6);
```

### 5.2 Main Loop (while(1))
```c
// UART TX + GPIO — all control happens in ISR
if (tx_pending) {
    tx_pending = 0;
    HAL_UART_Transmit(&hlpuart1, tx_buf, 68, 2);  // blocking, ~272µs at 2Mbaud
}
// Gripper GPIO (driven here — acceptable latency for gripper)
HAL_GPIO_WritePin(EMER_OUTPUT_relay_GPIO_Port, EMER_OUTPUT_relay_Pin,
                  emer_output ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GRIPPER_UPDOWN_GPIO_Port,   GRIPPER_UPDOWN_Pin,
                  gripper_updown   ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GRIPPER_OPENCLOSE_GPIO_Port, GRIPPER_OPENCLOSE_Pin,
                  gripper_openclose ? GPIO_PIN_SET : GPIO_PIN_RESET);
```

### 5.3 ISR Control Loops (HAL_TIM_PeriodElapsedCallback)

**TIM6 @ 1 kHz — inner loop (priority 0)**
```
① Encoder_Update() — 16-bit TIM1 counter, 10-tick windowed velocity
② ADC read (PA1 / ADC1_IN2): spin on ISR_EOC, g_current_A = (raw − zero)/scale
③ Rebuild Q matrix (second-order expansion from kf4_sigma_v2 + motor params)
   Set R[0] = kf4_r_theta
   Kalman4_Update(&my_kf4, position_rad, motor_voltage)
   g_kf4_position/velocity/current/tau_d_obs updated
④ Trajectory Update (Mode 3 only):
   SCurve_Update() OR Trapezoid_Update()
   → writes g_traj_pos, g_smooth_vel, g_smooth_accel, g_jerk, g_traj_active
⑤ if hard_stop_active || !pid_enabled → Velocity_PID_Reset() + Set_Motor_PWM(0)
   else switch(control_mode):
     Mode 3 → fb_vel = use_kf4_vel ? g_kf4_velocity : encoder.velocity_rad_s
              vel_error = vel_command − fb_vel
              pwm = Velocity_PID_Controller(vel_error)
              if refff_enabled: pwm += RefFF_Update(vel_command)/V_supply×100 %
              v_ff_d = DistFF_Update(&my_distff, g_kf4_tau_d_obs)
              g_distff_pwm = v_ff_d / V_supply × 100 %
              if distff_enabled: pwm += g_distff_pwm
              Set_Motor_PWM(clamp(pwm, ±max_pwm))
     Mode 7 → Set_Motor_PWM(pwm_command)  [direct from TIM7]
              repurposes g_traj_pos/vel/accel/jerk for Kalman4 state in telemetry
     default → Set_Motor_PWM(0)
⑥ g_position_rad, g_velocity_rad_s, g_pwm_duty updated
⑦ Pack_Telemetry() + set tx_pending   (72 bytes)
```

**TIM7 @ 500 Hz — outer loop (priority 1)**
```
① reset_all: re-init SCurve, Trapezoid, Encoder, RefFF, Kalman4, DistFF;
   clear all globals, abort seq/sweep; force mode-change handler next tick
② apply_motor_params: re-init RefFF, Kalman4, DistFF (position not zeroed)
③ zero_encoder: reset TIM1 counter + encoder struct + Kalman4 state
④ Mode / traj_type change: reset PIDs, Kalman4_Reset, Start_Trajectory
⑤ Sequence state machine (seq_enabled): manage 9-target array, detect settle
⑥ Sweep state machine (sweep_enabled): generate targets on-the-fly 0→T→0→…
⑦ traj_target_deg change / start_move / pid_enabled rising-edge handling
⑧ if !pid_enabled → zero commands and return
⑨ switch(control_mode):
     Mode 3 → read g_traj_pos, g_smooth_vel, g_traj_active from TIM6
              pos_error = g_traj_pos - encoder.position_rad
              vel_correct = Position_PID_Controller(pos_error, encoder.velocity_rad_s)
              vel_cmd = g_smooth_vel + vel_correct  (clamped ±max_velocity)
              if !g_traj_active && ss_reached:
                vel_command = 0, hard_stop_active = 1  (TIM6 holds zero)
              else: vel_command = vel_cmd, hard_stop_active = 0  (non-sticky)
     Mode 7 → generate kf_cal_pwm ± sine, write pwm_command
     default → vel_command=0, pwm_command=0, hard_stop_active=1
```

### 5.4 Start_Trajectory Helper
```c
static void Start_Trajectory(float displacement) {
    traj_start_pos = my_encoder.position_rad;  // save absolute start
    float abs_disp = fabsf(displacement);

    if (traj_type == TRAJ_TRAPEZOID) {
        Trapezoid_Init(&my_trapezoid, max_velocity, max_accel, TRAJ_DT);
        // t_acc_seg, t_cruise_seg, t_dec_seg are user-specified in Live Expressions.
        // v_peak = |d| / (t_acc/2 + t_cruise + t_dec/2) — back-computed inside ByTime.
        // v_peak clamped to max_velocity only if it exceeds it; t_cruise unchanged.
        Trapezoid_SetTarget_ByTime(&my_trapezoid, displacement,
                                   t_acc_seg, t_cruise_seg, t_dec_seg);
    } else {
        SCurve_Init(&my_scurve, max_velocity, max_accel, max_jerk, TRAJ_DT);
        // S-curve: t_cruise auto-computed from max_velocity (t_cruise_seg is read-only here)
        float t_cruise = (max_velocity > 0)
                         ? (abs_disp/max_velocity - 2.0f*t1_seg - t2_seg) : 0;
        if (t_cruise < 0) t_cruise = 0;
        t_cruise_seg = t_cruise;  // overwrite — S-curve t_cruise_seg is read-only
        SCurve_SetTarget_ByTime(&my_scurve, displacement, t1_seg, t2_seg, t_cruise);
    }
    Velocity_PID_Reset();
    Position_PID_Reset();
}
```

> **Critical:** displacement ≠ absolute target. It's `target − current_position`.

> **Trapezoid:** `t_cruise_seg` is **writable** — set it directly in Live Expressions. `Start_Trajectory` passes it straight to `Trapezoid_SetTarget_ByTime` without modification.
> **S-curve:** `t_cruise_seg` is **read-only** — auto-computed as `|d|/max_velocity − 2×t1 − t2` and overwritten every `Start_Trajectory` call.

### 5.5 Set_Motor_PWM
```c
void Set_Motor_PWM(float pwm_duty) {
    // Direction → GPIO PC7 (MOTOR_DIR_Pin = GPIO_PIN_7, GPIOC)
    if (pwm_duty >= 0) HAL_GPIO_WritePin(MOTOR_DIR_Pin_GPIO_Port, MOTOR_DIR_Pin_Pin, GPIO_PIN_SET);
    else { HAL_GPIO_WritePin(..., GPIO_PIN_RESET); pwm_duty = -pwm_duty; }

    // Minimum threshold — overcomes motor static friction
    if (pwm_duty > 0.0f && pwm_duty < min_pwm_threshold)
        pwm_duty = min_pwm_threshold;

    // Map 0-100% → TIM3 CCR 0-1000
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                           (uint32_t)(pwm_duty / 100.0f * 1000.0f));
}
```

---

## 6. Control Modes

Set `control_mode` in Live Expressions. Changing mode resets PIDs and restarts trajectory.

> **Only Mode 3 and Mode 7 have active logic.** All other values fall through to the default
> case which outputs zero PWM. Modes 1, 2, 4, 5, 6 are not implemented in the current code.

### Mode 3 — Full Cascade (primary operating mode)
```
Arming (TIM7): Start_Trajectory(disp) → arms SCurve or Trapezoid profile
Update (TIM6, 1 kHz): SCurve_Update / Trapezoid_Update
  → writes g_traj_pos, g_smooth_vel, g_traj_active to volatile globals

TIM7 @ 500 Hz — Position PID:
  pos_error = g_traj_pos − encoder.position_rad
  vel_correct = Position_PID(pos_error, encoder.velocity_rad_s)
  vel_cmd = g_smooth_vel + vel_correct  (clamped ±max_velocity)

  Hard stop (non-sticky):
    if (!g_traj_active && ss_reached):
      vel_command = 0, hard_stop_active = 1
    else:
      vel_command = vel_cmd, hard_stop_active = 0

TIM6 @ 1 kHz — Velocity PID:
  fb_vel = use_kf4_vel ? g_kf4_velocity : encoder.velocity_rad_s
  vel_error = vel_command − fb_vel
  pwm = Velocity_PID(vel_error)
  if refff_enabled: pwm += RefFF_Update(vel_command) / V_supply × 100 %
  g_distff_pwm = DistFF_Update(my_distff, g_kf4_tau_d_obs) / V_supply × 100 %
  if distff_enabled: pwm += g_distff_pwm
  Set_Motor_PWM(clamp(pwm, ±max_pwm))
```

Sequence mode (`seq_enabled`) and sweep mode (`sweep_enabled`) work in Mode 3 only.

### Mode 7 — Kalman4 Calibration
```
TIM7: generate kf_cal_pwm ± sine excitation → pwm_command
TIM6: Set_Motor_PWM(pwm_command) [direct]
      Kalman4_Update() runs unconditionally as normal
      g_traj_pos/vel/accel/jerk repurposed → Kalman4 state channels in telemetry
```
- No trajectory, no PID — motor driven open-loop by `kf_cal_pwm` (−100 to +100 %)
- Sine excitation: `kf_sine_enabled=1`, set `kf_sine_amp/freq/dir` for swept identification
- Tune `kf4_sigma_v2` and `kf4_r_theta`, then `apply_motor_params=1` or `reset_all=1`
- Compare `g_velocity_rad_s` (bytes 22–25) vs `g_kf4_velocity` (bytes 46–49) on Simulink

**UART packet in Mode 7** (bytes 2–17 repurposed for Kalman4 state):
| Bytes | Variable | Content |
|-------|----------|---------|
| 2–5 | `g_traj_pos` slot | **Kalman4 position estimate** (rad) |
| 6–9 | `g_smooth_vel` slot | **Kalman4 velocity** (rad/s) |
| 10–13 | `g_smooth_accel` slot | **Kalman4 current estimate** (A) |
| 14–17 | `g_jerk` slot | **Kalman4 τ_d estimate** (N·m) |
| 18–21 | `g_position_rad` | actual encoder position (rad) — unchanged |
| 22–25 | `g_velocity_rad_s` | actual raw velocity (rad/s) — unchanged |
| 26–49 | Kalman4 state | same as Mode 3 (dedicated Kalman4 slots unchanged) |

---

## 7. Live Expressions Variables

### 7.1 Control & Mode
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `control_mode` | `uint8_t` | 3 | Mode selector — **3 = Cascade, 7 = Kalman4 Cal** |
| `traj_type` | `uint8_t` | 0 | 0=S-Curve, 1=Trapezoid |
| `traj_target_deg` | `float` | 180.0 | Target position (degrees) — **เปลี่ยนแล้วเริ่มใหม่อัตโนมัติ** |
| `start_move` | `uint8_t` | 0 | ตั้งเป็น 1 เพื่อ retrigger |
| `zero_encoder` | `uint8_t` | 0 | ตั้งเป็น 1 เพื่อ reset position=0 |
| `reset_all` | `uint8_t` | 0 | ตั้งเป็น 1 เพื่อ **clear ทุก state** — หยุด trajectory, reset PIDs ทั้งหมด, zero encoder, clear commands และ observables ทั้งหมด Motor หยุดทันที จากนั้น self-clears กลับเป็น 0 |
| `pid_enabled` | `uint8_t` | **1** | **0 = freeze** — PWM=0, PIDs reset, trajectory หยุด advance. **1 = resume** — control loop กลับมาทำงานปกติ ใช้ `start_move=1` หลัง re-enable เพื่อ restart trajectory |

### 7.2 Trajectory Profile Limits
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `max_velocity` | `float` | **6.28** | rad/s — peak velocity (always reached if displacement allows) |
| `max_accel` | `float` | **12.56** | rad/s² — used only for Init; profile shape set by time params |
| `max_jerk` | `float` | 10.0 | rad/s³ — S-curve only |

> เปลี่ยน parameter แล้ว **ต้อง trigger ใหม่** (เปลี่ยน traj_target_deg หรือ start_move=1) เพราะ profile คำนวณตอน SetTarget เท่านั้น

### 7.3 Trajectory Shape Parameters
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `t1_seg` | `float` | 0.1 | S-curve: jerk segment duration (s) |
| `t2_seg` | `float` | 0.1 | S-curve: const-accel duration (s) |
| `t_acc_seg` | `float` | 0.3 | Trapezoid: acceleration duration (s) |
| `t_cruise_seg` | `float` | 0.2 | **Trapezoid: cruise duration — writable** \| S-curve: read-only auto-computed |
| `t_dec_seg` | `float` | 0.3 | Trapezoid: deceleration duration (s) — may differ from t_acc_seg |

> **S-curve:** `t_cruise_seg` is auto-computed as `|d|/max_velocity − 2×t1 − t2` (read-only). Set `t1_seg`, `t2_seg` to control ramp shape; observe `t_cruise_seg` to verify.
> **Trapezoid:** `t_acc_seg`, `t_cruise_seg`, `t_dec_seg` are all **writable**. `v_peak` is back-computed from these times. `max_velocity` acts as a safety ceiling only — it does NOT determine `t_cruise_seg`.

### 7.3a Inter-Loop Interface (Read-only observables)
| Variable | Type | Description |
|----------|------|-------------|
| `vel_command` | `float` | Velocity set-point written by TIM7, consumed by TIM6 (Modes 3,5) |
| `pwm_command` | `float` | Direct PWM written by TIM7, applied by TIM6 (Modes 1,2,6) |
| `hard_stop_active` | `uint8_t` | 1 = TIM7 commanded hold-zero; TIM6 resets vel PID each cycle |

### 7.4 Velocity PID (Inner Loop)
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `kp_vel` | `float` | **30.0** | P gain — [%PWM]/[rad/s] |
| `ki_vel` | `float` | 0.1 | I gain |
| `kd_vel` | `float` | 0.0 | D gain — off by default due to encoder noise |
| `min_pwm_threshold` | `float` | 5.0 | %PWM — minimum to overcome static friction |
| `use_kf4_vel` | `uint8_t` | **0** | Velocity feedback source: **0** = raw encoder (`my_encoder.velocity_rad_s`) · **1** = Kalman4 (`g_kf4_velocity`) — applies to both velocity PID error and position PID kd term |

### 7.4a Kalman4
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `kf4_sigma_v2` | `float` | 1e-3 | σ_v² — voltage noise variance → drives Q second-order expansion |
| `kf4_r_theta` | `float` | 1e-6 | σ_θ² — position measurement noise variance |
| `g_kf4_velocity` | `float` | R/O | Kalman4 estimated velocity (rad/s) |
| `g_kf4_position` | `float` | R/O | Kalman4 estimated position (rad) |
| `g_kf4_current` | `float` | R/O | Kalman4 estimated current (A) — model-driven |
| `g_kf4_tau_d_obs` | `float` | R/O | Kalman4 estimated disturbance torque (N·m) — feeds DistFF |

> Apply: `apply_motor_params = 1` (position preserved) or `reset_all = 1`.

### 7.4b Sequence Mode
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `seq_targets[0]`…`seq_targets[8]` | `float[9]` | 0.0 | Target positions (deg) — fill in order; unused entries ignored |
| `seq_count` | `uint8_t` | 0 | Number of active steps (1–9) |
| `seq_enabled` | `uint8_t` | 0 | Write 1 to start sequence — self-clears when all steps complete; write 0 to abort |
| `seq_index` | `uint8_t` | R/O | Current step being executed (0-based) |
| `seq_step_delay` | `float` | 2.0 | Dwell time between steps (s) — set 0 for immediate advance |

> Works in Modes 1, 2, 3. Each step fires when the previous trajectory is complete **and** `|pos_error| ≤ pos_deadband_deg`, then waits `seq_step_delay` seconds before the next move. Set `pos_deadband_deg` to a non-zero value or the sequence will never advance. `reset_all` aborts the sequence.

### 7.4c Current Sensor — ADC1 IN2 (PA1)
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `current_zero_counts` | `float` | 2892.7 | ADC count at zero current — trim until `g_current_A ≈ 0` with motor stopped |
| `current_counts_per_amp` | `float` | 86.57 | ADC counts per ampere — adjust to match known current reference |
| `adc_raw_current` | `uint32_t` | R/O | Raw 12-bit ADC count (0–4095) |
| `g_current_A` | `float` | R/O | Computed motor current (A) — in UART packet bytes 38–41 |

> Formula: `I = (adc_raw_current − current_zero_counts) / current_counts_per_amp`
> ADC read in TIM6 ISR: `HAL_ADC_Stop` → `HAL_ADC_Start` → spin on `ADC_ISR_EOC` → read `hadc1.Instance->DR`.

### 7.4d Kalman4 Calibration (Mode 7)
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `kf_cal_pwm` | `float` | 0.0 | DC offset PWM for Mode 7 (−100 to +100 %) |
| `kf_sine_enabled` | `uint8_t` | 0 | 0=constant `kf_cal_pwm`, 1=sine wave excitation |
| `kf_sine_amp` | `float` | 10.0 | Sine amplitude (%) |
| `kf_sine_freq` | `float` | 1.0 | Sine frequency (Hz) |
| `kf_sine_dir` | `float` | 1.0 | +1=normal, −1=reversed |

> Kalman4 runs in **all modes** every TIM6 tick. Mode 7 provides controlled open-loop
> excitation for identification. Compare `g_velocity_rad_s` (bytes 22–25) vs
> `g_kf4_velocity` (bytes 46–49). After tuning, write `apply_motor_params = 1`.

### 7.4e Reference Feedforward
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `refff_enabled` | `uint8_t` | 0 | 0=off, 1=adds model-based PWM on top of vel PID |
| `V_supply` | `float` | 24.0 | Motor bus voltage (V) — must match actual supply |

### 7.4f Disturbance Feedforward
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `distff_enabled` | `uint8_t` | 1 | 0=observe only, 1=adds DistFF PWM on top of vel PID |
| `distff_tau` | `float` | 0.02 | Filter time constant (s) — apply with `apply_motor_params=1` |
| `g_distff_pwm` | `float` | R/O | DistFF PWM addend (%) — observe before enabling |

> Check `g_distff_pwm` magnitude at rest before enabling. If oscillation occurs near
> zero speed (gear backlash), set `pos_deadband_deg` ≥ backlash amplitude.

### 7.4g Motor Parameters (Live Tunable)
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `motor_J_eq` | `float` | 0.027 | Equivalent inertia at output shaft (kg·m²) |
| `motor_b_eq` | `float` | 0.5 | Equivalent viscous friction (N·m·s/rad) |
| `motor_N` | `float` | 50.0 | Gear ratio |
| `motor_Kt` | `float` | 0.00747 | Torque constant (N·m/A) |
| `motor_Ke` | `float` | 0.0083 | Back-EMF constant (V·s/rad) |
| `motor_L` | `float` | 0.0012794 | Armature inductance (H) |
| `motor_R_arm` | `float` | 2.8 | Armature resistance (Ω) |
| `apply_motor_params` | `uint8_t` | 0 | Write 1 to re-init RefFF, Kalman4, DistFF with new values — does **not** zero encoder — self-clears |

> Change any motor param, then write `apply_motor_params = 1` to apply without
> resetting position. `reset_all = 1` also applies current motor params but zeros encoder.

### 7.4h Sweep Mode
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `sweep_enabled` | `uint8_t` | 0 | Write 1 to start, 0 to abort — self-clears when done |
| `sweep_start_deg` | `float` | 5.0 | First non-zero target (deg) |
| `sweep_stop_deg` | `float` | 360.0 | Last non-zero target (deg) |
| `sweep_step_deg` | `float` | 5.0 | Increment per step (deg) |
| `sweep_delay_s` | `float` | 2.0 | Dwell time at each position (s) |
| `sweep_index` | `uint8_t` | R/O | Current step index (0-based) |

> Example: `start=5, stop=360, step=5` → 144 total moves: 0→5→0→10→…→0→360.
> Advances when trajectory done AND `|pos_error| ≤ pos_deadband_deg`.
> `reset_all` aborts sweep. Works in Mode 3 only.

### 7.5 Position PID (Outer Loop — Mode 3)
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `kp_position` | `float` | 1.0 | P gain [rad/s per rad] |
| `ki_pos` | `float` | 0.0 | I gain [rad/s per rad·s] |
| `kd_pos` | `float` | 0.0 | D gain via −velocity [rad/s per rad/s] |
| `pos_deadband_deg` | `float` | 2.0 | Hard-stop settle threshold (deg) — set ≥ backlash amplitude |
| `pos_integral_live` | `float` | R/O | Position integrator — watch for windup |
| `ss_error_deg` | `float` | R/O | Steady-state position error (deg) |
| `ss_error_rad` | `float` | R/O | Steady-state position error (rad) |
| `ss_reached` | `uint8_t` | R/O | 1 when `\|pos_error\| ≤ pos_deadband_deg` |
| `ss_final_error_deg` | `float` | R/O | Error frozen when hard stop last fired |

### 7.6 Motor Output
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `motor_dir_inverted` | `uint8_t` | 0 | Set to **1** if actual motion is opposite the trajectory — flips PWM sign, takes effect immediately |
| `min_pwm_threshold` | `float` | 5.0 | %PWM — minimum to overcome friction |

### 7.7 Debug Observables (Read-only)
| Variable | Description |
|----------|-------------|
| `g_position_rad` | Actual encoder position (rad) |
| `g_velocity_rad_s` | Raw encoder velocity (rad/s) — 10-tick windowed |
| `g_current_A` | Measured motor current (A) |
| `adc_raw_current` | Raw ADC count (0–4095) — verify sensor connected |
| `g_smooth_vel` | Trajectory velocity (rad/s) |
| `g_smooth_accel` | Trajectory acceleration (rad/s²) |
| `g_jerk` | Trajectory jerk (rad/s³; 0 for trapezoid) |
| `g_traj_pos` | Trajectory ideal position (rad) |
| `g_pwm_duty` | PWM output (%) |
| `g_motor_voltage` | Estimated motor voltage (V) — used by Kalman4 |
| `vel_error_live` | Current velocity error into PID (rad/s) |
| `i_term_live` | Velocity PID integral accumulator |
| `g_kf4_position` | Kalman4 estimated position (rad) |
| `g_kf4_velocity` | Kalman4 estimated velocity (rad/s) |
| `g_kf4_current` | Kalman4 estimated current (A) — model only |
| `g_kf4_tau_d_obs` | Kalman4 disturbance torque (N·m) |
| `g_distff_pwm` | DistFF PWM addend (%) — observe before enabling |

---

## 8. UART Telemetry → Simulink

### Port Settings
| Setting | Value |
|---------|-------|
| Port | COMx (STMicroelectronics STLink — check Device Manager) |
| Baud rate | **2,000,000** |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |

### Packet Format (72 bytes, sent at 1 kHz)
```
Byte  0– 1 : 0x7E 0x7E         — header
Byte  2– 5 : g_traj_pos         float32 LE  trajectory ideal position   (rad)
Byte  6– 9 : g_smooth_vel       float32 LE  trajectory velocity         (rad/s)
Byte 10–13 : g_smooth_accel     float32 LE  trajectory acceleration     (rad/s²)
Byte 14–17 : g_jerk             float32 LE  trajectory jerk             (rad/s³; 0 for trapezoid)
Byte 18–21 : g_position_rad     float32 LE  actual encoder position     (rad)
Byte 22–25 : g_velocity_rad_s   float32 LE  raw encoder velocity        (rad/s)
Byte 26–29 : g_kf4_velocity     float32 LE  Kalman4 velocity (backwards-compat slot) (rad/s)
Byte 30–33 : g_kf4_position     float32 LE  Kalman4 position (backwards-compat slot) (rad)
Byte 34–37 : ss_error_rad       float32 LE  steady-state position error (rad)
Byte 38–41 : g_current_A        float32 LE  measured motor current      (A)
Byte 42–45 : g_kf4_position     float32 LE  Kalman4 estimated position  (rad)
Byte 46–49 : g_kf4_velocity     float32 LE  Kalman4 estimated velocity  (rad/s)
Byte 50–53 : g_kf4_current      float32 LE  Kalman4 estimated current   (A)
Byte 54–57 : g_kf4_tau_d_obs    float32 LE  Kalman4 disturbance torque  (N·m)
Byte 58–61 : g_motor_voltage    float32 LE  motor commanded voltage     (V)
Byte 62–65 : adc_raw_current    uint32  LE  raw ADC count               (0–4095)
Byte 66–69 : i_term_live        float32 LE  velocity integrator accumulator
Byte 70–71 : 0x03 0x03          — footer
```
> Bytes 26–33 carry Kalman4 vel/pos (formerly 2-state Kalman slots — filter removed).
> Bytes 42–49 also carry Kalman4 pos/vel — dedicated slots.
> No checksum. Validate by checking header (0x7E 0x7E) and footer (0x03 0x03).

### Architecture (non-blocking)
```
ISR (1 kHz):  Pack_Telemetry() → set tx_pending=1
while(1):     if tx_pending → HAL_UART_Transmit (blocking ~272µs @ 2 Mbaud)
```

### Simulink Parser (MATLAB Function block)
```matlab
function [traj_pos, smooth_vel, smooth_accel, jerk, ...
          act_pos, act_vel, kf_vel, kf_pos, ss_error, current, ...
          kf4_pos, kf4_vel, kf4_current, kf4_tau_d, voltage, adc_raw, valid] = parse(raw)
    % Validate sync header and end marker — no checksum in this format
    valid = (raw(1)==hex2dec('7E')) && (raw(2)==hex2dec('7E')) && ...
            (raw(67)==hex2dec('03')) && (raw(68)==hex2dec('03'));
    traj_pos    = typecast(uint8(raw(3:6)),   'single');   % trajectory position (rad)
    smooth_vel  = typecast(uint8(raw(7:10)),  'single');   % trajectory velocity (rad/s)
    smooth_accel= typecast(uint8(raw(11:14)), 'single');   % trajectory accel (rad/s²)
    jerk        = typecast(uint8(raw(15:18)), 'single');   % trajectory jerk (rad/s³)
    act_pos     = typecast(uint8(raw(19:22)), 'single');   % actual encoder position (rad)
    act_vel     = typecast(uint8(raw(23:26)), 'single');   % raw encoder velocity (rad/s)
    kf_vel      = typecast(uint8(raw(27:30)), 'single');   % Kalman4 velocity estimate (rad/s) — backwards-compat slot
    kf_pos      = typecast(uint8(raw(31:34)), 'single');   % Kalman4 position estimate (rad)  — backwards-compat slot
    ss_error    = typecast(uint8(raw(35:38)), 'single');   % steady-state position error (rad)
    current     = typecast(uint8(raw(39:42)), 'single');   % motor current (A)
    kf4_pos     = typecast(uint8(raw(43:46)), 'single');   % Kalman4 position estimate (rad)
    kf4_vel     = typecast(uint8(raw(47:50)), 'single');   % Kalman4 velocity (rad/s)
    kf4_current = typecast(uint8(raw(51:54)), 'single');   % Kalman4 current estimate (A)
    kf4_tau_d   = typecast(uint8(raw(55:58)), 'single');   % Kalman4 disturbance torque (N·m)
    voltage     = typecast(uint8(raw(59:62)), 'single');   % motor commanded voltage (V)
    adc_raw     = typecast(uint8(raw(63:66)), 'uint32');   % raw ADC count (0–4095)
end
```

---

## 9. Trajectory Generation

### How Target → Profile Works
```
1. User sets traj_target_deg = 180 (degrees)
2. TIM7 detects change (within 2 ms), calls Start_Trajectory(displacement)
   - saves traj_start_pos = encoder.position_rad
   - S-curve:   t_cruise = |d|/max_velocity − 2×t1 − t2 (clamped ≥ 0), writes t_cruise_seg
   - Trapezoid: t_acc_seg, t_cruise_seg, t_dec_seg used directly
   - calls SCurve_SetTarget_ByTime() or Trapezoid_SetTarget_ByTime()
3. Each TIM6 tick (1 ms): SCurve_Update() or Trapezoid_Update()
   → g_traj_pos, g_smooth_vel, g_smooth_accel, g_jerk, g_traj_active written to volatile globals
4. Each TIM7 tick (2 ms): Position PID reads g_traj_pos, g_smooth_vel from TIM6
   pos_error = g_traj_pos − encoder.position_rad
   → writes vel_command
5. Each TIM6 tick: reads vel_command → velocity PID → PWM
6. When g_traj_active → 0 AND ss_reached: TIM7 sets hard_stop_active=1
```

### Position Reference in Mode 3
```c
// p_current is RELATIVE (starts at 0 each move)
// traj_start_pos is ABSOLUTE (captured at move start)
float pos_abs_ideal = traj_start_pos + p_current;  // absolute ideal position
float pos_error = pos_abs_ideal - my_encoder.position_rad;
```

> **ถ้าไม่ใช้ traj_start_pos:** pos_error = 0 - actual_pos (huge!) → vel_correct saturates → trajectory ไม่ทำงาน

### Minimum Displacement for Cruise Phase (t_cruise > 0)
| Profile | How cruise is determined |
|---------|--------------------------|
| **Trapezoidal** | Set `t_cruise_seg > 0` directly in Live Expressions |
| **S-Curve** | Auto-computed: cruise exists when `\|d\| > max_velocity × (2×t1 + t2)` |

**Trapezoid:** set `t_cruise_seg = 0` for a triangle (accel + decel only). v_peak is still back-computed from the given times.
**S-Curve:** ถ้า displacement น้อยเกินไป → t_cruise clamps to 0 → parabola profile with v_peak < max_velocity — ถูกต้อง ไม่ใช่ bug.

---

## 10. PID Controllers

### 10.1 Velocity PID (Inner Loop)

```c
float Velocity_PID_Controller(float error) {
    float p_out = kp_vel * error;

    // Anti-windup: clamp ACCUMULATOR (not just output)
    vel_integral += error * LOOP_DT;
    float max_int = (ki_vel > 0) ? (max_pwm / ki_vel) : 0;
    vel_integral = clamp(vel_integral, -max_int, +max_int);
    float i_out = ki_vel * vel_integral;

    float d_out = kd_vel * (error - vel_prev_error) / LOOP_DT;
    vel_prev_error = error;

    return clamp(p_out + i_out + d_out, -max_pwm, +max_pwm);
}
```

**kp_vel sizing:**
```
kp_vel = max_pwm / max_velocity = 100 / 1.57 ≈ 64  (full authority)
Start at 30 → tune up
```

### 10.2 Position PID (Outer Loop)

```c
float Position_PID_Controller(float pos_error, float vel_feedback) {
    float p_out = kp_position * pos_error;

    // Anti-windup — uses POS_DT (0.002 s) because this runs in TIM7 @ 500 Hz
    pos_integral += pos_error * POS_DT;
    float max_int = (ki_pos > 0) ? (max_velocity / ki_pos) : 0;
    pos_integral = clamp(pos_integral, -max_int, +max_int);
    float i_out = ki_pos * pos_integral;

    // D term uses NEGATIVE velocity (not Δerror/dt)
    // Reason: d(pos_error)/dt = -d(actual)/dt = -velocity → less noisy
    float d_out = -kd_pos * vel_feedback;

    return p_out + i_out + d_out;  // output = velocity command (rad/s)
}
```

### 10.3 PID Resets

| Reset | Called from | When |
|-------|------------|------|
| `Velocity_PID_Reset()` | **TIM6** (via `hard_stop_active=1`) | Hard stop, mode change |
| `Velocity_PID_Reset()` | **TIM7** | Trajectory completion (is_active 1→0), mode change, zero_encoder |
| `Position_PID_Reset()` | **TIM7** | All of the above (TIM7 owns position PID state) |

> TIM7 owns `pos_integral`. TIM6 owns `vel_integral`. No cross-ISR writes except during mode change (transient inconsistency < 2 µs, benign).

---

## 11. Bugs Fixed in This Session

| # | Bug | Fix |
|---|-----|-----|
| 1 | `Set_Motor_PWM` direction pin = `0` (not `GPIO_PIN_0=0x0001`) | เปลี่ยนเป็น `MOTOR_DIR_Pin_Pin` |
| 2 | TIM1 ARR=8191 → signed delta fails at wrap | เปลี่ยน ARR เป็น 65535 |
| 3 | `Encoder_Init` ก่อน `HAL_TIM_Encoder_Start` → first delta spurious | สลับลำดับ: Start → Zero counter → Init |
| 4 | `Start_Trajectory` ส่ง absolute target แทน displacement | เปลี่ยนเป็น `target - current_pos` |
| 5 | Mode 3 position error: `p_current - actual_pos` (relative vs absolute) | ใช้ `traj_start_pos + p_current` |
| 6 | Anti-windup clamp output ไม่ clamp accumulator → integral windup | Clamp `vel_integral` accumulator โดยตรง |
| 7 | Hard-stop ไม่ reset position integral → motor drift | เพิ่ม `Position_PID_Reset()` ใน hard-stop |
| 8 | `kp_vel = 1.0` → max PWM = 1.57% → motor ไม่ขยับ | เปลี่ยน default เป็น 30.0 |
| 9 | Mode 2 ไม่รัน trajectory (ใช้ raw target) | เพิ่ม trajectory Update() ใน Mode 2 |
| 10 | `traj_target_deg` ไม่ auto-trigger ใน Mode 2 | เพิ่ม Mode 2 ใน auto-retrigger handler |
| 11 | `kp_position=1.0` ใน Mode 2 → 3.14% PWM ที่ 180° | เพิ่ม `kp_pos_pwm=20.0` แยกสำหรับ Mode 2 |
| 12 | Velocity noise 0.767 rad/s/count → jiggle | Windowed velocity (10-tick average) |
| 13 | `kd_vel=0.05` → D spike 38% PWM ต่อ tick | Set `kd_vel=0.0` |
| 14 | `MAX_VELOCITY/ACCEL/JERK` เป็น #define → reflash ทุกครั้ง | เปลี่ยนเป็น `volatile float` |

---

## 12. Tuning Guide

### ลำดับการ Tune (Mode 3)

```
Step 1: ปิด I และ D ก่อน
    ki_vel = 0, kd_vel = 0
    ki_pos = 0, kd_pos = 0

Step 2: Tune kp_vel (velocity inner loop)
    เริ่มที่ 30, เพิ่มจน motor ตาม trajectory ได้
    ถ้า oscillate → ลด
    Target: g_velocity_rad_s ≈ g_smooth_vel

Step 3: Tune kp_position (position outer loop)
    เริ่มที่ 1.0, เพิ่มจน motor หยุดตรงเป้า
    ถ้า oscillate → ลด
    ดู ss_error_deg เพื่อตรวจสอบ

Step 4: Tune ki_vel ถ้ามี steady-state velocity error
    เริ่มที่ 0.1, เพิ่มทีละ 0.05

Step 5: เปิด refff_enabled = 1 ถ้า velocity ยัง lag ช่วง ramp
    ตั้ง V_supply ให้ตรงกับ bus voltage จริง (เช่น 24.0)
    feedforward จะลด PID effort ช่วง acceleration

Step 6: Tune Kalman4 ถ้าต้องการ DistFF หรือ τ_d estimation
    ใช้ Mode 7 + sine excitation (kf_sine_enabled=1)
    เปรียบเทียบ g_velocity_rad_s กับ g_kf4_velocity บน Simulink (bytes 22-25 vs 46-49)
    เพิ่ม kf4_sigma_v2 = faster ω/τ_d response แต่ noisy
    เพิ่ม kf4_r_theta = smoother θ estimate แต่ lag
    apply: apply_motor_params = 1

Step 7: เปิด distff_enabled = 1 ถ้าต้องการ disturbance compensation
    ก่อนเปิด: ดู g_distff_pwm ว่า magnitude ไม่ oversized
    ถ้า oscillate → เพิ่ม pos_deadband_deg ≥ backlash amplitude
    หรือ เพิ่ม distff_tau (slow DistFF response)

Step 8: ปรับ min_pwm_threshold
    หาค่าที่ motor พอดีเริ่มหมุน (dead zone boundary)
```

### Diagnosis ผ่าน Live Expressions
```
Motor ไม่ขยับ:
  → ดู g_pwm_duty — ถ้า < 5% → kp_vel ต่ำเกินไป หรือ min_pwm_threshold น้อยเกิน

Motor jiggle:
  → kd_vel ต้องเป็น 0 (ปัญหา encoder noise)
  → ลด kp_vel, ลด kp_position

Motor ไม่หยุดตรงเป้า:
  → ss_error_deg บอก error เป็น degree
  → ถ้า error < pos_deadband_deg → ลด deadband
  → ถ้า error > pos_deadband_deg → เพิ่ม ki_pos ทีละน้อย

Velocity ไม่ match trajectory:
  → เพิ่ม kp_vel
  → เปิด refff_enabled = 1 (reference feedforward ช่วยช่วง ramp)
  → ถ้า velocity noisy → tune kf4_sigma_v2 / kf4_r_theta ใน Mode 7
```

### Parameter Quick Reference
```
Minimum to move:  kp_vel ≥ 15, min_pwm_threshold ≥ motor dead zone
Good starting:    kp_vel = 30, kp_position = 1.0, ki_vel = 0.1, refff_enabled = 0, distff_enabled = 0
Cruise phase:     target > 180° (trapezoid) หรือ > 270° (S-curve) at default params
                  (t_cruise_seg > 0 in Live Expressions confirms cruise exists)
Trajectory shape: tune t1_seg/t2_seg/t_acc_seg — t_cruise_seg is auto-computed
```

---

## 13. DC Motor State-Space Model

### Geared DC Motor with Disturbance State
**States:** `x = [θ_l, θ̇_l, i, τ_d]ᵀ`

```
⎡ θ̇_l ⎤   ⎡  0       1       0      0  ⎤ ⎡θ_l ⎤   ⎡  0  ⎤
⎢ θ̈_l ⎥ = ⎢  0     -B/J   NKt/J  -1/J ⎥ ⎢θ̇_l ⎥ + ⎢  0  ⎥ · V
⎢  i̇  ⎥   ⎢  0  -NKe/L   -R/L    0   ⎥ ⎢ i  ⎥   ⎢ 1/L ⎥
⎣ τ̇_d ⎦   ⎣  0       0       0      0  ⎦ ⎣τ_d ⎦   ⎣  0  ⎦

y = [1 0 0 0] · x    (output: load angle)
```

| Symbol | Meaning |
|--------|---------|
| `J = Jm·N² + Jl` | Total inertia at load shaft |
| `N` | Gear ratio (motor→load, N>1 = reduction) |
| `Kt` | Torque constant (Nm/A) |
| `Ke` | Back-EMF constant (V·s/rad) |
| `B = Bm·N² + Bl` | Total damping at load shaft |
| `R` | Armature resistance (Ω) |
| `L` | Armature inductance (H) |
| `τ_d` | Disturbance torque (random walk model) |

### Transfer Functions (τ_d = 0, L→0 simplified)
```
G_V(s) = ω_l(s)/V(s) = Km / (τm·s + 1)

Km = N·Kt / (B·R + N²·Kt·Ke)    [rad/s per V]
τm = J·R  / (B·R + N²·Kt·Ke)    [s]
```

### Reference Feedforward — Implemented in `REF_FEEDFORWARD.c`

Full 2nd-order Tustin-discretised feedforward. Transfer function:
```
G_FF(s) = [J·L·s² + (J·R + b·L)·s + (b·R + N²·Kt·Ke)] / [N·Kt·(τ²·s² + 2τ·s + 1)]
```

Bilinear discretisation (Tustin, c = 2/Ts):
```
b0 = n2·c² + n1·c + n0      a0 = d2·c² + d1·c + d0
b1 = −2·n2·c² + 2·n0        a1 = −2·d2·c² + 2·d0
b2 = n2·c² − n1·c + n0      a2 = d2·c² − d1·c + d0

y[k] = (b0·u[k] + b1·u[k-1] + b2·u[k-2] − a1·y[k-1] − a2·y[k-2]) / a0
```

Usage: `RefFF_Update(&my_refff, vel_command)` → returns voltage (V).  
PWM addend: `pwm_ff = voltage / V_supply × 100 %`.

Enable live: `refff_enabled = 1`, `V_supply = <actual bus voltage>`.

---

## 14. Known Limitations & Future Work

### ยังไม่ได้ทำ (Future Work)
| Item | Description |
|------|-------------|
| Current sensor → Kalman4 feedback | ADC current reading on PA1 is implemented but **not yet fed into Kalman4**. To activate: expand to `KALMAN4_MEAS_DIM=2`, H→[[1,0,0,0],[0,0,1,0]], R→diag(r_theta,r_current), pass `g_current_A` to `Kalman4_Update`. |
| Current sensor → Kalman4 feedback | ADC current reading on PA1 is implemented but **not yet fed into Kalman4**. To activate: expand to `KALMAN4_MEAS_DIM=2`, H→[[1,0,0,0],[0,0,1,0]], R→diag(r_theta,r_current), pass `g_current_A` to `Kalman4_Update`. |
| Button B1 cycling | PC13 EXTI configured but not implemented |
| UART receive | LPUART1 configured full duplex แต่รับไม่ได้ยัง |
| Multi-turn homing | ไม่มี limit switch หรือ absolute position reference |
| Kalman model params | Motor-model G matrix currently uses simple u=0; add voltage input path |

### ข้อควรระวัง
1. **`traj_start_pos`** ต้องถูก capture ตอน SetTarget เท่านั้น — ถ้า encoder drift ระหว่าง move จะมี position error
2. **`max_velocity` เปลี่ยน mid-move** ไม่มีผล — trajectory คำนวณแล้วตอน SetTarget
3. **Velocity filter lag** 5ms (window) + IIR lag → ถ้า alpha ต่ำเกินอาจทำให้ PID oscillate
4. **`min_pwm_threshold`** ทำงานทุก mode — ถ้าต้องการ stop แบบ smooth ที่ใกล้ 0% ต้อง set = 0
5. **Mode 2 `kp_pos_pwm`** มี unit ต่างจาก `kp_position` — อย่า confuse
6. **`t_cruise_seg` is read-only** — ค่าที่ type ใน Live Expressions จะถูก overwrite ทุกครั้งที่ Start_Trajectory ถูกเรียก
7. **Dual-loop phase offset** — TIM7 (500 Hz) อาจถูก preempt โดย TIM6 (1 kHz) ระหว่าง position calculation → `vel_filtered` ที่ TIM7 อ่านอาจ stale ได้ 1 ms ซึ่งถือว่า acceptable
8. **Short moves** (< cruise threshold) — `t_cruise_seg = 0` และ `v_peak < max_velocity` — ปกติ ไม่ใช่ bug

### Performance Numbers (at default params)
```
Inner loop (TIM6):   1000 Hz — encoder, ADC, Kalman4, trajectory update, vel PID, DistFF
Outer loop (TIM7):   500 Hz  — position PID, seq/sweep, Start_Trajectory arming
Velocity resolution: 0.077 rad/s (10-tick windowed)
Position resolution: 0.000767 rad = 0.044°
UART packet rate:    1000 packets/sec = 68 kB/s (packed in TIM6)
Max travel:          ±2^31 counts / 8192 counts/rev ≈ ±262,144 revolutions
Trajectory latency:  up to 2 ms (TIM7 period) from target change to profile arm;
                     trajectory update starts next TIM6 tick (≤ 1 ms after arm)
```

---

## Appendix: Source File Summary

| File | Purpose | Key Symbols |
|------|---------|-------------|
| `main.c` | All application logic | `HAL_TIM_PeriodElapsedCallback`, `Start_Trajectory`, `Set_Motor_PWM`, `Velocity_PID_Controller`, `Position_PID_Controller`, `Pack_Telemetry` |
| `ENCODER.c` | Windowed velocity encoder | `Encoder_Init`, `Encoder_Update` |
| `SCURVE.c` | 7-segment S-curve | `SCurve_Init`, `SCurve_SetTarget`, `SCurve_SetTarget_ByTime`, `SCurve_Update` |
| `TRAPEZOID.c` | 3-segment trapezoidal | `Trapezoid_Init`, `Trapezoid_SetTarget`, `Trapezoid_SetTarget_ByTime`, `Trapezoid_Update` |
| `REF_FEEDFORWARD.c` | Tustin-discretised model feedforward | `RefFF_Init`, `RefFF_Reset`, `RefFF_Update` |
| `Kalman4.c` | 4-state motor Kalman filter (pos, vel, current, τ_d) | `Kalman4_Init`, `Kalman4_Reset`, `Kalman4_Update` |
| `DistFF.c` | Disturbance feedforward (τ_d estimate → voltage) | `DistFF_Init`, `DistFF_Reset`, `DistFF_Update` |
| `stm32g4xx_hal_msp.c` | HAL callbacks (CubeMX generated) | GPIO, TIM, UART peripheral init |

---

*Generated: June 2026 rev7 | Project: Auto_Control | MCU: STM32G474RE*
