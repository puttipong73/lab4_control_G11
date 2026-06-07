# PID Gain Design — Auto_Control Motor Controller

**Specifications:** Settling time ≤ 0.5 s · Overshoot < 1 %  
**Motor supply assumed:** V_supply = 24 V (scale gains if different — see §5)

---

## 1. Plant Model

### Transfer Function: %PWM → ω_load (rad/s)

$$G(s) = \frac{N \cdot K_t}{(L J_{eq})s^2 + (L b_{eq} + R J_{eq})s + (R b_{eq} + N^2 K_t K_e)}$$

### Computed Coefficients

| Term | Formula | Value |
|------|---------|-------|
| Numerator | N · Kt | 50 × 0.00747 = **0.3735** |
| a₂ | L · J_eq | 0.0012794 × 0.027 = **3.454 × 10⁻⁵** |
| a₁ | L · b_eq + R · J_eq | 6.4 × 10⁻⁴ + 0.0756 = **0.07624** |
| a₀ | R · b_eq + N² Kt Ke | 1.4 + 0.155 = **1.555** |

### Plant Poles

```
p₁ = −20.6 rad/s    dominant (mechanical)
p₂ = −2186 rad/s    fast electrical — neglected for outer loop design
```

### Reduced First-Order Model (neglecting p₂)

```
G_pwm(s) ≈ 1.188 / (s + 20.6)

τ_mech  = 1 / 20.6  = 0.0485 s
K_DC    = 1.188 / 20.6 = 0.0577  (rad/s) / (%PWM)   @ 24 V
```

---

## 2. Inner Velocity Loop — TIM6 @ 1 kHz

**Target bandwidth:** ωc = 50 rad/s (must be >> outer loop ωn)

### Method: Zero-Pole Cancellation

Place PI zero at the dominant plant pole p₁:

```
ki_vel / kp_vel  =  |p₁|  =  20.6 rad/s
```

After cancellation the open loop becomes a pure integrator:

```
L_vel(s) = kp_vel × 1.188 / s
```

At crossover ωc = 50 rad/s → |L(jωc)| = 1:

```
kp_vel = ωc / 1.188 = 50 / 1.188 = 42.1  →  42  %/(rad/s)
ki_vel = kp_vel × 20.6 = 42.1 × 20.6 = 867  →  865  %/rad
kd_vel = 0   (too noisy at 1 kHz)
```

### Closed-Loop Velocity (approximation)

```
CL_vel(s) ≈ 50 / (s + 50)     bandwidth = 50 rad/s
```

### Anti-Windup Check

```
max_integral = max_pwm / ki_vel = 100 / 865 = 0.116 rad ≈ 6.6°
```

---

## 3. Outer Position Loop — TIM7 @ 500 Hz

### Plant Seen by Outer Loop

```
G_pos(s) = CL_vel(s) / s  ≈  50 / (s(s + 50))
```

### Closed-Loop with P Controller

```
CL_pos(s) = 50 · kp_pos / (s² + 50s + 50 · kp_pos)
```

Poles:  s₁,₂ = (−50 ± √(2500 − 200 · kp_pos)) / 2

### Pole–Performance Table

| kp_position | Poles | ts (2%) | Overshoot |
|-------------|-------|---------|-----------|
| 5.0 | −5.6, −44.4 | 0.71 s | 0 % |
| **8.0** | **−10.0, −40.0** | **0.40 s** | **0 %** ✓ |
| 10.4 | complex (ζ = 0.85) | 0.27 s | 0.63 % ✓ |
| 12.5 | complex (ζ = 0.76) | 0.27 s | ~2.4 % ✗ |

**Selected:** kp_position = 8.0 → overdamped, poles at −10 and −40

```
Settling time  =  4 / 10  =  0.40 s  ≤ 0.5 s  ✓
Overshoot      =  0 %  (overdamped, two real poles)  ✓
```

### ki_pos — Steady-State Error Correction

Add a small integral after velocity loop is stable:

```
ki_pos = 2.0  rad/s / (rad·s)
```

Low enough that the dominant poles at −10 and −40 are not significantly moved.

---

## 4. Final Gain Summary

```
════════════════════════════════════════
  VELOCITY PID   (TIM6 inner, 1 kHz)
════════════════════════════════════════
  kp_vel              =   42.0   %/(rad/s)
  ki_vel              =  865.0   %/rad
  kd_vel              =    0.0
  min_pwm_threshold   =    5.0   %     (raise if static friction)
  use_kf4_vel         =    0     0=raw encoder · 1=Kalman4 velocity

════════════════════════════════════════
  POSITION PID  (TIM7 outer, 500 Hz)
════════════════════════════════════════
  kp_position         =    8.0   rad/s/rad
  ki_pos              =    2.0   rad/s/(rad·s)
  kd_pos              =    0.0   (enable only if oscillation persists)
  pos_deadband_deg    =    0.5   deg   (raise if backlash > 0.5°)
```

**Expected closed-loop response:**
- Settling time: ~0.40 s
- Overshoot: 0 % (overdamped)

---

## 5. Scaling for Different V_supply

All gains were derived assuming V_supply = 24 V. If your actual supply differs:

```
kp_vel_actual = kp_vel × (24 / V_supply_actual)
ki_vel_actual = ki_vel × (24 / V_supply_actual)
```

Position gains (kp_position, ki_pos) are not affected by supply voltage.

---

## 6. Tuning Procedure on Real Hardware

Apply gains in this order. Use Live Expressions to change values without reflashing.

**Step 1 — Verify encoder direction**
```
control_mode = 3, pid_enabled = 1, traj_target_deg = 30
```
`g_position_rad` should increase positively. If reversed → `motor_dir_inverted = 1`.

**Step 2 — Apply motor parameters**

Verify `motor_*` variables match your motor datasheet, then:
```
apply_motor_params = 1
```

**Step 3 — Start with conservative velocity gains**
```
kp_vel = 20,  ki_vel = 400,  kd_vel = 0
```
Observe `g_kf4_velocity` vs `g_smooth_vel` on telemetry bytes 46–49 vs 6–9.
Increase `kp_vel` until tracking, back off if oscillation.

**Step 4 — Move to calculated velocity gains**
```
kp_vel = 42,  ki_vel = 865
```
Check `i_term_live` (telemetry bytes 66–69) does not saturate permanently.

**Step 4b — Switch to Kalman4 velocity (optional)**
Once Kalman4 noise is tuned (kf4_sigma_v2, kf4_r_theta):
```
use_kf4_vel = 1
```
`g_kf4_velocity` (bytes 46–49) replaces `g_velocity_rad_s` in both
the velocity PID error and the position PID kd term.
If PID becomes unstable after switching → reduce kf4_sigma_v2 for a smoother estimate,
or revert with `use_kf4_vel = 0`.

**Step 5 — Tune position loop**
```
kp_position = 8.0,  ki_pos = 0.0
```
Command `traj_target_deg = 90`. Check `ss_error_deg` converges within 0.5 s.
Add `ki_pos = 2.0` after confirming no oscillation.

**Step 6 — Wind-up test (optional)**
Block the motor shaft during a 90° move. Release. Confirm:
- `i_term_live` stays below `max_pwm / ki_vel` = 0.116 rad
- No large overshoot after release

**Step 7 — Enable feedforward**
```
refff_enabled = 1   (verify V_supply matches actual bus voltage)
distff_enabled = 1  (observe g_distff_pwm before enabling)
```

---

## 7. Troubleshooting

| Symptom | Likely cause | Action |
|---------|-------------|--------|
| Oscillation during ramp | kp_vel too high | Reduce kp_vel by 20 % |
| Slow velocity tracking | kp_vel too low | Increase kp_vel toward 42 |
| Position never settles | ki_pos absent or kp_pos too low | Increase kp_pos by 1.0 steps |
| Position overshoots | kp_pos too high (near 10+) | Reduce kp_pos toward 8 |
| `i_term_live` always at limit | ki_vel too high | Reduce ki_vel; check V_supply |
| Motor stalls at small error | Static friction > min_pwm_threshold | Raise min_pwm_threshold to 8–12 % |

---

## 8. Integrator Wind-Up Test

To verify anti-windup is working:

1. Set `traj_target_deg = 90`, `control_mode = 3`, `pid_enabled = 1`
2. Block the motor shaft physically
3. Observe in Live Expressions:
   - `i_term_live` ≤ 0.116 rad (clamped)
   - `g_pwm_duty` = 100 % (saturated)
4. Release shaft — motor should move smoothly to target without large overshoot

---

*Derived from motor model: J_eq=0.027 kg·m², b_eq=0.5 N·m·s/rad, N=50, Kt=0.00747 N·m/A, Ke=0.0083 V·s/rad, L=0.001279 H, R=2.8 Ω — recalculate if parameters change.*

*Auto_Control — June 2026*
