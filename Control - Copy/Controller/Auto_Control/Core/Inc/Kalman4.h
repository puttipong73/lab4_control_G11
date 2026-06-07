/*
 * Kalman4.h
 *
 * 4-state motor Kalman filter — position measurement only.
 * (No current sensor: H is 1x4, R is 1x1)
 *
 * State vector:  x = [theta_l,  omega_l,  i,    tau_d]^T
 *                     position  velocity  current  disturbance torque
 *
 * Measurement:   z = [theta_meas]    (encoder position only)
 * Control input: u = [voltage]       (V)
 *
 * Discrete model (Euler first-order):
 *
 *   F = [[1,  Ts,              0,            0        ],
 *        [0,  1-(b/J)*Ts,  (N*Kt/J)*Ts, -(1/J)*Ts   ],
 *        [0, -(Ke*N/L)*Ts, 1-(R/L)*Ts,   0           ],
 *        [0,  0,              0,            1        ]]
 *
 *   G = [0, 0, Ts/L, 0]^T
 *
 *   H = [1, 0, 0, 0]    (1x4 — position only)
 *
 *   Q = G * sigma_v^2 * G^T  →  Q[2][2] = sigma_v^2 * (Ts/L)^2, rest = 0
 *
 *   R = [r_theta]        (1x1 scalar)
 *
 * When a current sensor is added:
 *   Change KALMAN4_MEAS_DIM to 2
 *   H  -> [[1,0,0,0],[0,0,1,0]]
 *   R  -> diag(r_theta, r_current)
 *   Restore mat2x2_inv and 2-measurement API
 */

#ifndef INC_KALMAN4_H_
#define INC_KALMAN4_H_

#include "main.h"
#include "arm_math.h"

#define KALMAN4_STATE_DIM    4
#define KALMAN4_MEAS_DIM     1   /* position only — update to 2 when current sensor added */
#define KALMAN4_CONTROL_DIM  1

typedef struct {
    /* State matrices */
    arm_matrix_instance_f32 x;       /* 4x1  state: [theta, omega, i, tau_d]^T */
    arm_matrix_instance_f32 P;       /* 4x4  error covariance */
    arm_matrix_instance_f32 F;       /* 4x4  state transition (physics) */
    arm_matrix_instance_f32 G;       /* 4x1  control input */
    arm_matrix_instance_f32 Q;       /* 4x4  process noise covariance */
    arm_matrix_instance_f32 H;       /* 1x4  observation matrix (position only) */
    arm_matrix_instance_f32 R;       /* 1x1  measurement noise covariance */

    /* Intermediate matrices */
    arm_matrix_instance_f32 K;       /* 4x1  Kalman gain */
    arm_matrix_instance_f32 S;       /* 1x1  innovation covariance */
    arm_matrix_instance_f32 S_inv;   /* 1x1  inverse of S (scalar) */
    arm_matrix_instance_f32 I;       /* 4x4  identity */

    /* Backing memory (row-major) */
    float x_data    [KALMAN4_STATE_DIM * 1];
    float P_data    [KALMAN4_STATE_DIM * KALMAN4_STATE_DIM];
    float F_data    [KALMAN4_STATE_DIM * KALMAN4_STATE_DIM];
    float G_data    [KALMAN4_STATE_DIM * KALMAN4_CONTROL_DIM];
    float Q_data    [KALMAN4_STATE_DIM * KALMAN4_STATE_DIM];
    float H_data    [KALMAN4_MEAS_DIM  * KALMAN4_STATE_DIM];  /* 1x4 = 4 floats */
    float R_data    [KALMAN4_MEAS_DIM  * KALMAN4_MEAS_DIM];   /* 1x1 = 1 float  */
    float K_data    [KALMAN4_STATE_DIM * KALMAN4_MEAS_DIM];   /* 4x1 = 4 floats */
    float S_data    [KALMAN4_MEAS_DIM  * KALMAN4_MEAS_DIM];   /* 1x1 = 1 float  */
    float S_inv_data[KALMAN4_MEAS_DIM  * KALMAN4_MEAS_DIM];   /* 1x1 = 1 float  */
    float I_data    [KALMAN4_STATE_DIM * KALMAN4_STATE_DIM];

    float Ts;
} KalmanFilter4_t;


/*
 * Kalman4_Init — build all matrices from motor parameters.
 *
 * Motor parameters:
 *   J_eq   kg*m^2      equivalent inertia
 *   b_eq   N*m*s/rad   equivalent viscous friction
 *   N      -           gear ratio (motor->load)
 *   Kt     N*m/A       torque constant
 *   Ke     V*s/rad     back-EMF constant
 *   L      H           armature inductance
 *   R_arm  Ohm         armature resistance
 *   Ts     s           sampling period (= LOOP_DT = 0.001 s)
 *
 * Noise parameters:
 *   sigma_v2  sigma_v^2         — voltage noise variance  Q = G*sigma_v2*G^T
 *   r_theta   sigma_theta^2     — position measurement noise variance (rad^2)
 *
 * Initial state:
 *   theta0, omega0, current0  (tau_d initialised to 0)
 */
void Kalman4_Init(KalmanFilter4_t *kf,
                  float theta0,  float omega0,  float current0,
                  float Ts,
                  float J_eq,   float b_eq,  float N,
                  float Kt,     float Ke,    float L,   float R_arm,
                  float sigma_v2,
                  float r_theta);

/*
 * Kalman4_Reset — reset state and P to initial values.
 * Keeps F, G, Q, H, R unchanged.
 * Call after reset_all, or call Kalman4_Init again if noise values changed.
 */
void Kalman4_Reset(KalmanFilter4_t *kf,
                   float theta0, float omega0, float current0);

/*
 * Kalman4_Update — one predict + correct cycle.
 *
 *   theta_meas  encoder position (rad)
 *   voltage     applied motor voltage (V) = pwm_duty/100 * V_supply
 *
 * Returns pointer to x_data[4]:
 *   [0] theta_l  — position           (rad)
 *   [1] omega_l  — velocity           (rad/s)
 *   [2] i        — current estimate   (A)    driven by physics model only
 *   [3] tau_d    — disturbance torque (N*m)
 */
float* Kalman4_Update(KalmanFilter4_t *kf,
                      float theta_meas,
                      float voltage);

#endif /* INC_KALMAN4_H_ */
