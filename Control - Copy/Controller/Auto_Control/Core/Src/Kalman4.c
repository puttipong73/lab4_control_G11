/*
 * Kalman4.c
 *
 * 4-state physics-based Kalman filter — position measurement only.
 *
 * State:       x = [theta_l, omega_l, i, tau_d]^T
 * Measurement: z = [theta_meas]   (1x1 scalar — no current sensor)
 * Input:       u = [voltage]      (V)
 *
 * S is 1x1 → inverse is a scalar reciprocal (no mat2x2_inv needed).
 */

#include "Kalman4.h"
#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Kalman4_Init
 * --------------------------------------------------------------------------- */
void Kalman4_Init(KalmanFilter4_t *kf,
                  float theta0,  float omega0,  float current0,
                  float Ts,
                  float J_eq,   float b_eq,  float N,
                  float Kt,     float Ke,    float L,   float R_arm,
                  float sigma_v2,
                  float r_theta)
{
    kf->Ts = Ts;

    /* Bind instances to backing arrays */
    arm_mat_init_f32(&kf->x,     4, 1, kf->x_data);
    arm_mat_init_f32(&kf->P,     4, 4, kf->P_data);
    arm_mat_init_f32(&kf->F,     4, 4, kf->F_data);
    arm_mat_init_f32(&kf->G,     4, 1, kf->G_data);
    arm_mat_init_f32(&kf->Q,     4, 4, kf->Q_data);
    arm_mat_init_f32(&kf->H,     1, 4, kf->H_data);   /* 1x4 — position only */
    arm_mat_init_f32(&kf->R,     1, 1, kf->R_data);   /* 1x1 scalar */
    arm_mat_init_f32(&kf->K,     4, 1, kf->K_data);   /* 4x1 */
    arm_mat_init_f32(&kf->S,     1, 1, kf->S_data);   /* 1x1 scalar */
    arm_mat_init_f32(&kf->S_inv, 1, 1, kf->S_inv_data);
    arm_mat_init_f32(&kf->I,     4, 4, kf->I_data);

    /* --- Initial state --- */
    kf->x_data[0] = theta0;
    kf->x_data[1] = omega0;
    kf->x_data[2] = current0;
    kf->x_data[3] = 0.0f;

    /* -------------------------------------------------------------------
     * F — discrete state transition (Euler first-order, row-major)
     *
     * Row 0: [ 1,             Ts,             0,          0        ]
     * Row 1: [ 0,  1-(b/J)*Ts,  (N*Kt/J)*Ts, -(1/J)*Ts  ]
     * Row 2: [ 0, -(Ke*N/L)*Ts, 1-(R/L)*Ts,   0          ]
     * Row 3: [ 0,             0,             0,          1        ]
     * ------------------------------------------------------------------- */
    kf->F_data[ 0] = 1.0f;
    kf->F_data[ 1] = Ts;
    kf->F_data[ 2] = 0.0f;
    kf->F_data[ 3] = 0.0f;

    kf->F_data[ 4] = 0.0f;
    kf->F_data[ 5] = 1.0f - (b_eq / J_eq) * Ts;
    kf->F_data[ 6] = (N * Kt / J_eq) * Ts;
    kf->F_data[ 7] = -(1.0f / J_eq) * Ts;

    kf->F_data[ 8] = 0.0f;
    kf->F_data[ 9] = -(Ke * N / L) * Ts;
    kf->F_data[10] = expf(-(R_arm / L) * Ts);  /* ZOH: stable when tau_e = L/R < Ts */
    kf->F_data[11] = 0.0f;

    kf->F_data[12] = 0.0f;
    kf->F_data[13] = 0.0f;
    kf->F_data[14] = 0.0f;
    kf->F_data[15] = 1.0f;

    /* --- G = [0, 0, Ts/L, 0]^T --- */
    kf->G_data[0] = 0.0f;
    kf->G_data[1] = 0.0f;
    kf->G_data[2] = Ts / L;
    kf->G_data[3] = 0.0f;

    /* -------------------------------------------------------------------
     * H — 1x4, measures position (state 0) only
     * [1, 0, 0, 0]
     * ------------------------------------------------------------------- */
    kf->H_data[0] = 1.0f;
    kf->H_data[1] = 0.0f;
    kf->H_data[2] = 0.0f;
    kf->H_data[3] = 0.0f;

    /* -------------------------------------------------------------------
     * Q = G * sigma_v2 * G^T  (4x4)
     *
     * Noise enters only through the voltage input channel.
     * G = [0, 0, Ts/L, 0]^T  (already built above)
     * sigma_v2 = sigma_v^2  — voltage noise variance
     *
     * Result: Q[2][2] = sigma_v^2 * (Ts/L)^2, all other entries = 0.
     * ------------------------------------------------------------------- */
    {
        float gt_data[4];
        float t41_data[4];
        float qa_data[1] = {sigma_v2};

        arm_matrix_instance_f32 GT, T41, Qa_mat;
        arm_mat_init_f32(&GT,     1, 4, gt_data);
        arm_mat_init_f32(&T41,    4, 1, t41_data);
        arm_mat_init_f32(&Qa_mat, 1, 1, qa_data);

        arm_mat_mult_f32(&kf->G, &Qa_mat, &T41);    /* T41 = G * sigma_v2  (4x1) */
        arm_mat_trans_f32(&kf->G, &GT);              /* GT  = G^T           (1x4) */
        arm_mat_mult_f32(&T41,   &GT,     &kf->Q);  /* Q   = G*sigma_v2*G^T (4x4) */
    }

    /* --- R = [r_theta] — scalar measurement noise --- */
    kf->R_data[0] = r_theta;

    /* --- P — initial covariance (large = high uncertainty at startup) --- */
    memset(kf->P_data, 0, sizeof(kf->P_data));
    kf->P_data[ 0] = 100.0f;   /* theta  */
    kf->P_data[ 5] = 100.0f;   /* omega  */
    kf->P_data[10] = 100.0f;   /* i      */
    kf->P_data[15] = 100.0f;   /* tau_d  */

    /* --- I — 4x4 identity --- */
    memset(kf->I_data, 0, sizeof(kf->I_data));
    kf->I_data[ 0] = 1.0f;
    kf->I_data[ 5] = 1.0f;
    kf->I_data[10] = 1.0f;
    kf->I_data[15] = 1.0f;
}

/* ---------------------------------------------------------------------------
 * Kalman4_Reset — reset state and P; keeps F, G, Q, H, R unchanged.
 * --------------------------------------------------------------------------- */
void Kalman4_Reset(KalmanFilter4_t *kf,
                   float theta0, float omega0, float current0)
{
    kf->x_data[0] = theta0;
    kf->x_data[1] = omega0;
    kf->x_data[2] = current0;
    kf->x_data[3] = 0.0f;

    memset(kf->P_data, 0, sizeof(kf->P_data));
    kf->P_data[ 0] = 100.0f;
    kf->P_data[ 5] = 100.0f;
    kf->P_data[10] = 100.0f;
    kf->P_data[15] = 100.0f;
}

/* ---------------------------------------------------------------------------
 * Kalman4_Update — one predict + correct cycle.
 *
 * Dimensions (MEAS_DIM = 1):
 *   H   : 1x4    H^T : 4x1
 *   S   : 1x1    S^-1: scalar
 *   K   : 4x1
 *   z   : 1x1
 *   innov: 1x1
 *
 * PREDICT:
 *   x = F*x + G*u          (4x4)(4x1) + (4x1)(1x1)
 *   P = F*P*F^T + Q        (4x4)(4x4)(4x4) + (4x4)
 *
 * CORRECT:
 *   S     = H*P*H^T + R    (1x4)(4x4)(4x1) + (1x1) = 1x1
 *   S^-1  = 1 / S[0]
 *   K     = P*H^T * S^-1   (4x4)(4x1)(1x1) = 4x1
 *   x     = x + K*(z - H*x)
 *   P     = (I - K*H)*P
 * --------------------------------------------------------------------------- */
float* Kalman4_Update(KalmanFilter4_t *kf,
                      float theta_meas,
                      float voltage)
{
    /* --- Temporary backing arrays --- */
    float ft[16];       /* F^T       4x4 */
    float ht[4];        /* H^T       4x1 */
    float t44a[16];     /* general   4x4 */
    float t44b[16];     /* general   4x4 */
    float t41a[4];      /* general   4x1 */
    float t41b[4];      /* general   4x1 */
    float t14a[4];      /* H*P       1x4 */
    float t11a[1];      /* general   1x1 */
    float t11b[1];      /* innovation 1x1 */
    float u_d[1] = {voltage};
    float z_d[1] = {theta_meas};

    arm_matrix_instance_f32 F_T, H_T;
    arm_matrix_instance_f32 T44a, T44b;
    arm_matrix_instance_f32 T41a, T41b;
    arm_matrix_instance_f32 T14a;
    arm_matrix_instance_f32 T11a, T11b;
    arm_matrix_instance_f32 u_mat, z_mat;

    arm_mat_init_f32(&F_T,   4, 4, ft);
    arm_mat_init_f32(&H_T,   4, 1, ht);     /* H^T is 4x1 */
    arm_mat_init_f32(&T44a,  4, 4, t44a);
    arm_mat_init_f32(&T44b,  4, 4, t44b);
    arm_mat_init_f32(&T41a,  4, 1, t41a);
    arm_mat_init_f32(&T41b,  4, 1, t41b);
    arm_mat_init_f32(&T14a,  1, 4, t14a);   /* H*P result is 1x4 */
    arm_mat_init_f32(&T11a,  1, 1, t11a);
    arm_mat_init_f32(&T11b,  1, 1, t11b);
    arm_mat_init_f32(&u_mat, 1, 1, u_d);
    arm_mat_init_f32(&z_mat, 1, 1, z_d);

    /* ====================================================================
     * PREDICT
     * ==================================================================== */

    /* x = F*x + G*u */
    arm_mat_mult_f32(&kf->F, &kf->x, &T41a);         /* T41a = F*x  (4x1) */
    arm_mat_mult_f32(&kf->G, &u_mat, &T41b);          /* T41b = G*u  (4x1) */
    arm_mat_add_f32 (&T41a,  &T41b,  &kf->x);         /* x    = F*x + G*u  */

    /* P = F*P*F^T + Q */
    arm_mat_trans_f32(&kf->F, &F_T);                  /* F_T  = F^T  (4x4) */
    arm_mat_mult_f32(&kf->F, &kf->P, &T44a);          /* T44a = F*P  (4x4) */
    arm_mat_mult_f32(&T44a,  &F_T,   &T44b);          /* T44b = F*P*F^T    */
    arm_mat_add_f32 (&T44b,  &kf->Q, &kf->P);         /* P    = F*P*F^T + Q */

    /* ====================================================================
     * CORRECT
     * ==================================================================== */

    arm_mat_trans_f32(&kf->H, &H_T);                  /* H_T  = H^T  (4x1) */

    /* S = H*P*H^T + R  (1x1) */
    arm_mat_mult_f32(&kf->H, &kf->P, &T14a);          /* T14a = H*P  (1x4) */
    arm_mat_mult_f32(&T14a,  &H_T,   &T11a);          /* T11a = H*P*H^T (1x1) */
    arm_mat_add_f32 (&T11a,  &kf->R, &kf->S);         /* S    = H*P*H^T + R */

    /* S^-1 — scalar reciprocal */
    kf->S_inv_data[0] = 1.0f / kf->S_data[0];

    /* K = P*H^T * S^-1  (4x1) */
    arm_mat_mult_f32(&kf->P,   &H_T,      &T41a);     /* T41a = P*H^T  (4x1) */
    arm_mat_mult_f32(&T41a,    &kf->S_inv, &kf->K);   /* K    = P*H^T * S^-1 */

    /* x = x + K*(z - H*x) */
    arm_mat_mult_f32(&kf->H, &kf->x, &T11a);          /* T11a = H*x   (1x1) */
    arm_mat_sub_f32 (&z_mat, &T11a,   &T11b);          /* T11b = z - H*x (innovation) */
    arm_mat_mult_f32(&kf->K, &T11b,   &T41a);          /* T41a = K*innov (4x1) */
    arm_mat_add_f32 (&kf->x, &T41a,   &kf->x);         /* x    = x + K*innov */

    /* P = (I - K*H)*P */
    arm_mat_mult_f32(&kf->K,   &kf->H,  &T44a);       /* T44a = K*H   (4x4) */
    arm_mat_sub_f32 (&kf->I,   &T44a,   &T44b);       /* T44b = I - K*H     */
    arm_mat_mult_f32(&T44b,    &kf->P,  &T44a);       /* T44a = (I-K*H)*P   */
    arm_copy_f32(T44a.pData, kf->P.pData, 16);         /* P    = (I-K*H)*P   */

    return kf->x_data;
}
