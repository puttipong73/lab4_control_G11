/*
 * Kalman.c
 *
 *  Created on: Apr 15, 2026
 *      Author: User
 */

#include "Kalman.h"

void Kalman_Init(KalmanFilter_t *kf, float initial_position, float dt, float processDisplacementVariance, float measurementNoise,float velocityState,float processVelocityVariance,int output) {
    // 1. Initialize structures to memory arrays
    arm_mat_init_f32(&kf->x, KALMAN_STATE_DIM, 1, kf->x_data);
    arm_mat_init_f32(&kf->P, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->P_data);
    arm_mat_init_f32(&kf->F, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->F_data);
    arm_mat_init_f32(&kf->G, KALMAN_STATE_DIM, KALMAN_CONTROL_DIM, kf->G_data);
    arm_mat_init_f32(&kf->Q, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->Q_data);
    arm_mat_init_f32(&kf->H, KALMAN_MEAS_DIM,  KALMAN_STATE_DIM, kf->H_data);
    arm_mat_init_f32(&kf->R, KALMAN_MEAS_DIM,  KALMAN_MEAS_DIM,  kf->R_data);
    arm_mat_init_f32(&kf->K, KALMAN_STATE_DIM, KALMAN_MEAS_DIM, kf->K_data);
    arm_mat_init_f32(&kf->S, KALMAN_MEAS_DIM,  KALMAN_MEAS_DIM, kf->S_data);
    arm_mat_init_f32(&kf->S_inv, KALMAN_MEAS_DIM, KALMAN_MEAS_DIM, kf->S_inv_data);
    arm_mat_init_f32(&kf->I, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->I_data);
    arm_mat_init_f32(&kf->QA, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->QA_data);

    // 2. Set Constant Velocity (CV) Initial Values
    kf->x_data[0] = 0.08f; // Position
    kf->x_data[1] = 0.0f;             // Velocity


    kf->F_data[0] = 1.0f; kf->F_data[1] = dt;
   kf->F_data[2] = 0.0f; kf->F_data[3] = 1.0f;
//
//    kf->F_data[0] = 1.0f; kf->F_data[1] = dt;
//   kf->F_data[2] = 0.0f; kf->F_data[3] = 1.0f;

    kf->H_data[0] = 1.0f; kf->H_data[1] = 0.0f; // Only measure position/

    kf->P_data[0] = 100.0f; kf->P_data[1] = 0.0f;
    kf->P_data[2] = 0.0f;   kf->P_data[3] = 100.0f;

    kf->QA_data[0] = 0.0f; kf->QA_data[1] = 0.0f;
    kf->QA_data[2] = 0.0f; kf->QA_data[3] = processVelocityVariance;
//        kf->QA_data[0] = 0.00001f;; kf->QA_data[1] = 0.0f;
//        kf->QA_data[2] = 0.0f; kf->QA_data[3] = 0.0f;


    float t1_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
	float t2_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
	arm_matrix_instance_f32 Temp1, Temp2,F_trans;
	arm_mat_init_f32(&Temp1, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t1_d);
	arm_mat_init_f32(&Temp2, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t2_d);


    float ft_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
//    Q=F*Qa*FT
	arm_mat_init_f32(&F_trans, KALMAN_STATE_DIM, KALMAN_STATE_DIM, ft_d);
	arm_mat_trans_f32(&kf->F, &F_trans);
	arm_mat_mult_f32(&kf->F, &kf->QA, &Temp1);
	arm_mat_mult_f32(&Temp1, &F_trans, &kf->Q);


    kf->G_data[0] = 0.0f; kf->G_data[1] = 0.0f;

//    kf->R_data[0] = 0.00292f; // Sensor noise variance
    kf->R_data[0] = measurementNoise;

    kf->I_data[0] = 1.0f; kf->I_data[1] = 0.0f;
    kf->I_data[2] = 0.0f; kf->I_data[3] = 1.0f;
}

float Kalman_Update(KalmanFilter_t *kf, float measurement, float control_input, float dt,int output) {

    // UPDATE PHYSICS MATRIX WITH EXACT TIME STEP
//    kf->F_data[1] = dt;

    // --- TEMPORARY MATRICES ---
    float t1_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float t2_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float t3_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float t4_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float ht_d[KALMAN_STATE_DIM * KALMAN_MEAS_DIM];

    arm_matrix_instance_f32 Temp1, Temp2, Temp3, Temp4, H_trans;
    arm_mat_init_f32(&Temp1, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t1_d);
    arm_mat_init_f32(&Temp2, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t2_d);
    arm_mat_init_f32(&Temp3, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t3_d);
    arm_mat_init_f32(&Temp4, KALMAN_STATE_DIM, KALMAN_STATE_DIM, t4_d);
    arm_mat_init_f32(&H_trans, KALMAN_STATE_DIM, KALMAN_MEAS_DIM, ht_d);

    arm_matrix_instance_f32 u, z;
    float u_data[1] = {control_input};
    float z_data[1] = {measurement};
    arm_mat_init_f32(&u, KALMAN_CONTROL_DIM, 1, u_data);
    arm_mat_init_f32(&z, KALMAN_MEAS_DIM, 1, z_data);

    // ==========================================
    // PREDICT STEP
    // ==========================================
    // x = F*x + G*u
    arm_mat_mult_f32(&kf->F, &kf->x, &Temp1);
    arm_mat_mult_f32(&kf->G, &u, &Temp2);
    arm_mat_add_f32(&Temp1, &Temp2, &kf->x);

    // P = F*P*F^T + Q
    arm_matrix_instance_f32 F_trans;
    float ft_d[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    arm_mat_init_f32(&F_trans, KALMAN_STATE_DIM, KALMAN_STATE_DIM, ft_d);

    arm_mat_trans_f32(&kf->F, &F_trans);
    arm_mat_mult_f32(&kf->F, &kf->P, &Temp1);
    arm_mat_mult_f32(&Temp1, &F_trans, &Temp2);
    arm_mat_add_f32(&Temp2, &kf->Q, &kf->P);

    // ==========================================
    // CORRECT STEP (JOSEPH FORM)
    // ==========================================
    // 1. Compute Innovation Covariance: S = H*P*H^T + R
        arm_mat_trans_f32(&kf->H, &H_trans);

        // H(1x2) * P(2x2) = Temp1(1x2)
        Temp1.numRows = KALMAN_MEAS_DIM;    // 1
        Temp1.numCols = KALMAN_STATE_DIM;   // 2
        arm_mat_mult_f32(&kf->H, &kf->P, &Temp1);

        // Temp1(1x2) * H_trans(2x1) = Temp2(1x1)
        Temp2.numRows = KALMAN_MEAS_DIM;    // 1
        Temp2.numCols = KALMAN_MEAS_DIM;    // 1
        arm_mat_mult_f32(&Temp1, &H_trans, &Temp2);

        // Temp2(1x1) + R(1x1) = S(1x1)
        arm_mat_add_f32(&Temp2, &kf->R, &kf->S);

        // --- 1x1 INVERSE FIX ---
        kf->S_inv_data[0] = 1.0f / kf->S_data[0];

        // 2. Compute Kalman Gain: K = P*H^T * S^-1
        // P(2x2) * H_trans(2x1) = Temp1(2x1)
        Temp1.numRows = KALMAN_STATE_DIM;   // 2
        Temp1.numCols = KALMAN_MEAS_DIM;    // 1
        arm_mat_mult_f32(&kf->P, &H_trans, &Temp1);

        // Temp1(2x1) * S_inv(1x1) = K(2x1)
        arm_mat_mult_f32(&Temp1, &kf->S_inv, &kf->K);

        // 3. Update State Estimate: x = x + K*(z - H*x)
        // H(1x2) * x(2x1) = Temp1(1x1)
        Temp1.numRows = KALMAN_MEAS_DIM;    // 1
        Temp1.numCols = 1;                  // 1
        arm_mat_mult_f32(&kf->H, &kf->x, &Temp1);

        // z(1x1) - Temp1(1x1) = Temp2(1x1)
        Temp2.numRows = KALMAN_MEAS_DIM;    // 1
        Temp2.numCols = 1;                  // 1
        arm_mat_sub_f32(&z, &Temp1, &Temp2);

        // K(2x1) * Temp2(1x1) = Temp1(2x1)
        Temp1.numRows = KALMAN_STATE_DIM;   // 2
        Temp1.numCols = 1;                  // 1
        arm_mat_mult_f32(&kf->K, &Temp2, &Temp1);

        // x(2x1) + Temp1(2x1) = x(2x1)
        arm_mat_add_f32(&kf->x, &Temp1, &kf->x);

        // 4. Update Uncertainty: P = (I - K*H)*P
        // K(2x1) * H(1x2) = Temp1(2x2)
        Temp1.numRows = KALMAN_STATE_DIM;   // 2
        Temp1.numCols = KALMAN_STATE_DIM;   // 2
        arm_mat_mult_f32(&kf->K, &kf->H, &Temp1);

        // I(2x2) - Temp1(2x2) = Temp2(2x2)
        Temp2.numRows = KALMAN_STATE_DIM;   // 2
        Temp2.numCols = KALMAN_STATE_DIM;   // 2
        arm_mat_sub_f32(&kf->I, &Temp1, &Temp2);

        // Temp2(2x2) * P(2x2) = Temp1(2x2)
        arm_mat_mult_f32(&Temp2, &kf->P, &Temp1);

        // Safely copy to P
        arm_copy_f32(Temp1.pData, kf->P.pData, KALMAN_STATE_DIM * KALMAN_STATE_DIM);


    // Return the cleaned position (index 0)
    return kf->x_data[output];
}
