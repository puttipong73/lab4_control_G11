/*
 * Kalman.h
 *
 *  Created on: Apr 15, 2026
 *      Author: User
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_
#include "main.h"
#include "arm_math.h"


// --- CONFIGURATION ---
#define KALMAN_STATE_DIM   2  // Tracking Position [0] AND Velocity [1]
#define KALMAN_MEAS_DIM    1  // Only measuring Distance
#define KALMAN_CONTROL_DIM 1  // No control input, but kept for structural safety

// --- KALMAN FILTER STRUCTURE ---
typedef struct {
    arm_matrix_instance_f32 x; // State vector estimate (Position & Velocity)
    arm_matrix_instance_f32 P; // Estimate error covariance
    arm_matrix_instance_f32 F; // State transition matrix (Physics)
    arm_matrix_instance_f32 G;
    arm_matrix_instance_f32 QA;// Control input matrix
    arm_matrix_instance_f32 Q; // Process noise covariance
    arm_matrix_instance_f32 H; // Observation/Measurement matrix
    arm_matrix_instance_f32 R; // Measurement noise covariance

    // Intermediate calculation matrices
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 S;
    arm_matrix_instance_f32 S_inv;
    arm_matrix_instance_f32 I;

    // Backing memory arrays
    float x_data[KALMAN_STATE_DIM * 1];
    float P_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float F_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float G_data[KALMAN_STATE_DIM * KALMAN_CONTROL_DIM];
    float Q_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float QA_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float H_data[KALMAN_MEAS_DIM  * KALMAN_STATE_DIM];
    float R_data[KALMAN_MEAS_DIM  * KALMAN_MEAS_DIM];
    float K_data[KALMAN_STATE_DIM * KALMAN_MEAS_DIM];
    float S_data[KALMAN_MEAS_DIM  * KALMAN_MEAS_DIM];
    float S_inv_data[KALMAN_MEAS_DIM * KALMAN_MEAS_DIM];
    float I_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];

} KalmanFilter_t;

// --- PROTOTYPES ---
void Kalman_Init(KalmanFilter_t *kf, float initial_position, float dt, float processDisplacementNoise, float measurementNoise,float velocityState,float processVelocitytNoise,int output);
float Kalman_Update(KalmanFilter_t *kf, float measurement, float control_input, float dt, int output);



#endif /* INC_KALMAN_H_ */
