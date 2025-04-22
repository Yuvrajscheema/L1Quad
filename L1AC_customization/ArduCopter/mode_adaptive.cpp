// mode_adaptive.cpp
// This file contains the main code for running ACRL's adaptive flight mode.
// Copyright 2021 Sheng Cheng, all rights reserved.

// Dr. Sheng Cheng, Nov. 2021
// Email: chengs@illinois.edu
// Advanced Controls Research Laboratory
// Department of Mechanical Science and Engineering
// University of Illinois Urbana-Champaign
// Urbana, IL 61821, USA

#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "ACRL_trajectories.h"         // small libraries for trajectories at ACRL

#if MODE_ADAPTIVE_ENABLED == ENABLED

/*
 * Init and run calls for adaptive flight mode (copied from stabilize)
 */

// Init function: this function will be called everytime the FC enters the adaptive mode.
bool ModeAdaptive::init(bool ignore_checks)
{
    motorEnable = 1; // whether to raise motor PWM
    landingComplete = 0; 

    if (ahrs.have_inertial_nav())
    {
        if(ahrs.get_velocity_NED(v_hat_prev)){;} // state predictor value of translational speed
        v_prev = v_hat_prev;               // initialize the previous velocity in the same way
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Inertial navigation is inactive upon entering adaptive mode.");
        motorEnable = 0; // if the velocity is unavailable, disable the flight.
    }
    omega_hat_prev = AP::ahrs().get_gyro(); // state predictor value of rotational speed
    omega_prev = omega_hat_prev;



    // initialize rotation matrix
    Quaternion q;
    q.rotation_matrix(R_prev); // transforming the quaternion q to rotation matrix R

    u_b_prev = u_b_prev * 0;   // initialize u_baseline
    u_ad_prev = u_ad_prev * 0; // initialize u_ad
    lpf1_prev = lpf1_prev * 0; // initialize lpf1_prev
    lpf2_prev = lpf2_prev * 0; // initialize lpf2_prev

    landingTriggered = 0; // set the indicator to 0  
    if (motorEnable == 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "Adaptive mode initialization is done with motors enabled");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Adpative mode initialization is done.");
    }
    return true;
}

void ModeAdaptive::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();
    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }
    VectorN<float, 4> thrustMomentCmd;
    
    thrustMomentCmd = generateThrustMomentCMD(pilot_desired_throttle, target_pitch, target_roll, target_yaw_rate);
    VectorN<float, 4> l1ThrustMomentCmd;
    l1ThrustMomentCmd = L1AdaptiveAugmentation(thrustMomentCmd);

    VectorN<float, 4> motorPWM;

    motorPWM = motorMixing(thrustMomentCmd + l1ThrustMomentCmd);

    if (motorPWM[0] < 0) {motorPWM[0] = 0;}
    else if (motorPWM[0] > 100) {motorPWM[0] = 100;}
    if (motorPWM[1] < 0) {motorPWM[1] = 0;}
    else if (motorPWM[1] > 100) {motorPWM[1] = 100;}
    if (motorPWM[2] < 0) {motorPWM[2] = 0;}
    else if (motorPWM[2] > 100) {motorPWM[2] = 100;}
    if (motorPWM[3] < 0) {motorPWM[3] = 0;}
    else if (motorPWM[3] > 100) {motorPWM[3] = 100;}

    if (motors->armed()) {
        motors->rc_write(0, 1000 + motorEnable * 10 * motorPWM[0]);
        motors->rc_write(1, 1000 + motorEnable * 10 * motorPWM[1]);
        motors->rc_write(2, 1000 + motorEnable * 10 * motorPWM[2]);
        motors->rc_write(3, 1000 + motorEnable * 10 * motorPWM[3]);
    } else {
        motors->rc_write(0,0);
        motors->rc_write(1,0);
        motors->rc_write(2,0);
        motors->rc_write(3,0);
    }

}

VectorN<float, 4> ModeAdaptive::generateThrustMomentCMD(float target_thrust,
                                                        float target_pitch,
                                                        float target_roll,
                                                        float target_yaw_rate) {

    // const float dt = 0.0025; // sampling time (update rate at 400 Hz)
    float current_pitch = ahrs.get_pitch();
    float current_roll = ahrs.get_roll();
    float current_yaw_rate = ahrs.get_gyro_latest()[2];
    

    float pitch_error = target_pitch - current_pitch;
    float roll_error = target_roll - current_roll;
    float yaw_rate_error = target_yaw_rate - current_yaw_rate;

    float thrust = g.custom_KThrust * attitude_control->get_throttle_boosted(target_thrust) * 10;
    float M_x = g.custom_KPr * roll_error / 1000;
    float M_y = g.custom_KPp * pitch_error / 1000;
    float M_z = g.custom_KPy * yaw_rate_error / 100000;

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = thrust;
    thrustMomentCmd[1] = M_x;
    thrustMomentCmd[2] = M_y;
    thrustMomentCmd[3] = M_z;
    

    return thrustMomentCmd;

}

VectorN<float, 4> ModeAdaptive::L1AdaptiveAugmentation(VectorN<float, 4> thrustMomentCmd)
{
    // state predictor
    Vector3f v_hat;          // state predictor value of translational speed
    Vector3f omega_hat;      // state predictor value of rotational speed
    Vector3f e3 = {0, 0, 1}; // unit vector

    const float dt = 0.0025; // sampling time (update rate at 400 Hz)

    int8_t l1enable = g.l1enable;

    // load translational velocity
    Vector3f v_now;
    if (ahrs.have_inertial_nav())
    {
        if(ahrs.get_velocity_NED(v_now)){;} // state predictor value of translational speed
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive in state predictor");
    }
    // coefficient for As
    float As_v = g.Asv;         //
    float As_omega = g.Asomega; //

    // load rotational velocity
    Vector3f omega_now = AP::ahrs().get_gyro();

    float massInverse = 1.0f / kg_vehicleMass;

    // compute prediction error (on previous step)
    Vector3f vpred_error_prev = v_hat_prev - v_prev;
    Vector3f omegapred_error_prev = omega_hat_prev - omega_prev;

    // NOTE: per the definition in vector3.h, vector * scalaer must have vector first then multiply the scalar later
    v_hat = v_hat_prev + (e3 * GRAVITY_MAGNITUDE - R_prev.colz() * (u_b_prev[0] + u_ad_prev[0] + sigma_m_hat_prev[0]) * massInverse + R_prev.colx() * sigma_um_hat_prev[0] * massInverse + R_prev.coly() * sigma_um_hat_prev[1] * massInverse + vpred_error_prev * As_v) * dt;

    // Jin is the inverse of inertia J, which has been done in mode.h

    // temp vector: thrustMomentCmd[1--3] + u_ad_prev[1--3] + sigma_m_hat_prev[1--3]
    Vector3f tempVec = {u_b_prev[1] + u_ad_prev[1] + sigma_m_hat_prev[1], u_b_prev[2] + u_ad_prev[2] + sigma_m_hat_prev[2], u_b_prev[3] + u_ad_prev[3] + sigma_m_hat_prev[3]};
    omega_hat = omega_hat_prev + (-Jinv * (omega_prev % (J * omega_prev)) + Jinv * tempVec + omegapred_error_prev * As_omega) * dt;

    // update the state prediction storage
    v_hat_prev = v_hat;
    omega_hat_prev = omega_hat;

    // compute prediction error (for this step)
    Vector3f vpred_error = v_hat - v_now;
    Vector3f omegapred_error = omega_hat - omega_now;

    // exponential coefficients for As
    float exp_As_v_dt = expf(As_v * dt);
    float exp_As_omega_dt = expf(As_omega * dt);

    // later part of uncertainty estimation (piecewise constant)
    Vector3f PhiInvmu_v = vpred_error / (exp_As_v_dt - 1) * As_v * exp_As_v_dt;
    Vector3f PhiInvmu_omega = omegapred_error / (exp_As_omega_dt - 1) * As_omega * exp_As_omega_dt;

    VectorN<float, 4> sigma_m_hat; // estimated matched uncertainty
    Vector3f sigma_m_hat_2to4;     // second to fourth element of the estimated matched uncertainty
    Vector2f sigma_um_hat;         // estimated unmatched uncertainty

    // use the rotation matrix in the current step
    Quaternion q;
    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    sigma_m_hat[0] = R.colz() * PhiInvmu_v * kg_vehicleMass;
    sigma_m_hat_2to4 = -J * PhiInvmu_omega;
    sigma_m_hat[1] = sigma_m_hat_2to4[0];
    sigma_m_hat[2] = sigma_m_hat_2to4[1];
    sigma_m_hat[3] = sigma_m_hat_2to4[2];

    sigma_um_hat[0] = -R.colx() * PhiInvmu_v * kg_vehicleMass;
    sigma_um_hat[1] = -R.coly() * PhiInvmu_v * kg_vehicleMass;

    // store uncertainty estimations
    sigma_m_hat_prev = sigma_m_hat;
    sigma_um_hat_prev = sigma_um_hat;

    // compute lpf1 coefficients
    float lpf1_coefficientThrust1 = expf(-g.ctoffq1Thrust * 0.0025);
    float lpf1_coefficientThrust2 = 1.0 - lpf1_coefficientThrust1;

    float lpf1_coefficientMoment1 = expf(-g.ctoffq1Moment * 0.0025);
    float lpf1_coefficientMoment2 = 1.0 - lpf1_coefficientMoment1;

    // update the adaptive control
    VectorN<float, 4> u_ad_int;
    VectorN<float, 4> u_ad;

    // low-pass filter 1 (negation is added to u_ad_prev to filter the correct signal)
    u_ad_int[0] = lpf1_coefficientThrust1 * (lpf1_prev[0]) + lpf1_coefficientThrust2 * sigma_m_hat[0];
    u_ad_int[1] = lpf1_coefficientMoment1 * (lpf1_prev[1]) + lpf1_coefficientMoment2 * sigma_m_hat[1];
    u_ad_int[2] = lpf1_coefficientMoment1 * (lpf1_prev[2]) + lpf1_coefficientMoment2 * sigma_m_hat[2];
    u_ad_int[3] = lpf1_coefficientMoment1 * (lpf1_prev[3]) + lpf1_coefficientMoment2 * sigma_m_hat[3];

    lpf1_prev = u_ad_int; // store the current state

    float lpf2_coefficientMoment1 = expf(-g.ctoffq2Moment * 0.0025);
    float lpf2_coefficientMoment2 = 1.0 - lpf2_coefficientMoment1;

    // low-pass filter 2 (optional)
    u_ad[0] = u_ad_int[0]; // only one filter on the thrust channel
    u_ad[1] = lpf2_coefficientMoment1 * lpf2_prev[1] + lpf2_coefficientMoment2 * u_ad_int[1];
    u_ad[2] = lpf2_coefficientMoment1 * lpf2_prev[2] + lpf2_coefficientMoment2 * u_ad_int[2];
    u_ad[3] = lpf2_coefficientMoment1 * lpf2_prev[3] + lpf2_coefficientMoment2 * u_ad_int[3];

    lpf2_prev = u_ad; // store the current state
    // negate
    u_ad = -u_ad;

    AP::logger().Write("L1AD", "v1,v2,v3,v1hat,v2hat,v3hat,o1,o2,o3,o1hat,o2hat,o3hat", "ffffffffffff",
                       (double)v_now.x,
                       (double)v_now.y,
                       (double)v_now.z,
                       (double)v_hat_prev.x,
                       (double)v_hat_prev.y,
                       (double)v_hat_prev.z,
                       (double)omega_now.x,
                       (double)omega_now.y,
                       (double)omega_now.z,
                       (double)omega_hat_prev.x,
                       (double)omega_hat_prev.y,
                       (double)omega_hat_prev.z);

    AP::logger().Write("L1AE", "sm1,sm2,sm3,sm4,sum1,sum2,uad1,uad2,uad3,uad4,l1enable", "ffffffffffb",
                       (double)sigma_m_hat_prev[0],
                       (double)sigma_m_hat_prev[1],
                       (double)sigma_m_hat_prev[2],
                       (double)sigma_m_hat_prev[3],
                       (double)sigma_um_hat_prev[0],
                       (double)sigma_um_hat_prev[1],
                       (double)u_ad[0],
                       (double)u_ad[1],
                       (double)u_ad[2],
                       (double)u_ad[3],
                       l1enable);
    // store the values for next iteration
    u_ad_prev = u_ad * l1enable;

    v_prev = v_now;
    omega_prev = omega_now;
    R_prev = R;
    u_b_prev = thrustMomentCmd;

    return u_ad_prev; // return the updated l1 control
}

VectorN<float, 9> ModeAdaptive::unit_vec(Vector3f q, Vector3f q_dot, Vector3f q_ddot)
{
    // This function comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    VectorN<float, 9> uCollection; // for storage of the output
    float nq = q.length();
    Vector3f u = q / nq;
    Vector3f u_dot = q_dot / nq - q * (q * q_dot) / powF(nq, 3);
    Vector3f u_ddot = q_ddot / nq - q_dot / powF(nq, 3) * 2 * (q * q_dot) - q / powF(nq, 3) * (q_dot * q_dot + q * q_ddot) + q * 3 / powF(nq, 5) * powF(q * q_dot, 2);

    uCollection[0] = u[0];
    uCollection[1] = u[1];
    uCollection[2] = u[2];

    uCollection[3] = u_dot[0];
    uCollection[4] = u_dot[1];
    uCollection[5] = u_dot[2];

    uCollection[6] = u_ddot[0];
    uCollection[7] = u_ddot[1];
    uCollection[8] = u_ddot[2];

    return uCollection;
}

Matrix3f ModeAdaptive::hatOperator(Vector3f input)
{
    // hatOperator: convert R^3 to so(3)
    Matrix3f output;
    output = output * 0; // initialize by zero
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.a.x = 0;
    output.a.y = -input.z;
    output.a.z = input.y;
    output.b.x = input.z;
    output.b.y = 0;
    output.b.z = -input.x;
    output.c.x = -input.y;
    output.c.y = input.x;
    output.c.z = 0;

    return output;
}

Vector3f ModeAdaptive::veeOperator(Matrix3f input)
{
    // veeOperator: convert so(3) to R^3
    Vector3f output;
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.x = input.c.y;
    output.y = input.a.z;
    output.z = input.b.x;

    return output;
}

VectorN<float, 4> ModeAdaptive::motorMixing(VectorN<float, 4> thrustMomentCmd)
{
    VectorN<float, 4> w;
#if (!REAL_OR_SITL)       // SITL
    const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    const float a_M = 0.000011667;
    const float b_M = 0.0059137;
#elif (REAL_OR_SITL) // parameters for real drone
    const float L = 0.175; // longer distance between adjacent motors
    const float D = 0.131; // shorter distance between adjacent motors
    const float a_F = 0.0009251;
    const float b_F = 0.021145;
    const float a_M = 0.00001211;
    const float b_M = 0.0009864;
#endif

    // solve for linearizing point
    float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;

    float c_F = 2 * a_F * w0 + b_F;
    float c_M = 2 * a_M * w0 + b_M;

    float thrust_biased = 2 * thrustMomentCmd[0] - 4 * b_F * w0;
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];

    // motor mixing for x layout
    const float c_F4_inv = 1 / (4 * c_F);
    const float c_FL_inv = 1 / (2 * L * c_F);
    const float c_FD_inv = 1 / (2 * D * c_F);
    const float c_M4_inv = 1 / (4 * c_M);

    w[0] = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
    w[1] = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
    w[2] = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
    w[3] = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

    // 2nd shot on solving for motor speed
    // output: VectorN<float, 4> new motor speed
    // input: a_F, b_F, a_M, b_M, w, L, D, thrustMomentCmd

    // motor speed after the second iteration
    VectorN<float, 4> w2;
    w2 = iterativeMotorMixing(w, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // motor speed after the third iteration
    VectorN<float, 4> w3;
    w3 = iterativeMotorMixing(w2, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // logging
    // AP::logger().Write("L1A1", "m1,m2,m3,m4", "ffff",
    //                      (double)w[0],
    //                      (double)w[1],
    //                      (double)w[2],
    //                      (double)w[3]);
    // AP::logger().Write("L1A2", "m1,m2,m3,m4", "ffff",
    //                      (double)w2[0],
    //                      (double)w2[1],
    //                      (double)w2[2],
    //                      (double)w2[3]);
    // AP::logger().Write("L1A3", "m1,m2,m3,m4", "ffff",
    //                      (double)w3[0],
    //                      (double)w3[1],
    //                      (double)w3[2],
    //                      (double)w3[3]);
    return w3;
}

VectorN<float, 4> ModeAdaptive::iterativeMotorMixing(VectorN<float, 4> w_input, VectorN<float, 4> thrustMomentCmd, float a_F, float b_F, float a_M, float b_M, float L, float D)
{
    // The function iterativeMotorMixing computes the motor speed to achieve the desired thrustMoment command
    // input:
    // VectorN<float, 4> w_input -- initial guess of the motor speed (linearizing point)
    // VectorN<float, 4> thrustMomentCmd -- desired thrust and moment command
    // float a_F -- 2nd-order coefficient for motor's thrust-speed curve
    // float b_F -- 1st-order coefficient for motor's thrust-speed curve
    // float a_M -- 2nd-order coefficient for motor's torque-speed curve
    // float b_M -- 1st-order coefficient for motor's torque-speed curve
    // float L -- longer distance between adjacent motors
    // float D -- shorter distance between adjacent motors

    // output:
    // VectorN<float, 4> w_new new motor speed

    VectorN<float, 4> w_new; // new motor speed

    float w1_square = w_input[0] * w_input[0];
    float w2_square = w_input[1] * w_input[1];
    float w3_square = w_input[2] * w_input[2];
    float w4_square = w_input[3] * w_input[3];

    float c_F1 = -a_F * w1_square;
    float c_F2 = -a_F * w2_square;
    float c_F3 = -a_F * w3_square;
    float c_F4 = -a_F * w4_square;

    float c_M1 = -a_M * w1_square;
    float c_M2 = -a_M * w2_square;
    float c_M3 = -a_M * w3_square;
    float c_M4 = -a_M * w4_square;

    float d_F1 = 2 * a_F * w_input[0] + b_F;
    float d_F2 = 2 * a_F * w_input[1] + b_F;
    float d_F3 = 2 * a_F * w_input[2] + b_F;
    float d_F4 = 2 * a_F * w_input[3] + b_F;

    float d_M1 = 2 * a_M * w_input[0] + b_M;
    float d_M2 = 2 * a_M * w_input[1] + b_M;
    float d_M3 = 2 * a_M * w_input[2] + b_M;
    float d_M4 = 2 * a_M * w_input[3] + b_M;

    VectorN<float, 4> coefficientRow1;
    VectorN<float, 4> coefficientRow2;
    VectorN<float, 4> coefficientRow3;
    VectorN<float, 4> coefficientRow4;

    coefficientRow1[0] = d_F1;
    coefficientRow1[1] = d_F2;
    coefficientRow1[2] = d_F3;
    coefficientRow1[3] = d_F4;

    coefficientRow2[0] = -d_F1;
    coefficientRow2[1] = d_F2;
    coefficientRow2[2] = d_F3;
    coefficientRow2[3] = -d_F4;

    coefficientRow3[0] = d_F1;
    coefficientRow3[1] = -d_F2;
    coefficientRow3[2] = d_F3;
    coefficientRow3[3] = -d_F4;

    coefficientRow4[0] = d_M1;
    coefficientRow4[1] = d_M2;
    coefficientRow4[2] = -d_M3;
    coefficientRow4[3] = -d_M4;

    VectorN<float, 16> coefficientMatrixInv = mat4Inv(coefficientRow1, coefficientRow2, coefficientRow3, coefficientRow4);

    VectorN<float, 4> coefficientInvRow1;
    VectorN<float, 4> coefficientInvRow2;
    VectorN<float, 4> coefficientInvRow3;
    VectorN<float, 4> coefficientInvRow4;

    coefficientInvRow1[0] = coefficientMatrixInv[0];
    coefficientInvRow1[1] = coefficientMatrixInv[1];
    coefficientInvRow1[2] = coefficientMatrixInv[2];
    coefficientInvRow1[3] = coefficientMatrixInv[3];

    coefficientInvRow2[0] = coefficientMatrixInv[4];
    coefficientInvRow2[1] = coefficientMatrixInv[5];
    coefficientInvRow2[2] = coefficientMatrixInv[6];
    coefficientInvRow2[3] = coefficientMatrixInv[7];

    coefficientInvRow3[0] = coefficientMatrixInv[8];
    coefficientInvRow3[1] = coefficientMatrixInv[9];
    coefficientInvRow3[2] = coefficientMatrixInv[10];
    coefficientInvRow3[3] = coefficientMatrixInv[11];

    coefficientInvRow4[0] = coefficientMatrixInv[12];
    coefficientInvRow4[1] = coefficientMatrixInv[13];
    coefficientInvRow4[2] = coefficientMatrixInv[14];
    coefficientInvRow4[3] = coefficientMatrixInv[15];

    VectorN<float, 4> shiftedCmd;
    shiftedCmd[0] = thrustMomentCmd[0] - c_F1 - c_F2 - c_F3 - c_F4;
    shiftedCmd[1] = 2 * thrustMomentCmd[1] / L + c_F1 - c_F2 - c_F3 + c_F4;
    shiftedCmd[2] = 2 * thrustMomentCmd[2] / D - c_F1 + c_F2 - c_F3 + c_F4;
    shiftedCmd[3] = thrustMomentCmd[3] - c_M1 - c_M2 + c_M3 + c_M4;

    w_new[0] = coefficientInvRow1 * shiftedCmd;
    w_new[1] = coefficientInvRow2 * shiftedCmd;
    w_new[2] = coefficientInvRow3 * shiftedCmd;
    w_new[3] = coefficientInvRow4 * shiftedCmd;

    return w_new;
}

VectorN<float, 16> ModeAdaptive::mat4Inv(VectorN<float, 4> coefficientRow1, VectorN<float, 4> coefficientRow2, VectorN<float, 4> coefficientRow3, VectorN<float, 4> coefficientRow4)
{
    // inverse of a 4x4 matrix
    // modified from https://stackoverflow.com/a/44446912
    float A2323 = coefficientRow3[2] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[2];
    float A1323 = coefficientRow3[1] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[1];
    float A1223 = coefficientRow3[1] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[1];
    float A0323 = coefficientRow3[0] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[0];
    float A0223 = coefficientRow3[0] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[0];
    float A0123 = coefficientRow3[0] * coefficientRow4[1] - coefficientRow3[1] * coefficientRow4[0];
    float A2313 = coefficientRow2[2] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[2];
    float A1313 = coefficientRow2[1] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[1];
    float A1213 = coefficientRow2[1] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[1];
    float A2312 = coefficientRow2[2] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[2];
    float A1312 = coefficientRow2[1] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[1];
    float A1212 = coefficientRow2[1] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[1];
    float A0313 = coefficientRow2[0] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[0];
    float A0213 = coefficientRow2[0] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[0];
    float A0312 = coefficientRow2[0] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[0];
    float A0212 = coefficientRow2[0] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[0];
    float A0113 = coefficientRow2[0] * coefficientRow4[1] - coefficientRow2[1] * coefficientRow4[0];
    float A0112 = coefficientRow2[0] * coefficientRow3[1] - coefficientRow2[1] * coefficientRow3[0];

    float det = coefficientRow1[0] * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223) - coefficientRow1[1] * (coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223) + coefficientRow1[2] * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123) - coefficientRow1[3] * (coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    det = 1 / det;

    VectorN<float, 16> inv;
    inv[0] = det * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223);
    inv[1] = det * -(coefficientRow1[1] * A2323 - coefficientRow1[2] * A1323 + coefficientRow1[3] * A1223);
    inv[2] = det * (coefficientRow1[1] * A2313 - coefficientRow1[2] * A1313 + coefficientRow1[3] * A1213);
    inv[3] = det * -(coefficientRow1[1] * A2312 - coefficientRow1[2] * A1312 + coefficientRow1[3] * A1212);
    inv[4] = det * -(coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223);
    inv[5] = det * (coefficientRow1[0] * A2323 - coefficientRow1[2] * A0323 + coefficientRow1[3] * A0223);
    inv[6] = det * -(coefficientRow1[0] * A2313 - coefficientRow1[2] * A0313 + coefficientRow1[3] * A0213);
    inv[7] = det * (coefficientRow1[0] * A2312 - coefficientRow1[2] * A0312 + coefficientRow1[3] * A0212);
    inv[8] = det * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123);
    inv[9] = det * -(coefficientRow1[0] * A1323 - coefficientRow1[1] * A0323 + coefficientRow1[3] * A0123);
    inv[10] = det * (coefficientRow1[0] * A1313 - coefficientRow1[1] * A0313 + coefficientRow1[3] * A0113);
    inv[11] = det * -(coefficientRow1[0] * A1312 - coefficientRow1[1] * A0312 + coefficientRow1[3] * A0112);
    inv[12] = det * -(coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    inv[13] = det * (coefficientRow1[0] * A1223 - coefficientRow1[1] * A0223 + coefficientRow1[2] * A0123);
    inv[14] = det * -(coefficientRow1[0] * A1213 - coefficientRow1[1] * A0213 + coefficientRow1[2] * A0113);
    inv[15] = det * (coefficientRow1[0] * A1212 - coefficientRow1[1] * A0212 + coefficientRow1[2] * A0112);

    return inv;
}

#endif