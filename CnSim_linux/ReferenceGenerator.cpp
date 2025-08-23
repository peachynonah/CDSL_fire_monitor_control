#include "ReferenceGenerator.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

// 생성자
ReferenceGenerator::ReferenceGenerator() {
    // //situation1: for joint 2.
    // time_ref_start = 0.0; time_ref_fin = 10.0; current_joint1_position = -0.39;
    // alpha_coeffs[5] = 1.7261498e-07; alpha_coeffs[4] = -8.199212e-05; alpha_coeffs[3] = +0.001064459;
    // alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;

    // //situation1: for joint 2.
    // time_ref_start = 0.0; time_ref_fin = 10.0; current_joint1_position = -0.10;
    // alpha_coeffs[5] = -2.30153307e-07; alpha_coeffs[4] = 0.00010932282; alpha_coeffs[3] = -0.0014192787;
    // alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;

    /////////
    // //situation2: start at 0.0, finish at pi/2 : anticlockwise
    // time_ref_start = 0.0; time_ref_fin = 30.0; current_joint1_position = 0.378;
    // alpha_coeffs[5] = 1.33282e-09; alpha_coeffs[4] = -5.89773e-06; alpha_coeffs[3] = 0.00023391;
    // alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;

    // //situation3: start at 0.0, finish at minus pi/2 : clockwise
    // time_ref_start = 0.0; time_ref_fin = 30.0; current_joint1_position = 1.36;
    // alpha_coeffs[5] = -1.33282e-09; alpha_coeffs[4] = 5.89773e-06; alpha_coeffs[3] = -0.00023391;
    // alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;

    ////////////
    // //situation4: start at 0.0, finish at pi/2 : anticlockwise
    // time_ref_start = 0.0; time_ref_fin = 10.0; current_joint1_position = -0.413;
    // alpha_coeffs[5] = 1.035690e-06; alpha_coeffs[4] = -0.0004919527; alpha_coeffs[3] = 0.00638675;
    // alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;

    //situation5: start at 0.0, finish at minus pi/2 : clockwise
    time_ref_start = 0.0; time_ref_fin = 10.0; current_joint1_position = 0.0;
    alpha_coeffs[5] = -1.035690e-06; alpha_coeffs[4] = 0.0004919527; alpha_coeffs[3] = -0.00638675;
    alpha_coeffs[2] = 0.0; alpha_coeffs[1] = 0.0; alpha_coeffs[0] = current_joint1_position;


}


// 1. desired reference position at current time
double ReferenceGenerator::get_position(double current_time) {
    // 궤적 시작 전에는 초기 위치를 반환
    if (current_time < time_ref_start) {
        return 0.0;
    }

    // 궤적 끝난 후에는 최종 위치를 반환
    else if (current_time >= time_ref_fin) {
        double t_diff_final = time_ref_fin - time_ref_start;
        return alpha_coeffs[5] * std::pow(t_diff_final, 5) +
               alpha_coeffs[4] * std::pow(t_diff_final, 4) +
               alpha_coeffs[3] * std::pow(t_diff_final, 3) +
               alpha_coeffs[2] * std::pow(t_diff_final, 2) +
               alpha_coeffs[1] * t_diff_final +
               alpha_coeffs[0];
    }

    else if (time_ref_start <= current_time < time_ref_fin){
    // 궤적 구간 내에서는 다항식 계산
    double t_diff = current_time - time_ref_start;

    printf("inner loop reference generator, time difference is %f", t_diff);  
    return alpha_coeffs[5] * std::pow(t_diff, 5) +
           alpha_coeffs[4] * std::pow(t_diff, 4) +
           alpha_coeffs[3] * std::pow(t_diff, 3) +
           alpha_coeffs[2] * std::pow(t_diff, 2) +
           alpha_coeffs[1] * t_diff +
           alpha_coeffs[0];
    }
}

// 2. desired reference velocity at current time
double ReferenceGenerator::get_velocity(double current_time) {
    if (current_time < time_ref_start || current_time >= time_ref_fin) {
        return 0.0;
    }
    
    double t_diff = current_time - time_ref_start;
    return 5 * alpha_coeffs[5] * std::pow(t_diff, 4) +
           4 * alpha_coeffs[4] * std::pow(t_diff, 3) +
           3 * alpha_coeffs[3] * std::pow(t_diff, 2) +
           2 * alpha_coeffs[2] * t_diff +
           alpha_coeffs[1];
}

// 3. desired reference accleration at current time
double ReferenceGenerator::get_acceleration(double current_time) {
    if (current_time < time_ref_start || current_time >= time_ref_fin) {
        return 0.0;
    }
    
    double t_diff = current_time - time_ref_start;
    return 20 * alpha_coeffs[5] * std::pow(t_diff, 3) +
           12 * alpha_coeffs[4] * std::pow(t_diff, 2) +
           6 * alpha_coeffs[3] * t_diff +
           2 * alpha_coeffs[2];
}