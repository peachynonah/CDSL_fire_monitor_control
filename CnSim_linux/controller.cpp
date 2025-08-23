# include <iostream>
# include "controller.h"


using namespace std;


// functions
int torque_saturate(int torque, int max_torque_norm) {
    int max_torque = max_torque_norm;
    int min_torque = -max_torque_norm;

    if (torque > max_torque) {
        return max_torque;
    } else if (torque < min_torque) {
        return min_torque;
    }
    
    return torque;
}

// double get_velocity_numerical(double theta_d_curr, double theta_d_prev, double sampling_period){
//     double theta_dot_d = (theta_d_curr - theta_d_prev) / (sampling_period);
//     return theta_dot_d;
// }


//controller implementations

ManualController::ManualController() {
    // Constructor to initialize the manual controller
}

int ManualController::calculateTau(int input_tau) {
    // Implement the calculation for tau
    tau = torque_saturate(input_tau, 990); // Example max torque norm
    return tau;
}


PDController::PDController() {
    // Initialize gains
    Kp_PD[0] = 1100.0; Kp_PD[1] = 0.0; // -1020 is quite good P gain
    Kd_PD[0] = 3000.0; Kd_PD[1] = 0.0;
}

double PDController::calculateTau(int index, double joint_error, double joint_error_dot) {
    tau[index] = Kp_PD[index] * joint_error + Kd_PD[index] * joint_error_dot;
    // printf("\nin controller, generated torque is  : %f\n", tau[index]);
    tau[index] = - torque_saturate(tau[index], 990);    
    return tau[index];
}

//to develop
double PDController::tauPropo(int index, double joint_error) {
    tau_propo[index] = Kp_PD[index] * joint_error;
    // printf("\nin controller, generated torque is  : %f\n", tau[index]);
    tau_propo[index] = - torque_saturate(tau_propo[index], 990);    
    return tau_propo[index];
}

double PDController::tauDeriv(int index, double joint_error_dot) {
    tau_deriv[index] = Kd_PD[index] * joint_error_dot;
    // printf("\nin controller, generated torque is  : %f\n", tau[index]);
    tau_deriv[index] = - torque_saturate(tau_deriv[index], 990);    
    return tau_deriv[index];
}


FLController::FLController() {
    // Initialize gains
    Kp_FL[0] = 1.0; Kp_FL[1] = 1.0;
    Kd_FL[0] = 0.1; Kd_FL[1] = 0.1;
}

double FLController::calculateTau(int index, double joint_error, double joint_error_dot) {
    //ModelReference: This is a placeholder for the fuzzy logic controller's tau calculation
    tau[index] = 0.0;
    tau[index] = torque_saturate(tau[index], 990);
    return tau[index];
}


LowPassFilter::LowPassFilter(){
    time_param = 0.444; // time_param = sampling_time / (time_constant + sampling_time) = 4ms / (5ms + 4ms)
    unfiltered_value = 0.0;
    filtered_value = 0.0;
    filtered_value_prev = 0.0;
}

double LowPassFilter::calculate_lowpass_filter(double unfiltered_value, double filtered_value_prev) {
    filtered_value = time_param * unfiltered_value + (1 - time_param) * filtered_value_prev;
    return filtered_value;
}//y_k = time_param * u_k + (1 - time_param) * y_k-1
