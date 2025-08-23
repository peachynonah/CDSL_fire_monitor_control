# include <iostream>
# include "controller.h"
# include "model_dynamics.h"
# include <vector>
using namespace std;

ModelDynamics m_model_dynamics;

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
    Kp_PD[0] = 1100.0; Kp_PD[1] = 9000.0; 
    Kd_PD[0] = 3000.0; Kd_PD[1] = 1000.0; 
}

double PDController::calculateTau(int index, double joint_error, double joint_error_dot) {
    tau[index] = Kp_PD[index] * joint_error + Kd_PD[index] * joint_error_dot;
    // printf("\nin controller, generated torque is  : %f\n", tau[index]);
    tau[index] = - torque_saturate(tau[index], 2500);    
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
    Kp_FL[0] = 0.01; Kp_FL[1] = 0.01;
    Kd_FL[0] = 0.001; Kd_FL[1] = 0.001;
}

double FLController::calculateTau(int index, double theta1_ddot_desired_d, double joint_error, double joint_error_dot, 
                                  double theta1, double theta2, double theta1_dot, double theta2_dot){
    tau[index] = 0.0;
    //ModelReference: This is a placeholder for FL controller's tau calculation
    std::vector<double> mass_matrix = m_model_dynamics.get_mass_matrix(theta1, theta2);
    double m11 = mass_matrix[0]; double m12 = mass_matrix[1]; 
    double m21 = mass_matrix[2]; double m22 = mass_matrix[3]; 
    printf("\nin FL controller, the mass matrix {m11, m12, m21, m22 is}: (%f, %f, %f, %f)\n", m11, m12, m21, m22);	
    
    std::vector<double> nonlinear_dynamics_term = m_model_dynamics.get_nonlinear_dynamics(theta1, theta2, theta1_dot, theta2_dot);
    double h1 = nonlinear_dynamics_term[0];
    double h2 = nonlinear_dynamics_term[1];
    printf("\nin FL controller, the nonlinear term h {h1, h2 is}: (%f, %f)\n", h1, h2);

    //temp..
    double joint2_error = 0.0; double joint2_error_dot = 0.0; double theta2_ddot_desired_d = 0.0;
    double temp1 = theta1_ddot_desired_d + Kp_FL[0] * joint_error + Kd_FL[0] * joint_error_dot;
    double temp2 = theta2_ddot_desired_d + Kp_FL[1] * joint2_error + Kd_FL[1] * joint2_error_dot;
    tau[0] = m11 * (temp1) + m12 * (temp2) + h1; // Nm
    tau[1] = h2;

    tau[index] = 1e3* tau[index]; // mNm
    tau[index] = static_cast<int>((1e3 / 52.8)* tau[index]); // Thousand Per Rated Torque
    printf("\nin FL controller, generated torque {tau[%d]} is: (%f)\n", index, tau[index]);
    tau[index] = - torque_saturate(tau[index], 0);
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
