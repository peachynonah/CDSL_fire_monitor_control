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
    tau = torque_saturate(input_tau, 2000); // Example max torque norm
    return tau;
}


PDController::PDController() {
    // Initialize gains
    Kp_PD[0] = -1020.0; Kp_PD[1] = 50.0;
    Kd_PD[0] = 10.0; Kd_PD[1] = 5.0;
}

double PDController::calculateTau(int index, double joint_error, double joint_error_dot) {
    tau[index] = Kp_PD[index] * joint_error + Kd_PD[index] * joint_error_dot;    
    return tau[index];
}


FLController::FLController() {
    // Initialize gains
    Kp_FL[0] = 1.0; Kp_FL[1] = 1.0;
    Kd_FL[0] = 0.1; Kd_FL[1] = 0.1;
}

double FLController::calculateTau(int index, double joint_error, double joint_error_dot) {
    //ModelReference: This is a placeholder for the fuzzy logic controller's tau calculation
    return 0.0;
}

