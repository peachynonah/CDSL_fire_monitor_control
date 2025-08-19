# include <iostream>
# include "controller.h"


using namespace std;




ManualController::ManualController() {
    // Constructor to initialize the manual controller
}

double ManualController::calculateTau(double tau) {
    // Implement the calculation for tau
    return tau;
}


PDController::PDController() {
    // Initialize gains
    Kp_PD[0] = 1.0; Kp_PD[1] = 1.0;
    Kd_PD[0] = 0.1; Kd_PD[1] = 0.1;
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

