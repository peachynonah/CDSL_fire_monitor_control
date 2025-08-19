# include <iostream>
# include "controller.h"

using namespace std;

ManualController ManualCont;
PDController PDCont;
FLController FLCont;

int main() {
 
    // Example usage of ManualController
    double manual_tau[2];
    manual_tau[0] = ManualCont.calculateTau(5.0);
    manual_tau[1] = ManualCont.calculateTau(10.0);
    
    cout << "Manual Tau: " << manual_tau[0] << ", " << manual_tau[1] << endl;

    double PD_tau[2];
    double joint_error[2] = {2.0, 3.0};
    double joint_error_dot[2] = {0.5, 0.7};
    PD_tau[0] = PDCont.calculateTau(0, joint_error[0], joint_error_dot[0]);
    PD_tau[1] = PDCont.calculateTau(1, joint_error[1], joint_error_dot[1]);

    cout << "PD Tau: " << PD_tau[0] << ", " << PD_tau[1] << endl;

    double FL_tau[2];   
    FL_tau[0] = FLCont.calculateTau(0, joint_error[0], joint_error_dot[0]);
    FL_tau[1] = FLCont.calculateTau(1, joint_error[1], joint_error_dot[1]);
    cout << "FL Tau: " << FL_tau[0] << ", " << FL_tau[1] << endl;
    
    return 0;
}


