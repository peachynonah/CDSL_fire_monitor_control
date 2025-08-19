#pragma once

class ManualController {
    public:
        ManualController();
        
    public:
        double calculateTau(double tau);
};


class PDController {

    public:
        double Kp_PD[2];
        double Kd_PD[2];
        PDController();
        
        double tau[2];
        double error[2];

    public:
        double calculateTau(int index, double joint_error, double joint_error_dot);
    
};


class FLController {
    public:
        double Kp_FL[2];
        double Kd_FL[2];
        FLController();
        
        double tau[2];
        double error[2];

    public:
        double calculateTau(int index, double joint_error, double joint_error_dot);
    
};
