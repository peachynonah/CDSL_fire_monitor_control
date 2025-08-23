#pragma once

#define ctrl_manual 0
#define ctrl_pd 1
#define ctrl_fl 2

class ManualController {
    public:
        ManualController();
        int tau;
        
    public:
        int calculateTau(int input_tau = 0);
};


class PDController {

    public:
        double Kp_PD[2];
        double Kd_PD[2];
        PDController();
        
        double tau[2];
        double tau_propo[2];
        double tau_deriv[2];
        double error[2];

    public:
        double calculateTau(int index, double joint_error, double joint_error_dot);
        double tauPropo(int index, double joint_error);
        double tauDeriv(int index, double joint_error_dot);
    
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


class LowPassFilter {
    public:
        double time_param;
        double unfiltered_value;
        double filtered_value;
        double filtered_value_prev;
        LowPassFilter();

    public:
        double calculate_lowpass_filter(double unfiltered_value, double filtered_value_prev);
};