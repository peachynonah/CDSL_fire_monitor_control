#pragma once
# include <cmath>
# include <vector>

#define MM_to_M 1e-3

class ModelDynamics {
    public:
        ModelDynamics();
        //Robot properties
        double DH_param_dist[2];
        double link_mass[2];
        double com_x[2];
        double com_y[2];
        double com_z[2];
        double mass_matrix[2][2];
        double nonlinear_dynamics_term[2];

    public:
        std::vector<double> get_mass_matrix(double theta1, double theta2);
        std::vector<double> get_nonlinear_dynamics(double theta1, double theta2, double theta1_dot, double theta2_dot);
};

