# include <cmath>
# include <vector>
# include "model_dynamics.h"

using namespace std;
// Constructor
ModelDynamics::ModelDynamics() {
    // Robot properties
    DH_param_dist[0] = 83.7*MM_to_M; DH_param_dist[1] = 291*MM_to_M; // meters
    Link_mass[0] = 0.0; Link_mass[1] = 0.0; // kg
    com_x[0] = 0.0; com_x[1] = 0.0; // meters
    com_y[0] = 0.0; com_y[1] = 0.0; // meters
    com_z[0] = 0.0; com_z[1] = 0.0; // meters
    mass_matrix[0][0] = 0.0; mass_matrix[0][1] = 0.0;
    mass_matrix[1][0] = 0.0; mass_matrix[1][1] = 0.0;
    nonlinear_dynamics_term[0] = 0.0; nonlinear_dynamics_term[1] = 0.0;

}

// Method to compute the mass matrix given joint angles
std::vector<double> ModelDynamics::get_mass_matrix(double theta1, double theta2) {

    //redefine robot properties
    double d1 = DH_param_dist[0]; double d2 = DH_param_dist[1];
    double m1 = Link_mass[0]; double m2 = Link_mass[1];
    double c1 = std::cos(theta1); double s1 = std::sin(theta1);
    double c2 = std::cos(theta2); double s2 = std::sin(theta2);

    // Placeholder implementation: Replace with actual mass matrix computation
    mass_matrix[0][0]   = m2 * std::pow(d2, 2) 
                        + m1 * (std::pow(com_x[0], 2) + std::pow(com_y[0], 2)) 
                        + m2 * (std::pow(com_y[1], 2) + std::pow(com_z[1], 2))
                        + m2 * std::pow(c2, 2) * (std::pow(com_x[1], 2) - std::pow(com_y[1], 2))
                        + 2 * d2 * m2 * com_z[1] 
                        - 2 * m2 * com_x[1] * com_y[1] * s2;
    mass_matrix[0][1] = 0.0;
    mass_matrix[1][0] = 0.0;
    mass_matrix[1][1] = 0.0;
    return {mass_matrix[0][0], mass_matrix[0][1], mass_matrix[1][0], mass_matrix[1][1]};
}

// Method to compute the nonlinear dynamics terms given joint angles and velocities
std::vector<double> ModelDynamics::get_nonlinear_dynamics(double theta1, double theta2, double theta1_dot, double theta2_dot) {
    
    //redefine robot properties
    double d1 = DH_param_dist[0]; double d2 = DH_param_dist[1];
    double m1 = Link_mass[0]; double m2 = Link_mass[1];
    double c1 = std::cos(theta1); double s1 = std::sin(theta1);
    double c2 = std::cos(theta2); double s2 = std::sin(theta2);
    
    // Placeholder implementation: Replace with actual nonlinear dynamics computation
    nonlinear_dynamics_term[0] = 0.0;
    nonlinear_dynamics_term[1] = 0.0;
    return {nonlinear_dynamics_term[0], nonlinear_dynamics_term[1]};
}   

