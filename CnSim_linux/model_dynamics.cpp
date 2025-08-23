# include <iostream>
# include <cmath>
# include <vector>
# include "model_dynamics.h"

using namespace std;
// Constructor
ModelDynamics::ModelDynamics() {
    // Robot properties
    DH_param_dist[0] = 83.7*MM_to_M; DH_param_dist[1] = 291*MM_to_M; // meters
    link_mass[0] = 10.388; link_mass[1] = 8.615; // kg
    com_x[0] = -22.521*MM_to_M; com_x[1] = 273.091*MM_to_M; // meters
    com_y[0] = -0.315*MM_to_M; com_y[1] = 3.129*MM_to_M; // meters
    com_z[0] = 170.935*MM_to_M; com_z[1] = 0.071*MM_to_M; // meters
    grav_acc = 9.8; // meter / second^2

    //Initialize dynamics terms
    mass_matrix[0][0] = 0.0; mass_matrix[0][1] = 0.0;
    mass_matrix[1][0] = 0.0; mass_matrix[1][1] = 0.0;
    nonlinear_dynamics_term[0] = 0.0; nonlinear_dynamics_term[1] = 0.0;
}

// Method to compute the mass matrix given joint angles
std::vector<double> ModelDynamics::get_mass_matrix(double theta1, double theta2) {

    //redefine robot properties
    double d1 = DH_param_dist[0]; double d2 = DH_param_dist[1];
    double m1 = link_mass[0]; double m2 = link_mass[1];
    double c1 = std::cos(theta1); double s1 = std::sin(theta1);
    double c2 = std::cos(theta2); double s2 = std::sin(theta2);

    // Placeholder implementation: Replace with actual mass matrix computation
    mass_matrix[0][0]   = m2 * std::pow(d2, 2) 
                        + m1 * (std::pow(com_x[0], 2) + std::pow(com_y[0], 2)) 
                        + m2 * (std::pow(com_y[1], 2) + std::pow(com_z[1], 2))
                        + m2 * std::pow(c2, 2) * (std::pow(com_x[1], 2) - std::pow(com_y[1], 2))
                        + 2 * d2 * m2 * com_z[1] 
                        - 2 * m2 * com_x[1] * com_y[1] * s2;
    mass_matrix[0][1] = -m2*(d2*com_x[1]*s2 + com_y[1]*com_z[1]*c2 + com_x[1]*com_z[1]*s2 + d2*com_y[1]*c2);
    mass_matrix[1][0] = mass_matrix[0][1];
    mass_matrix[1][1] = m2*(std::pow(com_x[1], 2) + std::pow(com_y[1], 2));
    return {mass_matrix[0][0], mass_matrix[0][1], mass_matrix[1][0], mass_matrix[1][1]};
}

// Method to compute the nonlinear dynamics terms given joint angles and velocities
std::vector<double> ModelDynamics::get_nonlinear_dynamics(double theta1, double theta2, double theta1_dot, double theta2_dot) {
    
    //redefine robot properties
    double d1 = DH_param_dist[0]; double d2 = DH_param_dist[1];
    double m1 = link_mass[0]; double m2 = link_mass[1];
    double c1 = std::cos(theta1); double s1 = std::sin(theta1);
    double c2 = std::cos(theta2); double s2 = std::sin(theta2);
    
    // Placeholder implementation: Replace with actual nonlinear dynamics computation    
    // this part is sorted by gpt...
    //h1
    h1_grav =  grav_acc * (
    - m1 * com_y[0] * s1
    + m2 * com_z[1] * s1
    + d2 * m2 * s1
    + m1 * com_x[0] * c1
    + m2 * com_x[1] * c1 * c2
    - m2 * com_y[1] * c1 * s2
    );

    h1_cent = (
    - d2 * m2 * com_x[1] * c2
    + d2 * m2 * com_y[1] * s2
    - m2 * com_x[1] * com_z[1] * c2
    + m2 * com_y[1] * com_z[1] * s2
    ) * std::pow(theta2_dot, 2);

    h1_cori = (
    - m2 * std::pow(com_x[1], 2) * std::sin(2.0 * theta2)
    + m2 * std::pow(com_y[1], 2) * std::sin(2.0 * theta2)
    + 2.0 * m2 * com_x[1] * com_y[1]
    - 4.0 * m2 * com_x[1] * com_y[1] * std::pow(c2, 2)
    ) * theta1_dot * theta2_dot;

    nonlinear_dynamics_term[0] = h1_grav + h1_cori + h1_cent;

    // h2 
    h2_grav = 
    - grav_acc * m2 * com_y[1] * c2 * s1
    - grav_acc * m2 * com_x[1] * s1 * s2;

    h2_cent = (
    - m2 * com_x[1] * com_y[1]
    + 0.5 * m2 * std::pow(com_x[1], 2) * std::sin(2.0 * theta2)
    - 0.5 * m2 * std::pow(com_y[1], 2) * std::sin(2.0 * theta2)
    + 2.0 * m2 * com_x[1] * com_y[1] * std::pow(c2, 2)
    ) * std::pow(theta1_dot, 2);

    h2_cori = 0.0;

    nonlinear_dynamics_term[1] = h2_grav + h2_cent + h2_cori;
    return {nonlinear_dynamics_term[0], nonlinear_dynamics_term[1]};
}   

