/*
This is a Rigid Body Dynamics Solver, which takes the Input of 9x1 State Vector {uvw,pqr,euler}
and solves one time step for it, start by calling the constructor and then using the method rk4_solver
*/
#pragma once
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "structs.h"
//float eps = 0.00000011921; // matlab float eps
const double eps = 0.000000000000000222044605;
extern double dt;
extern int step;

class RBDSolve{
    private:
        int i;
        double *time_v;
        double m, w_dot_state, g, cos_theta,v_tot,z0;
        Eigen::Matrix<double,3,1> v, v_e, v_dot;
        Eigen::Matrix<double,3,1> omega, delta_omega_dot;
        Eigen::Matrix<double,3,1> euler, euler_dot;
        Eigen::Matrix<double,3,3> J, R, I;
        Eigen::Matrix<double,6,7> SD;
        Eigen::Matrix<double,6,4> CD;
        Eigen::Matrix<double,4,1>* Controls;
        Eigen::Matrix<double,6,1> Aerodynamic_accel;
        Eigen::Matrix<double,3,1> delta_v, delta_omega, delta_F, delta_M;
        Eigen::Matrix<double,9,1> y,y_dot; Eigen::Matrix<double,7,1> delta_y;
        Eigen::Matrix<double,9,1>* states;
        aircraft_data ac;
        flight_path *str_h;
    public:
        Eigen::Matrix<double,3,1> F_aero;
        Eigen::Matrix<double,3,1> F_grav;
        Eigen::Matrix<double,3,1> F_b;
        Eigen::Matrix<double,3,1> F_g0;
        Eigen::Matrix<double,3,1> F_g;
        Eigen::Matrix<double,3,1> M_total;
        RBDSolve(aircraft_data &ac, Eigen::Matrix<double,4,1>* Controls,flight_path *str_h ,Eigen::Matrix<double,9,1>* states);
        void eulerToRotationMatrix();
        void rk4Solver();
        Eigen::Matrix<double,9,1> RBDEquations(Eigen::Matrix<double,9,1> y);

        // // Getters for logging
        // Eigen::Matrix<double,6,1> getAerodynamicAccel() const { return Aerodynamic_accel; }
        // Eigen::Matrix<double,3,1> getAeroForces() const { return F_aero; }
        // Eigen::Matrix<double,3,1> getGravForces() const { return F_grav; }
        // Eigen::Matrix<double,3,1> getTotalForces() const { return F_b; }
        // Eigen::Matrix<double,3,1> getTotalMoments() const { return M_total; }
        // double getWDotAccel() const { return w_dot_state; }
        // double getVDotZ() const { return v_dot(2); }
        // double getCurrentTime() const { return current_time; }
};
