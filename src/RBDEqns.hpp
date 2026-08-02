#pragma once

#include "readxslx.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "derivatives.hpp"

//float eps = 0.00000011921; // matlab float eps
double eps = 0.000000000000000222044605;

class RBDsolve{
    private:
        int i;
        double m, w_dot_state, g, cos_theta,v_tot,z0;
        Eigen::Matrix<double,3,1> v, v_e, v_dot;
        Eigen::Matrix<double,3,1> omega, delta_omega_dot;
        Eigen::Matrix<double,3,1> euler, euler_dot;
        Eigen::Matrix<double,3,3> J, R, I;
        Eigen::Matrix<double,6,7> SD;
        Eigen::Matrix<double,7,1> delta_states;
        Eigen::Matrix<double,6,4> CD;
        Eigen::Matrix<double,4,1>* Controls;
        Eigen::Matrix<double,6,1> Aerodynamic_accel;
        Eigen::Matrix<double,3,1> delta_v, delta_omega, delta_F, delta_M;
        Eigen::Matrix<double,9,1> states_dot;
        Eigen::Matrix<double,9,1> states0;
        aircraft_data ac;

    public:
        Eigen::Matrix<double,3,1> F_aero;
        Eigen::Matrix<double,3,1> F_grav;
        Eigen::Matrix<double,3,1> F_b;
        Eigen::Matrix<double,3,1> F_g0;
        Eigen::Matrix<double,3,1> F_g;
        Eigen::Matrix<double,3,1> M_total;
        RBDsolve(aircraft_data &ac, Eigen::Matrix<double,4,1>* Controls,bool Autopiloted){
                this->ac = ac; this->g = ac.g; this->m = ac.m; this->I = ac.Inertia;this->z0=ac.z0;
                this->F_g0 = ac.mg0; this->v = ac.V0;
                this->omega = ac.omega0; this->euler = ac.euler0;
                this->SD = ac.SD; this->CD = ac.CD;
                this->w_dot_state = 0;
                this->Controls = Controls;
                this->current_time = 0.0;
                // Initialize force vectors
                F_aero.setZero();
                F_grav.setZero();
                F_b.setZero();
                M_total.setZero();
        }
        // Getters for logging
        Eigen::Matrix<double,6,1> getAerodynamicAccel() const { return Aerodynamic_accel; }
        Eigen::Matrix<double,3,1> getAeroForces() const { return F_aero; }
        Eigen::Matrix<double,3,1> getGravForces() const { return F_grav; }
        Eigen::Matrix<double,3,1> getTotalForces() const { return F_b; }
        Eigen::Matrix<double,3,1> getTotalMoments() const { return M_total; }
        double getWDotAccel() const { return w_dot_state; }
        double getVDotZ() const { return v_dot(2); }
        double getCurrentTime() const { return current_time; }
        Eigen::Matrix<double,3,1> delta(Eigen::Matrix<double,3,1> state, Eigen::Matrix<double,3,1> state0){
            return state - state0;
        }
        void eulerToRotationMatrix(const Eigen::Matrix<double,3,1>& euler){
            double phi = euler(0); double c_phi = std::cos(phi); double s_phi = std::sin(phi);
            double theta = euler(1); double c_theta = std::cos(theta); double s_theta = std::sin(theta);
            double psi = euler(2); double c_psi = std::cos(psi); double s_psi = std::sin(psi);

            R << c_psi * c_theta,
                 c_psi * s_theta * s_phi - s_psi * c_phi,
                 c_psi * s_theta * c_phi + s_psi * s_phi,

                 s_psi * c_theta,
                 s_psi * s_theta * s_phi + c_psi * c_phi,
                 s_psi * s_theta * c_phi - c_psi * s_phi,

                -s_theta,
                 c_theta * s_phi,
                 c_theta * c_phi;
        }

        Eigen::Matrix<double,9,1> Equations(Eigen::Matrix<double,9,1> states, double time = 0.0f){
            v = states.segment<3>(0);
            omega = states.segment<3>(3);
            euler = states.segment<3>(6);
            current_time = time;

            // calculate forces and moments
            delta_v = delta(v, ac.V0);
            delta_omega = delta(omega, ac.omega0);

            // Build delta_states
            delta_states(0) = delta_v(0);
            delta_states(1) = delta_v(1);
            delta_states(2) = delta_v(2);
            delta_states(3) = delta_omega(0);
            delta_states(4) = delta_omega(1);
            delta_states(5) = delta_omega(2);
            delta_states(6) = w_dot_state;

            // Calculate Aerodynamic Accelerations
            Aerodynamic_accel = SD * delta_states + CD * (*Controls);

            // Compute aerodynamic forces
            for (i = 0; i < 3; i++){
                delta_F[i] = Aerodynamic_accel[i] * m;
                F_aero[i] = delta_F[i];  // Store aerodynamic forces
            }
            // Compute moments
            for (i = 3; i < 6; i++){
                delta_M[i-3] = Aerodynamic_accel[i] * I.coeff(i-3, i-3);
            }
            M_total = delta_M;

            // Compute gravitational forces
            F_grav(0) = -m * g * sin(euler[1]) - F_g0[0];
            F_grav(1) =  m * g * cos(euler[1]) * sin(euler[0]) - F_g0[1];
            F_grav(2) =  m * g * cos(euler[1]) * cos(euler[0]) - F_g0[2];

            // Total body forces
            F_b = delta_F + F_grav;

            // linear newton
            v_dot = F_b / m - omega.cross(v); // Matlab 6DOF Abb

            // Angular Newton
            delta_omega_dot = delta_M - omega.cross(I * omega);
            delta_omega_dot = I.ldlt().solve(delta_omega_dot);

            // Euler Kinematics
            cos_theta = std::cos(euler[1]);
            if (std::abs(cos_theta) < eps) cos_theta = eps;

            J << 1, sin(euler[0])*tan(euler[1]), cos(euler[0])*tan(euler[1]),
                 0, cos(euler[0]),               -sin(euler[0]),
                 0, sin(euler[0])/cos_theta,      cos(euler[0])/cos_theta;
            euler_dot = J * omega;

            // Build states_dot (9x1)
            states_dot(0) = v_dot(0);
            states_dot(1) = v_dot(1);
            states_dot(2) = v_dot(2);
            states_dot(3) = delta_omega_dot(0);
            states_dot(4) = delta_omega_dot(1);
            states_dot(5) = delta_omega_dot(2);
            states_dot(6) = euler_dot(0);
            states_dot(7) = euler_dot(1);
            states_dot(8) = euler_dot(2);

            return states_dot;
        }
        // Eigen::Matrix<double,9,1> Verify(Eigen::Matrix<double,9,1> states){
        //     double y1 = states(0);
        //     double y2 = states(1);
        //     double t  = states(2);

        //     Eigen::Matrix<double,9,1> states_dot2;
        //     states_dot2.setZero();
        //     states_dot2(0) = std::sin(t) + std::cos(y1) + std::cos(y2);
        //     states_dot2(1) = std::cos(t) + std::sin(y2) + std::cos(y1);
        //     states_dot2(2) = 1.0f;
        //     return states_dot2;
        // }

        void updatewdot(double a){
            w_dot_state = a;
        }
        void h_calculation(Eigen::Matrix<double,9,1> y,double dt,flight_path &str_h){
            str_h.alpha=std::atan2(y(2),y(0));
            //std::cout<<"alpha = "<< str_h.alpha<<std::endl;
            str_h.v_tot=std::hypot(y(0),y(2));
            str_h.beta=std::atan2(y(1),str_h.v_tot);
            //std::cout<<"v_tot = "<< str_h.v_tot<<std::endl;
            str_h.gamma=y(7)-str_h.alpha;
            //std::cout<<"gamma = "<< str_h.gamma<<std::endl;
            str_h.delta_h_dot=str_h.v_tot*std::sin(str_h.gamma);
            //std::cout<<"h dot ="<< str_h.delta_h_dot<<std::endl;
            str_h.h+=dt*str_h.delta_h_dot;
            //std::cout<<"current H"<<str_h.h<<" "<< "Current h_dot" <<str_h.delta_h_dot<<std::endl;
        }

    private:
        double current_time;  // Store current time for logging
};
