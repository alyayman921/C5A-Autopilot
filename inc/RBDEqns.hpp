/*
This is a Rigid Body Dynamics Solver, which takes the Input of 9x1 State Vector {uvw,pqr,euler}
and solves one time step for it, start by calling the constructor and then using the method rk4_solver
*/
#pragma once
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "structs.h"
#include "matrix.h"
//float eps = 0.00000011921; // matlab float eps
const float eps = 0.000000000000000222044605;
extern float dt;
extern int step;

class RBDSolve{
    private:
        int i;
        float m, w_dot_state, g, cos_theta,v_tot,z0;
        struct Matrix v, v_dot;
        struct Matrix omega, delta_omega_dot;
        struct Matrix euler, euler_dot;
        struct Matrix J, I;
        struct Matrix SD;
        struct Matrix CD;
        struct Matrix* Controls;
        struct Matrix Aerodynamic_accel;
        struct Matrix delta_v, delta_omega, delta_F, delta_M;
        struct Matrix y,y_dot; struct Matrix delta_y;
        struct Matrix* states;
        aircraft_data* ac;
        flight_path *str_h;
        struct Matrix F_aero;
        struct Matrix F_grav;
        struct Matrix F_b;
        struct Matrix F_g0;
        struct Matrix M_total;
    public:
        RBDSolve(aircraft_data &ac, struct Matrix* Controls,flight_path *str_h ,struct Matrix* states);
        ~RBDSolve();
        void rk4Solver();
        void RBDEquations(struct Matrix y, struct Matrix* out);
};
