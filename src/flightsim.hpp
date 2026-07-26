#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
double pi=3.1415926;
double deg2rad=pi/180.0;
double rad2deg=180.0/pi;

#include "read_controls.hpp"

struct autopilot_inputs{
    bool alt_override=false; // overrides altitude loop straight to pitch control
    bool head_override=false; // overrides heading loop straight to roll control
    bool ext_controller=false; // controls from ext mcu
    double dt=0.01;double set_pitch=0;
    double set_vel=0;double set_alt=0;
    double set_heading=0;double set_roll=0;
};
autopilot_inputs commands;

struct flight_path{
    // make the h value the one gotten from xlsx, 
    double h=40000,v_tot,delta_h_dot,alpha,beta,gamma; 
    //also this is not the place to define it
};
flight_path str_h;
int step=0;
int* ptrStep=&step;
#include "readxslx.h"
#include "derivatives.hpp"
#include "RBDEqns.hpp"
#include "rk4.hpp"
#include "tf.hpp"
#include "controller.hpp"
// Variables to be read from controls file
double tfinal;
bool loop;
const char filename[] = "meta/C5A.xlsx";
Eigen::Matrix<double, 4, 1> Controls;
bool Autopiloted=true;
bool logging=true;

int main(int argc, char* argv[]);