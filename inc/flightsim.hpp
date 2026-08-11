#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include "structs.h"
#include "read_controls.hpp"
#include "readxlsx.h"
#include "aircraft_data.hpp"
#include "RBDEqns.hpp"
#include "controller.hpp"
#include "linear_sim.hpp"

double pi=3.1415926;
double deg2rad=pi/180.0;
double rad2deg=180.0/pi;
double dt = 0.01;
double tfinal = 100.0;
bool loop = false;
bool Autopiloted=true;
bool logging=true;
char mode='0';
int step=0;
int progress_percent=0;
autopilot_inputs commands;
flight_path str_h;

const char filename[] = "meta/C5A.xlsx";
Eigen::Matrix<double, 4, 1> Controls;
int main(int argc, char* argv[]);
