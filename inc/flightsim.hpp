/*
Flight Simulator Main code definitions, the code Reads Actuators and solver dt and final time
from a text file, and reads the content of XSLX to extract the aircraft data, handle it and passes
to the solver a struct with the dynamics and the initial states, the solver solves Rigid body
dynamics equations for the nonlinear mode, or solves with the linear solver for the same conditions
the solvers, call the controllers after each step of solving
*/
#pragma once

#include <iostream>
#include <chrono>
#include "matrix.h"
#ifdef USE_XLSX
#include "fileHandling.h"
#endif
#include "aircraft_data.hpp"
#include "RBDEqns.hpp"
#include "controller.hpp"
#include "linear_sim.hpp"

float pi=3.1415926;
float deg2rad=pi/180.0;
float rad2deg=180.0/pi;

int step=0;
float dt = 0.01;
float tfinal = 100.0;
int progress_percent=0;

bool loop = false;
bool Autopiloted=true;
char mode='0';

autopilot_inputs commands;
flight_path str_h;

const char aircraft_file[] = "~/.local/share/3lymnFlightSimulator/meta/C5A.xlsx";
struct Matrix Controls;
int main(int argc, char* argv[]);
