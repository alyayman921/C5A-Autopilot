#pragma once
#include <fstream>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// Function to read controls, timestep, and simulation time from a text file
// This was used earlier during testing, still used for getting dt and sim time

bool readControlsFromFile(const std::string& filename, Eigen::Matrix<double, 4, 1>& controls,
        double& dt, double& tfinal);
