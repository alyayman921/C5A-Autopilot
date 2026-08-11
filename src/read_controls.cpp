#include "read_controls.hpp"
bool readControlsFromFile(const std::string& filename,
                        Eigen::Matrix<double, 4, 1>& controls,
                        double& dt,
                        double& tfinal) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open controls file: " << filename << std::endl;
        return false;
    }
    double da, de, dth, dr;
    if (file >> da >> de >> dth >> dr >> dt >> tfinal) {
        controls << da, de, dth, dr;
        file.close();
        // Validate inputs
        if (dt <= 0) {
            std::cerr << "Error: Timestep must be positive. Using default dt=0.01" << std::endl;
            dt = 0.01;
        }
        if (tfinal <= 0) {
            std::cerr << "Error: Final time must be positive. Using default tfinal=100" << std::endl;
            tfinal = 100.0;
        }
        return true;
    } else {
        std::cerr << "Error: Invalid format in controls file. Expected 6 doubles:" << std::endl;
        std::cerr << "  da de dth dr dt tfinal" << std::endl;
        file.close();
        return false;
    }
}
