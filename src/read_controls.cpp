#include "read_controls.hpp"
bool readControlsFromFile(const std::string& filename,
                        struct Matrix* controls,
                        float& dt,
                        float& tfinal) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open controls file: " << filename << std::endl;
        return false;
    }
    float da, de, dth, dr;
    if (file >> da >> de >> dth >> dr >> dt >> tfinal) {
        controls->data[0][0] = da;
        controls->data[1][0] = de;
        controls->data[2][0] = dth;
        controls->data[3][0] = dr;
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
