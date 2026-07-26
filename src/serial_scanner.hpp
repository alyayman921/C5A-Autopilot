// serial_port_selector.h
#ifndef SERIAL_PORT_SELECTOR_H
#define SERIAL_PORT_SELECTOR_H

#include <string>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

/**
 * @brief Checks for available serial ports on a POSIX system.
 *        If exactly one port is found, it is returned.
 *        If multiple ports are found, the user is prompted to pick one.
 *        If no ports are found, an empty string is returned.
 * @return std::string The selected serial port device path (e.g., "/dev/ttyUSB0").
 * @throws std::runtime_error if the directory cannot be opened.
 */
inline std::string getSerialPort()
{
    // List of common serial device prefixes
    const std::vector<std::string> prefixes = {
        //"/dev/ttyS",    // standard serial ports
        "/dev/ttyUSB",  // USB-to-serial adapters
        "/dev/ttyACM"   // USB CDC ACM devices (e.g., Arduino, STM32)
    };

    std::vector<std::string> foundPorts;

    // Iterate over each prefix and scan the directory
    for (const auto& prefix : prefixes) {
        // Determine the directory part (everything up to the last '/')
        size_t lastSlash = prefix.find_last_of('/');
        if (lastSlash == std::string::npos) continue; // should not happen
        std::string dirPath = prefix.substr(0, lastSlash);
        std::string base = prefix.substr(lastSlash + 1);

        DIR* dir = opendir(dirPath.c_str());
        if (!dir) {
            // Directory might not exist; skip gracefully
            continue;
        }

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            // Check if the name starts with the required prefix
            if (name.find(base) == 0) {
                // Build full device path
                std::string fullPath = dirPath + "/" + name;

                // Optional: verify it's a character device (S_ISCHR)
                struct stat st;
                if (stat(fullPath.c_str(), &st) == 0 && S_ISCHR(st.st_mode)) {
                    foundPorts.push_back(fullPath);
                }
            }
        }
        closedir(dir);
    }

    // Handle the results
    if (foundPorts.empty()) {
        std::cerr << "No serial ports found." << std::endl;
        return std::string();  // empty string indicates none
    }

    if (foundPorts.size() == 1) {
        return foundPorts[0];
    }

    // Multiple ports: ask the user to select one
    std::cout << "Multiple serial ports found:\n";
    for (size_t i = 0; i < foundPorts.size(); ++i) {
        std::cout << "  " << (i + 1) << ". " << foundPorts[i] << '\n';
    }

    int choice = 0;
    while (true) {
        std::cout << "Enter the number of the port to use (1-" << foundPorts.size() << "): ";
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cerr << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (choice >= 1 && choice <= static_cast<int>(foundPorts.size())) {
            return foundPorts[choice - 1];
        }

        std::cerr << "Invalid selection. Please try again.\n";
    }
}

#endif // SERIAL_PORT_SELECTOR_H