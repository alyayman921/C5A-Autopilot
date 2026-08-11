#include "serial.hpp"

mySerial::mySerial() {}

mySerial::mySerial(std::string serial_port_string) {
  try {
    usleep(1000);
    this->serial_port.Open(serial_port_string);
  } catch (const OpenFailed &) {
    std::cerr << "The serial port did not open correctly." << std::endl;
    return;
  }
  std::cerr << "Connected To Serial Port: " << serial_port_string << std::endl;
  this->serial_port.SetBaudRate(BaudRate::BAUD_115200);
  this->serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
  this->serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  this->serial_port.SetParity(Parity::PARITY_NONE);
  this->serial_port.SetStopBits(StopBits::STOP_BITS_1);
}

void mySerial::close() { mySerial::serial_port.Close(); }

void mySerial::empty_string() {
  for (int i = 0; i < Buffer_Size; i++) {
    string[i] = '0';
  }
}

char mySerial::read() {
  try {
    serial_port.ReadByte(data_byte, ms_timeout);
  }

  catch (const ReadTimeout &) {
  }
  return data_byte;
}

int mySerial::read_string(char (&string)[Buffer_Size]) {
  int i = 0;
  mySerial::empty_string();
  try {
    mySerial::serial_port.ReadByte(data_byte, ms_timeout);
    if (data_byte == start_char) {
      // std::cout<<"found char"<<std::endl;
      do {
        mySerial::serial_port.ReadByte(data_byte, ms_timeout);
        if (data_byte != terminating_char) {
          string[i] = data_byte;
          i++;
          // std::cout<<i<<std::endl;
        } else {
          // std::cout<<"Success"<<std::endl;
          return 1; // success, found $ and string finished
        }
      } while (i < Buffer_Size); // it was buffer+1, but might be buffer
                                 // overflow
      // check later
    }
    return 0;
  } catch (const ReadTimeout &) {
    return 0;
  }
}

char mySerial::write(char a) {
  try {
    mySerial::serial_port.WriteByte(a);
  } catch (const ReadTimeout &) {
  }
  return data_byte;
}

char mySerial::write_string(char *a) {
  try {
    mySerial::serial_port.WriteByte(start_char);
    mySerial::serial_port.Write(a);
    mySerial::serial_port.WriteByte(terminating_char);
  } catch (const ReadTimeout &) {
  }
  return data_byte;
}

std::string mySerial::getSerialPort(){
    // Test/debug hook: point the sim at an explicit serial device and skip
    // the scan (e.g. a socat pty simulating the MCU).
    if (const char* port = getenv("FSIM_SERIAL_PORT")) {
        if (port[0]) return std::string(port);
    }
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

