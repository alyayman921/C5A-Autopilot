#pragma once
#ifdef USE_SERIAL
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cstdint>
#include <string>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <stdexcept>
#define Buffer_Size 256 // after throwing the start and end characters
using namespace LibSerial ;
// #define start_char '!'
// #define terminating_char '$'

class mySerial{
private:
    uint8_t data_byte;
    char string[Buffer_Size]={0};
		size_t ms_timeout = 1;
    SerialPort serial_port;
    char start_char='!';
    char terminating_char= '$';

public:
    mySerial();
    mySerial(std::string serial_port_string);
    void close();
    void empty_string();
    char read();
    int read_string(char (&string)[Buffer_Size]);
    char write(char a);
    char write_string(char* a);
    std::string getSerialPort();
};
#endif
