#pragma once
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cstdint>
using namespace LibSerial ;
char start_char='!';
char terminating_char='$';

class serial{
private:
    //constexpr const char* const SERIAL_PORT_1 = "/dev/ttyACM0" ;
    uint8_t data_byte; // Char variable to store data coming from the serial port.
    const static int size=20;
    char string[size]={0};
    size_t ms_timeout = 5;
    SerialPort serial_port;
public:
    serial(){}
    serial(char serial_port[]){
        try{
            usleep(1000) ;
            this->serial_port.Open(serial_port);
        }
        catch (const OpenFailed&)
        {
            std::cerr << "The serial port did not open correctly." << std::endl ;
        }
        // Set the baud rate of the serial port.
        this->serial_port.SetBaudRate(BaudRate::BAUD_115200) ;
        // Set the number of data bits.
        this->serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
        // Turn off hardware flow control.
        this->serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        // Disable parity.
        this->serial_port.SetParity(Parity::PARITY_NONE) ;
        // Set the number of stop bits.
        this->serial_port.SetStopBits(StopBits::STOP_BITS_1) ;
    }
    void close(){
        serial_port.Close();
    }
    void empty_string(){
        for (int i=0;i<size;i++){
            string[i]='0';
        }
    }
    char read(){
        try{
            serial_port.ReadByte(data_byte, ms_timeout) ;
        }

        catch (const ReadTimeout&)
        {
        }
        return data_byte;
    }
    int read_string(char (&string)[size]){
        int i=0;
        empty_string();
        try{
            serial_port.ReadByte(data_byte, ms_timeout);
                if (data_byte==start_char){
                    // std::cout<<"found char"<<std::endl;
                    do{
                        serial_port.ReadByte(data_byte, ms_timeout);
                        if(data_byte!=terminating_char){
                            string[i]=data_byte;
                            i++;
                            // std::cout<<i<<std::endl;
                        }else{
                            // std::cout<<"Success"<<std::endl;
                            return 1; // success, found $ and string finished
                        }
                        }while(i<size);
                    }
            return 0;
        }
    catch (const ReadTimeout&)
    {
        std::cout<<"Timeout Receiving string\n";
        return 0;
    }
}
    char write(char a){
        try{
            serial_port.WriteByte(a);
        }
        catch (const ReadTimeout&)
        {
        }
        return data_byte;
    }
    char write_string(char* a){
        try{
            serial_port.WriteByte(start_char) ;
            serial_port.Write(a);
            serial_port.WriteByte(terminating_char) ;
        }
        catch (const ReadTimeout&){
        }
        return data_byte;
    }
};
