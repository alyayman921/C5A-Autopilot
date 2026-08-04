#pragma once
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cstdint>
#define Buffer_Size 256 // after throwing the start and end characters
using namespace LibSerial ;
char start_char='!';
char terminating_char='$';

class mySerial{
private:
    uint8_t data_byte;
    char string[Buffer_Size]={0};
		size_t ms_timeout = 1;
    SerialPort serial_port;

public:
    mySerial(){}
    mySerial(std::string serial_port_string){
        try{
            usleep(1000) ;
            this->serial_port.Open(serial_port_string);
        }
        catch (const OpenFailed&)
        {
            std::cerr << "The serial port did not open correctly." << std::endl ;
        }
        std::cerr << "Connected To Serial Port: " <<serial_port_string<< std::endl ;
        this->serial_port.SetBaudRate(BaudRate::BAUD_115200) ;
        this->serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
        this->serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        this->serial_port.SetParity(Parity::PARITY_NONE) ;
        this->serial_port.SetStopBits(StopBits::STOP_BITS_1) ;
    }
    void close(){
        serial_port.Close();
    }
    void empty_string(){
        for (int i=0;i<Buffer_Size;i++){
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
    int read_string(char (&string)[Buffer_Size]){
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
                        }while(i<Buffer_Size); //it was buffer+1, but might be buffer overflow
                        // check later
                    }
            return 0;
        }
    catch (const ReadTimeout&)
    {
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
