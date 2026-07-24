#ifndef MY_SERIAL_H
#define MY_SERIAL_H
#include <stdint.h>
#define n_floats 4
#define size_float 5 // in received bytes
#define string_size n_floats*size_float

void set_serial_vars(uint8_t* byte_received,int inputs[n_floats]);
void empty_string();
int read_string();
void write_string();
int write_ack();
void handle_ints();
// void handle_floats(){
#endif
