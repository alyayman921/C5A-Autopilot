#ifndef MY_SERIAL_H
#define MY_SERIAL_H
#define n_floats 1
#define n_ints 1
#define size_float 4 // in received bytes
#define size_int 4 // in received bytes
#define string_size n_floats*size_float
#include <stdlib.h>
#include <stdint.h>

// void set_serial_vars(uint8_t* byte_received,int inputs[n_floats]);
void empty_string();
int read_string();
void write_string();
int write_ack();
void handle_ints();
void handle_floats();
#endif
