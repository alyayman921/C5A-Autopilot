#ifndef MY_SERIAL_H
#define MY_SERIAL_H
#define size_int 1 // char boolean 1 or 0
#define n_ints 4   // for n bools
#define n_floats 20 // then the states
#define size_float 9 // in received chars
#define Buffer_Size 256 // actual received without start/term
#include <stdlib.h>
#include <stdint.h>

void empty_string();
int read_string();
void echo_string();
void write_string();
int write_ack();
void handle_ints();
void handle_floats();
#endif
