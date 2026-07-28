#ifndef MY_SERIAL_H
#define MY_SERIAL_H
#define size_int 1 // char boolean 1 or 0
#define n_ints 10  // 4 bools + 6 padding
#define n_floats 20 // then the states
#define size_float 9 // in received chars
#define CA_size 4 // Control actions
#define Buffer_Size 256 // actual received without start/term
#include <stdlib.h>
#include <stdint.h>

void empty_string();
int read_string();
void echo_string();
void write_control_actions(float da, float de, float dth, float dr);
int write_ack();
void handle_ints();
void handle_floats();
#endif
