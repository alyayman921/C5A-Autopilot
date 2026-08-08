#pragma once
#include <stdint.h>
#include "arm_math.h"
extern float dt;
extern float* Controls;
extern uint8_t Autopiloted;
float select_lat[2]={0};
float yd[9]={0};
float delta_state[9]={0};

struct flight_path{
    float h, v_tot, delta_h_dot, alpha, beta, gamma;
};
struct{
  float x_long[4]={0};
  float x_lat[5]={0};
  float xd_long[4]={0};
  float xd_lat[5]={0};
}lsh;
float seg[2]={0,0};
float S0[9]={743.6104,0,45.4812,0,0,0,0,0.0612,0};
// JUST FYI I SWITCHED TO C++ BECAUSE THIS ASSIGNMENT DOESN'T WORK IN C HAHAHAHA
float A_Long[4][4]={
    {-0.0038 ,  0.0304,  -45.4812,  -32.1140},
    {0.0605,   -0.4270,  743.6104,   -1.9642},
    {-0.0002,   -0.0017,   -0.6413,    0.0004},
    {0,        0,   1.0000,         0}
  };
arm_matrix_instance_f32 ALo = {4, 4, &A_Long[0][0]};
float A_Lat[5][5]={
    {-0.0636,   45.4812, -743.6104,   32.1140,         0},
    {-0.0014,   -0.7060,    0.2330,         0,         0},
    {0.0003,   -0.0776,   -0.0991,         0,         0},
    {           0,    1.0000,    0.0612,         0,        0},
    {            0,         0,    1.0019,         0,         0}
  };
arm_matrix_instance_f32 ALa = {5, 5, &A_Lat[0][0]};
float B_Long[4][2]={
  {1.0000,   0.0000},
  {-16.4000,  -0.0000},
  {-0.9380,   0.0000},
  {0,0}
};
arm_matrix_instance_f32 BLo = {4, 2, &B_Long[0][0]};
float B_Lat[5][2]={
    {-0.0389,    6.7795},
    {0.2980,    0.1120},
    {0.0062,   -0.3240},
    {0,         0},
    {0,         0}
  };
arm_matrix_instance_f32 BLa = {5, 2, &B_Lat[0][0]};

#ifdef __cplusplus
extern "C" {
#endif
float solve_step(int step, float states[9]);
float* solve();
float atan2_2(float y, float x);
float hypot_2(float y, float x);
#ifdef __cplusplus
}
#endif
