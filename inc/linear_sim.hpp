/*
Linear Simulator, Not much to say, solves one step by multiplying the State space linearized
matrices, has faulty B_long matrix so no control in the throttle
*/
#pragma once

#include <iostream>
#include "structs.h"
#include "matrix.h"

extern float dt;

class fullLinear{
  private:
    struct{
      struct Matrix x_long;
      struct Matrix x_lat;
      struct Matrix xd_long;
      struct Matrix xd_lat;
    }lsh;
    struct Matrix* ptrResults;
    struct Matrix A_Long;
    struct Matrix A_Lat;
    struct Matrix B_Long;
    struct Matrix B_Lat;
    struct Matrix* Controls; aircraft_data *ac;
    flight_path *str_h;
		autopilot_inputs *commands ;bool Autopiloted;
    struct Matrix* state_history;
    struct Matrix select_lat;
    struct Matrix yd;
  public:
    fullLinear(struct Matrix* Controls,aircraft_data *ac ,flight_path *str_h ,
			autopilot_inputs *commands, struct Matrix* results);
    ~fullLinear();
    void solve();
    void h_calculation();
};
