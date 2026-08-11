#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "structs.h"
extern double dt;

class fullLinear{
  private:
    struct{
      Eigen::Matrix<double,4,1> x_long;
      Eigen::Matrix<double,5,1> x_lat;
      Eigen::Matrix<double,4,1> xd_long;
      Eigen::Matrix<double,5,1> xd_lat;
    }lsh;
    Eigen::Matrix<double,9,1>* ptrResults;
    Eigen::Matrix<double,4,4> A_Long;
    Eigen::Matrix<double,5,5> A_Lat;
    Eigen::Matrix<double,4,2> B_Long;
    Eigen::Matrix<double,5,2> B_Lat;
    Eigen::Matrix<double,4,1>* Controls; aircraft_data *ac;
    flight_path *str_h;
		autopilot_inputs *commands ;bool Autopiloted;
    Eigen::Matrix<double,9,1>* state_history=nullptr;
    Eigen::Matrix<double,2,1> select_lat;
    Eigen::Matrix<double,9,1> yd;
  public:
    fullLinear(Eigen::Matrix<double,4,1>* Controls,aircraft_data *ac ,flight_path *str_h ,
			autopilot_inputs *commands, Eigen::Matrix<double,9,1>* results);
    void solve();
    void h_calculation();
};

