#pragma once
// THIS IS MADE IN NEOVIM, LEARNING THROUGH BLOOD, SWEAT AND TEARS
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "flightsim.hpp"

class fullLinear{
  private:
    struct linear_state_histroy{ // DELTA STATES
      Eigen::Matrix<double,4,1> x_long;
      Eigen::Matrix<double,5,1> x_lat;
      Eigen::Matrix<double,4,1> xd_long;
      Eigen::Matrix<double,5,1> xd_lat;
    };linear_state_histroy lsh;
    Eigen::Matrix<double,9,1> initial_state;
    Eigen::Matrix<double,4,4> A_Long;
    Eigen::Matrix<double,5,5> A_Lat;
    Eigen::Matrix<double,4,2> B_Long;
    Eigen::Matrix<double,5,2> B_Lat;
    Eigen::Matrix<double,4,1>* Controls; aircraft_data *ac;
    flight_path *str_h;
		double dt; int *step; autopilot_inputs *commands ;bool Autopiloted;
    Eigen::Matrix<double,9,1>* state_history=nullptr;
    controller *con_obj;
 public:
    fullLinear(Eigen::Matrix<double,4,1>* Controls,aircraft_data *ac ,flight_path *str_h ,
			double dt,int *step,autopilot_inputs *commands, Eigen::Matrix<double,9,1>initial_state, controller *con_obj,bool Autopiloted){
      this->Controls=Controls; this->ac=ac; this->str_h=str_h; this->dt=dt;this->con_obj=con_obj;
      this->step=step; this->commands=commands; this->Autopiloted=Autopiloted;this->initial_state=initial_state;
      A_Long<< -0.0038 ,  0.0304,  -45.4812,  -32.1140,
          -0.0605,   -0.4270,  743.6104,   -1.9642,
          -0.0002,   -0.0017,   -0.6413,    0.0004,
                0,        0,   1.0000,         0;

      B_Long<< 1.0000,   0.0000,
                -16.4000,  -0.0000,
                -0.9380,   0.0000,
                      0,        0;
      A_Lat<<  -0.0636,   45.4812, -743.6104,   32.1140,         0,
                -0.0014,   -0.7060,    0.2330,         0,         0,
                  0.0003,   -0.0776,   -0.0991,         0,         0,
                      0,    1.0000,    0.0612,         0,        0,
                      0,         0,    1.0019,         0,         0;

      B_Lat<<    -0.0389,    6.7795,
                    0.2980,    0.1120,
                    0.0062,   -0.3240,
                        0,         0,
                        0,         0;
      lsh.x_long<<0,0,0,0;
      lsh.xd_long<<0,0,0,0;
      lsh.x_lat<<0,0,0,0,0;
      lsh.xd_lat<<0,0,0,0,0;
    }
    void resultsPointer(){ // should be called reference but idc
        // send the adress of results to controller
        con_obj->linear_pointers(state_history); // shoiuld be named solver reference
    }

    Eigen::Matrix<double,9,1>* solve(int n_steps){
      state_history=new Eigen::Matrix<double,9,1>[n_steps];
      state_history[0]=initial_state;
      Eigen::Matrix<double,2,1> select_lat;
      Eigen::Matrix<double,9,1> yd;
      resultsPointer();
      for(int i=1;i<=n_steps;i++){
        select_lat[0]=(*Controls)[0];
        select_lat[1]=(*Controls)[3];
        lsh.xd_long=A_Long*lsh.x_long+B_Long* (*Controls).segment(1,2);
        lsh.xd_lat=A_Lat*lsh.x_lat+B_Lat* select_lat;
        yd<<lsh.xd_long[0],lsh.xd_lat[0],lsh.xd_long[1],
            lsh.xd_lat[1],lsh.xd_long[2],lsh.xd_lat[2],
            lsh.xd_lat[3],lsh.xd_long[3],lsh.xd_lat[4];
        state_history[i]=state_history[i-1]+dt*yd;
        if(Autopiloted){
          con_obj->pitch_controller();
          con_obj->velocity_controller();
          con_obj->altitude_controller();
          con_obj->roll_controller();
          con_obj->yaw_controller();
          con_obj->send_states();
        }
      }
      std::cout<<"Solving Finished\n";
      return state_history;
    }
    void free(){
      delete state_history;
    }
};

