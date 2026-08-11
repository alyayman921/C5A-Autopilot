#include "linear_sim.hpp"

fullLinear::fullLinear(Eigen::Matrix<double,4,1>* Controls,aircraft_data *ac ,flight_path *str_h ,
			autopilot_inputs *commands, Eigen::Matrix<double,9,1>* results){
      this->Controls=Controls;
      this->ac=ac;
      this->str_h=str_h;
      this->commands=commands;
      this->ptrResults=results;
      (*str_h).h=-(*ac).z0;
      yd<<0,0,0,0,0,0,0,0,0;
      lsh.x_long<<0,0,0,0;
      lsh.xd_long<<0,0,0,0;
      lsh.x_lat<<0,0,0,0,0;
      lsh.xd_lat<<0,0,0,0,0;

      A_Long<<-0.0038,  0.0304, -45.4812,  -32.1140,
              -0.0605, -0.4270, 743.6104,   -1.9642,
              -0.0002, -0.0017,  -0.6413,    0.0004,
                    0,       0,   1.0000,         0;

      B_Long<< 1.0000,   0.0000,
             -16.4000,  -0.0000,
              -0.9380,   0.0000,
                    0,        0;

      A_Lat<<  -0.0636,   45.4812, -743.6104,   32.1140,       0,
              -0.0014,   -0.7060,    0.2330,         0,        0,
               0.0003,   -0.0776,   -0.0991,         0,        0,
                    0,    1.0000,    0.0612,         0,        0,
                    0,         0,    1.0019,         0,        0;

      B_Lat<<    -0.0389,    6.7795,
                  0.2980,    0.1120,
                  0.0062,   -0.3240,
                       0,         0,
                       0,         0;
    }

    void fullLinear::solve(){
        Eigen::Matrix<double,9,1> delta_state=*ptrResults-ac->states0;
        lsh.x_long<<delta_state[0],delta_state[2],delta_state[4],delta_state[7];
        lsh.x_lat<<delta_state[1],delta_state[3],delta_state[5],delta_state[6],delta_state[8];
        select_lat[0]=(*Controls)[0];
        select_lat[1]=(*Controls)[3];
        lsh.xd_long=A_Long*lsh.x_long+B_Long* (*Controls).segment(1,2);
        lsh.xd_lat=A_Lat*lsh.x_lat+B_Lat* select_lat;
        yd<<lsh.xd_long[0],lsh.xd_lat[0],lsh.xd_long[1],
            lsh.xd_lat[1],lsh.xd_long[2],lsh.xd_lat[2],
            lsh.xd_lat[3],lsh.xd_long[3],lsh.xd_lat[4];
        *ptrResults=*ptrResults+yd*dt;
        fullLinear::h_calculation();
    }

    void fullLinear::h_calculation(){
        str_h->alpha=std::atan2((*ptrResults)(2),(*ptrResults)(0));
        str_h->v_tot=std::hypot((*ptrResults)(0),(*ptrResults)(2));
        str_h->beta=std::atan2((*ptrResults)(1),str_h->v_tot);
        str_h->gamma=(*ptrResults)(7)-str_h->alpha;
        str_h->delta_h_dot=str_h->v_tot*std::sin(str_h->gamma);
        str_h->h+=dt*str_h->delta_h_dot;
    }

