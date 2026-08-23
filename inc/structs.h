#ifndef STRUCTS
#define STRUCTS

#include "matrix.h"



typedef struct{
  float Inertia_temp[4];
  float Vtotal,theta0,z0,m,g;
  float Xu,Zu,Mu,Xw,Zw,Mw,Zwd,Zq,Mwd,Mq,Xde,Zde,Mde,Xdth,Zdth,Mdth;
  float G,Yb,Yv,LB ,NB, LP , NP, LR,L_DR , NR, L_DA,N_DA, N_DR,Yda,Ydr;
  float Lv, Nv;
  float SD_Long_temp[16];
  float SD_Lat_dash[14];
  struct Matrix V0;
  struct Matrix mg0;
  struct Matrix omega0;
  struct Matrix euler0;
  struct Matrix Inertia;
  struct Matrix states0;
  struct Matrix SD;
  struct Matrix CD;
  struct Matrix T;
  struct Matrix lat_dash;
  struct Matrix Lat_dash;
  struct Matrix SD_Lat;
}aircraft_data;

struct flight_path{
    float h=0,v_tot=0,delta_h_dot=0,alpha=0,beta=0,gamma=0;
};


struct autopilot_inputs{
    bool alt_override=false; // overrides altitude loop straight to pitch control
    bool head_override=false; // overrides heading loop straight to roll control
    bool ext_controller=false; // controls from ext mcu
    bool onboard=false; // controls from ext mcu
    bool linear=false;
    float dt=0.01;float set_pitch=0;
    float set_vel=0;float set_alt=0;
    float set_heading=0;float set_roll=0;
};
#endif
