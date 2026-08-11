#ifndef STRUCTS
#define STRUCTS



typedef struct{
  double Inertia_temp[4];
  double Vtotal,theta0,z0,m,g;
  double Xu,Zu,Mu,Xw,Zw,Mw,Zwd,Zq,Mwd,Mq,Xde,Zde,Mde,Xdth,Zdth,Mdth;
  double G,Yb,Yv,LB ,NB, LP , NP, LR,L_DR , NR, L_DA,N_DA, N_DR,Yda,Ydr;
  double Lv, Nv;
  double SD_Long_temp[16];
  double SD_Lat_dash[14];
  Eigen::Matrix<double,3,1> V0;
  Eigen::Matrix<double,3,1> mg0;
  Eigen::VectorXf time_vector;
  Eigen::Matrix<double,3,1> omega0;
  Eigen::Matrix<double,3,1> euler0;
  Eigen::Matrix<double,3,3> Inertia;
  Eigen::Matrix<double,9,1> states0;
  Eigen::Matrix<double, 6, 7> SD;
  Eigen::Matrix<double, 6, 4> CD;
  Eigen::Matrix<double, 10, 10> T;
  Eigen::Matrix<double, 10, 1> lat_dash;
  Eigen::Matrix<double, 10, 1> Lat_dash;
  Eigen::Matrix<double, 14, 1> SD_Lat;
}aircraft_data;

struct flight_path{
    double h=0,v_tot=0,delta_h_dot=0,alpha=0,beta=0,gamma=0;
};


struct autopilot_inputs{
    bool alt_override=false; // overrides altitude loop straight to pitch control
    bool head_override=false; // overrides heading loop straight to roll control
    bool ext_controller=false; // controls from ext mcu
    bool onboard=false; // controls from ext mcu
    bool linear=false;
    double dt=0.01;double set_pitch=0;
    double set_vel=0;double set_alt=0;
    double set_heading=0;double set_roll=0;
};
#endif
