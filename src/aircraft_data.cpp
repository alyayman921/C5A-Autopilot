#include "aircraft_data.hpp"

aircraft_data readAircraft(){
  aircraft_data d;
  // States Node
  for(int i=5;i<8;i++){
    d.V0[i-5]=B[i];
    d.states0[i-5]=B[i];

  }
  d.Vtotal=sqrt(pow(d.V0[0],2)+pow(d.V0[1],2)+pow(d.V0[2],2));
  d.theta0=B[12]; // rad
  d.m=B[52];d.g=B[53];
  d.mg0 << -sin(d.theta0) , 0 , cos(d.theta0);
  d.mg0=d.m * d.g * d.mg0;
  d.z0=B[16];
  for (int i=8;i<11;i++){
    d.omega0[i-8]=B[i];
    d.states0[i-8+3]=B[i];
  }
  for (int i=11;i<14;i++){
    d.euler0[i-11]=B[i];
    d.states0[i-11+6]=B[i];
  }
  for(int i=54;i<58;i++){   // ixx iyy izz ixz
  d.Inertia_temp[i-54] = B[i];
  }

  d.Inertia<< d.Inertia_temp[0],      0,            -d.Inertia_temp[3],
                    0,          d.Inertia_temp[1],        0,
              -d.Inertia_temp[3],      0 ,            d.Inertia_temp[2];

  // Long Stability Derivatives
  d.Xu=B[22];d.Zu=B[23];d.Mu=B[24];
  d.Xw=B[25];d.Zw=B[26];d.Mw=B[27];
  d.Zwd=B[28];d.Zq=B[29];d.Mwd=B[30];
  d.Mq=B[31];d.Xde=B[32];d.Zde=B[33];
  d.Mde=B[34];d.Xdth=B[35];d.Zdth=B[36];d.Mdth=B[37];

  // Lat Stability Derivatives
  for(int i=38;i<52;i++){
    d.SD_Lat_dash[i-38] = B[i];
    //std::cout << B[i];
  }

  d.Yda=B[46];d.Ydr=B[47];
  d.G=1/(1-(pow(d.Inertia_temp[3],2)/(d.Inertia_temp[0]*d.Inertia_temp[2])));
  d.Yv = d.SD_Lat_dash[0];d.Yb = d.SD_Lat_dash[1];d.LB = d.SD_Lat_dash[2];
  d.NB = d.SD_Lat_dash[3];d.LP = d.SD_Lat_dash[4];d.NP = d.SD_Lat_dash[5];
  d.LR = d.SD_Lat_dash[6];d.NR = d.SD_Lat_dash[7];d.L_DA = d.SD_Lat_dash[10];
  d.N_DA = d.SD_Lat_dash[11];d.L_DR = d.SD_Lat_dash[12];d.N_DR = d.SD_Lat_dash[13];
  d.lat_dash << d.LB, d.NB, d.LP, d.NP, d.LR, d.NR, d.L_DA, d.N_DA, d.L_DR, d.N_DR;

  d.T << d.G,d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0, 0, 0, 0, 0,
       d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G,0, 0, 0, 0, 0, 0, 0, 0,
       0,  0,d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0, 0, 0,
       0,  0,d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0, 0, 0, 0, 0,
       0,  0,0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0,
       0,  0,0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0, 0, 0,
       0,  0,0, 0, 0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0,
       0,  0,0, 0, 0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0,
       0,  0,0, 0, 0, 0, 0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0],
       0,  0,0, 0, 0, 0, 0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G;
  // lat temp = T dash-1
  d.Lat_dash=d.T.lu().solve(d.lat_dash);
  d.SD_Lat << d.Yv,
              d.Yb, // 1
              d.Lat_dash[0],//LB_dash 2
              d.Lat_dash[1],//NB_dash 3
              d.Lat_dash[2],//LP_dash 4
              d.Lat_dash[3],//NP_dash 5
              d.Lat_dash[4],//LR_dash 6
              d.Lat_dash[5],//NR_dash 7
              d.Yda,
              d.Ydr,
              d.Lat_dash[6],//L_DA_dash 10
              d.Lat_dash[7],//N_DA_dash 11
              d.Lat_dash[8],//L_DR_dash 12
              d.Lat_dash[9];//N_DR_dash 13

  d.Lv=d.SD_Lat[2]/d.Vtotal;d.Nv=d.SD_Lat[3]/d.Vtotal;
  d.Yv = d.SD_Lat[0];d.Yb = d.SD_Lat[1];d.LB = d.SD_Lat[2];
  d.NB = d.SD_Lat[3];d.LP = d.SD_Lat[4];d.NP = d.SD_Lat[5];
  d.LR = d.SD_Lat[6];d.NR = d.SD_Lat[7];d.L_DA = d.SD_Lat[10];
  d.N_DA = d.SD_Lat[11];d.L_DR = d.SD_Lat[12];d.N_DR = d.SD_Lat[13];

  // SD Matrix Construction
  d.SD << d.Xu, 0,    d.Xw, 0,    0, 0, 0,
        0,    d.Yv, 0,    0,    0, 0, 0,
        d.Zu, 0,    d.Zw, 0,    d.Zq, 0, d.Zwd,
        0,    d.Lv, 0,    d.LP, 0, d.LR, 0,
        d.Mu, 0,    d.Mw, 0,    d.Mq, 0, d.Mwd,
        0,    d.Nv, 0,    d.NP, 0, d.NR, 0;
  d.CD << 0, d.Xde, d.Xdth, 0,
          d.Yda, 0, 0 , d.Ydr,
          0, d.Zde, d.Zdth,0,
          d.L_DA, 0,0,d.L_DR,
          0,d.Mde ,d.Mdth , 0,
          d.N_DA, 0, 0, d.N_DR;
  //std::cout<<d.SD<<std::endl;std::cout<<d.CD<<std::endl;
return d;
}
