#include "aircraft_data.hpp"
#include <cmath>

#ifndef USE_XLSX
static void fillM(struct Matrix *M, int m, int n, const float *v) {
    if (n == 1) *M = vector(m);
    else if (m == 1) *M = vec_row(n);
    else *M = matrix(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            M->data[i][j] = v[i * n + j];
}
#endif

#ifdef USE_XLSX
#include <Eigen/Dense>
template <int R, int C>
static void eigenToMat(struct Matrix *M, const Eigen::Matrix<float, R, C> &E) {
    *M = matrix(R, C);
    for (int i = 0; i < R; i++)
        for (int j = 0; j < C; j++)
            M->data[i][j] = E(i, j);
}
#endif

aircraft_data readAircraft() {
    aircraft_data d;
#ifdef USE_XLSX
    // ---- usual path: read spreadsheet, compute with Eigen, store into struct Matrix ----
    float V0[3] = {B[5], B[6], B[7]};
    float omega0[3] = {B[8], B[9], B[10]};
    float euler0[3] = {B[11], B[12], B[13]};
    float states0[9] = {B[5], B[6], B[7], B[8], B[9], B[10], B[11], B[12], B[13]};
    float Vtotal = sqrt(V0[0]*V0[0] + V0[1]*V0[1] + V0[2]*V0[2]);
    float m = B[52], g = B[53];
    float theta0 = B[12];
    float z0 = B[16];
    float mg0[3] = {m * g * -sin(theta0), 0, m * g * cos(theta0)};

    float Ixx = B[54], Iyy = B[55], Izz = B[56], Ixz = B[57];
    Eigen::Matrix<float,3,3> eInertia;
    eInertia << Ixx, 0, -Ixz,
                0, Iyy, 0,
                -Ixz, 0, Izz;

    float Xu=B[22],Zu=B[23],Mu=B[24],Xw=B[25],Zw=B[26],Mw=B[27],
           Zwd=B[28],Zq=B[29],Mwd=B[30],Mq=B[31],Xde=B[32],Zde=B[33],
           Mde=B[34],Xdth=B[35],Zdth=B[36],Mdth=B[37];

    float SD_Lat_dash[14];
    for (int i=38;i<52;i++) SD_Lat_dash[i-38]=B[i];

    float Yda=B[46],Ydr=B[47];
    float G = 1/(1 - (Ixz*Ixz)/(Ixx*Izz));
    float Yv=SD_Lat_dash[0],Yb=SD_Lat_dash[1],LB=SD_Lat_dash[2],NB=SD_Lat_dash[3],
           LP=SD_Lat_dash[4],NP=SD_Lat_dash[5],LR=SD_Lat_dash[6],NR=SD_Lat_dash[7],
           L_DA=SD_Lat_dash[10],N_DA=SD_Lat_dash[11],L_DR=SD_Lat_dash[12],N_DR=SD_Lat_dash[13];

    Eigen::Matrix<float,10,10> T;
    T << G, G*Ixz/Ixx, 0,0,0,0,0,0,0,0,
         G*Ixz/Izz, G, 0,0,0,0,0,0,0,0,
         0,0,G, G*Ixz/Ixx, 0,0,0,0,0,0,
         0,0,G*Ixz/Izz, G, 0,0,0,0,0,0,
         0,0,0,0,G, G*Ixz/Ixx, 0,0,0,0,
         0,0,0,0,G*Ixz/Izz, G, 0,0,0,0,
         0,0,0,0,0,0,G, G*Ixz/Ixx, 0,0,
         0,0,0,0,0,0,G*Ixz/Izz, G, 0,0,
         0,0,0,0,0,0,0,0,G, G*Ixz/Ixx,
         0,0,0,0,0,0,0,0,G*Ixz/Izz, G;

    Eigen::Matrix<float,10,1> elat_dash;
    elat_dash << LB, NB, LP, NP, LR, NR, L_DA, N_DA, L_DR, N_DR;
    Eigen::Matrix<float,10,1> eLat_dash = T.lu().solve(elat_dash);

    float Lv=eLat_dash[0]/Vtotal, Nv=eLat_dash[1]/Vtotal;

    Eigen::Matrix<float,11,1> eSD_Lat;
    eSD_Lat << Yv, eLat_dash[0], eLat_dash[1], eLat_dash[2], eLat_dash[3],
               eLat_dash[4], eLat_dash[5], L_DA, N_DA, L_DR, N_DR;

    Eigen::Matrix<float,6,7> eSD;
    eSD << Xu, 0, Xw, 0, 0, 0, 0,
           0, Yv, 0, 0, 0, 0, 0,
           Zu, 0, Zw, 0, Zq, 0, Zwd,
           0, Lv, 0, LP, 0, LR, 0,
           Mu, 0, Mw, 0, Mq, 0, Mwd,
           0, Nv, 0, NP, 0, NR, 0;

    Eigen::Matrix<float,6,4> eCD;
    eCD << 0, Xde, Xdth, 0,
           Yda, 0, 0, Ydr,
           0, Zde, Zdth, 0,
           L_DA, 0, 0, L_DR,
           0, Mde, Mdth, 0,
           N_DA, 0, 0, N_DR;

    Eigen::Matrix<float,3,1> eV0; eV0 << V0[0], V0[1], V0[2];
    Eigen::Matrix<float,3,1> eomega0; eomega0 << omega0[0], omega0[1], omega0[2];
    Eigen::Matrix<float,3,1> eeuler0; eeuler0 << euler0[0], euler0[1], euler0[2];
    Eigen::Matrix<float,9,1> estates0; estates0 << states0[0], states0[1], states0[2], states0[3], states0[4], states0[5], states0[6], states0[7], states0[8];
    Eigen::Matrix<float,3,1> emg0; emg0 << mg0[0], mg0[1], mg0[2];

    eigenToMat(&d.V0, eV0);
    eigenToMat(&d.omega0, eomega0);
    eigenToMat(&d.euler0, eeuler0);
    eigenToMat(&d.states0, estates0);
    eigenToMat(&d.mg0, emg0);
    eigenToMat(&d.Inertia, eInertia);
    eigenToMat(&d.SD, eSD);
    eigenToMat(&d.CD, eCD);

    d.Vtotal = Vtotal;
    d.m = m;
    d.g = g;
    d.theta0 = theta0;
    d.z0 = z0;
#else
    // ---- hardcoded path: matrix library only, no Eigen ----
    float V0[3] = {743.6104248f, 0, 45.48116195f};
    float omega0[3] = {0, 0, 0};
    float euler0[3] = {0, 0.06108652382f, 0};
    float states0[9] = {743.6104248f, 0, 45.48116195f, 0, 0, 0, 0, 0.06108652382f, 0};
    float mg0[3] = {-39950.10322f, 0, 653178.414f};
    float inertia[9] = {27800000.f, 0, -2460000.f,
                        0, 31800000.f, 0,
                        -2460000.f, 0, 56200000.f};
    float SD[42] = {
        -0.00379f, 0, 0.0304f, 0, 0, 0, 0,
        0, -0.0636f, 0, 0, 0, 0, 0,
        -0.0605f, 0, -0.427f, 0, 0, 0, 0,
        0, -0.001477814688f, 0, -0.706f, 0, 0.233f, 0,
        -0.000233f, 0, -0.00176f, 0, -0.506f, 0, -0.000182f,
        0, 0.0003815758676f, 0, -0.0776f, 0, -0.0991f, 0
    };
    float CD[24] = {
        0, 1, 4.91e-05f, 0,
        -5.22e-05f, 0, 0, 0.0091f,
        0, -16.4f, -1.72e-06f, 0,
        0.298f, 0, 0, 0.112f,
        0, -0.941f, 1.42e-07f, 0,
        0.00618f, 0, 0, -0.324f
    };

    fillM(&d.V0, 3, 1, V0);
    fillM(&d.omega0, 3, 1, omega0);
    fillM(&d.euler0, 3, 1, euler0);
    fillM(&d.states0, 9, 1, states0);
    fillM(&d.mg0, 3, 1, mg0);
    fillM(&d.Inertia, 3, 3, inertia);
    fillM(&d.SD, 6, 7, SD);
    fillM(&d.CD, 6, 4, CD);

    d.Vtotal = 745;
    d.m = 20339.3418;
    d.g = 32.17405;
    d.theta0 = 0.06108652382;
    d.z0 = -40000;
#endif
    return d;
}
