#include "linear_sim.hpp"
#include <cmath>

static void mat_from_array(struct Matrix *M, int m, int n, const float *v) {
    *M = matrix(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            M->data[i][j] = v[i * n + j];
}

fullLinear::fullLinear(struct Matrix* Controls,aircraft_data *ac ,flight_path *str_h ,
			autopilot_inputs *commands, struct Matrix* results){
      this->Controls=Controls;
      this->ac=ac;
      this->str_h=str_h;
      this->commands=commands;
      this->ptrResults=results;
      (*str_h).h=-(*ac).z0;
      this->yd = vector(9); mat_set_zero(&this->yd);
      this->lsh.x_long = vector(4);
      this->lsh.xd_long = vector(4);
      this->lsh.x_lat = vector(5);
      this->lsh.xd_lat = vector(5);
      this->select_lat = vector(2);
      this->select_long = vector(2);

      static const float A_Long[16] = {
        -0.0038f,  0.0304f, -45.4812f,  -32.1140f,
        -0.0605f, -0.4270f, 743.6104f,   -1.9642f,
        -0.0002f, -0.0017f,  -0.6413f,    0.0004f,
              0.f,      0.f,   1.0000f,         0.f
      };
      static const float B_Long[8] = {
         1.0000f,   0.0000f,
        -16.4000f, -0.0000f,
         -0.9380f,  0.0000f,
              0.f,       0.f
      };
      static const float A_Lat[25] = {
        -0.0636f,   45.4812f, -743.6104f,   32.1140f,       0.f,
        -0.0014f,   -0.7060f,    0.2330f,         0.f,        0.f,
         0.0003f,   -0.0776f,   -0.0991f,         0.f,        0.f,
             0.f,    1.0000f,    0.0612f,         0.f,        0.f,
             0.f,         0.f,    1.0019f,         0.f,        0.f
      };
      static const float B_Lat[10] = {
        -0.0389f,    6.7795f,
         0.2980f,    0.1120f,
         0.0062f,   -0.3240f,
             0.f,         0.f,
             0.f,         0.f
      };

      mat_from_array(&this->A_Long, 4, 4, A_Long);
      mat_from_array(&this->B_Long, 4, 2, B_Long);
      mat_from_array(&this->A_Lat, 5, 5, A_Lat);
      mat_from_array(&this->B_Lat, 5, 2, B_Lat);
    }

fullLinear::~fullLinear(){
    mat_free_memory(&A_Long); mat_free_memory(&A_Lat);
    mat_free_memory(&B_Long); mat_free_memory(&B_Lat);
    mat_free_memory(&yd); mat_free_memory(&select_lat); mat_free_memory(&select_long);
    mat_free_memory(&lsh.x_long); mat_free_memory(&lsh.xd_long);
    mat_free_memory(&lsh.x_lat); mat_free_memory(&lsh.xd_lat);
}

    void fullLinear::solve(){
        struct Matrix delta_state = vector(9);
        mat_sub(&delta_state, ptrResults, &ac->states0);
        this->lsh.x_long.data[0][0]=delta_state.data[0][0];
        this->lsh.x_long.data[1][0]=delta_state.data[2][0];
        this->lsh.x_long.data[2][0]=delta_state.data[4][0];
        this->lsh.x_long.data[3][0]=delta_state.data[7][0];
        this->lsh.x_lat.data[0][0]=delta_state.data[1][0];
        this->lsh.x_lat.data[1][0]=delta_state.data[3][0];
        this->lsh.x_lat.data[2][0]=delta_state.data[5][0];
        this->lsh.x_lat.data[3][0]=delta_state.data[6][0];
        this->lsh.x_lat.data[4][0]=delta_state.data[8][0];
        this->select_lat.data[0][0]=(*Controls).data[0][0];   // da  (lateral)
        this->select_lat.data[1][0]=(*Controls).data[3][0];   // dr  (lateral)
        this->select_long.data[0][0]=(*Controls).data[1][0];  // de  (longitudinal)
        this->select_long.data[1][0]=(*Controls).data[2][0];  // dth (longitudinal)
        struct Matrix t1 = {0};
        mat_mult(A_Long, lsh.x_long, &t1);
        struct Matrix t2 = {0};
        mat_mult(B_Long, select_long, &t2);
        mat_add(&lsh.xd_long, &t1, &t2);
        struct Matrix t3 = {0};
        mat_mult(A_Lat, lsh.x_lat, &t3);
        struct Matrix t4 = {0};
        mat_mult(B_Lat, select_lat, &t4);
        mat_add(&lsh.xd_lat, &t3, &t4);
        yd.data[0][0]=lsh.xd_long.data[0][0];
        yd.data[1][0]=lsh.xd_lat.data[0][0];
        yd.data[2][0]=lsh.xd_long.data[1][0];
        yd.data[3][0]=lsh.xd_lat.data[1][0];
        yd.data[4][0]=lsh.xd_long.data[2][0];
        yd.data[5][0]=lsh.xd_lat.data[2][0];
        yd.data[6][0]=lsh.xd_lat.data[3][0];
        yd.data[7][0]=lsh.xd_long.data[3][0];
        yd.data[8][0]=lsh.xd_lat.data[4][0];
        struct Matrix yd_dt = vector(9);
        mat_scalar_mul(&yd_dt, &yd, (float)dt);
        struct Matrix newres = vector(9);
        mat_add(&newres, ptrResults, &yd_dt);
        mat_copy(ptrResults, &newres);
        mat_free_memory(&delta_state); mat_free_memory(&t1); mat_free_memory(&t2);
        mat_free_memory(&t3); mat_free_memory(&t4); mat_free_memory(&yd_dt); mat_free_memory(&newres);
        fullLinear::h_calculation();
    }

    void fullLinear::h_calculation(){
        str_h->alpha=std::atan2((*ptrResults).data[2][0],(*ptrResults).data[0][0]);
        str_h->v_tot=std::hypot((*ptrResults).data[0][0],(*ptrResults).data[2][0]);
        str_h->beta=std::atan2((*ptrResults).data[1][0],str_h->v_tot);
        str_h->gamma=(*ptrResults).data[7][0]-str_h->alpha;
        str_h->delta_h_dot=str_h->v_tot*std::sin(str_h->gamma);
        str_h->h+=dt*str_h->delta_h_dot;
    }
