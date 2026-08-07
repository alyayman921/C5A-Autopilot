#include "lin_sim.h"
struct flight_path_linear str_h;
float solve_step(int step, float states[9]){
  // state_history=new Eigen::Matrix<double,9,1>[n_steps+1];
  // state_history[0]=S0;
  // resultsPointer();
   if(step==0){
     states=S0;
     return 1;
   }else{
    delta_state=yd*dt;
    lsh.x_long={delta_state[0],delta_state[2],delta_state[4],delta_state[7]};
    lsh.x_lat={delta_state[1],delta_state[3],delta_state[5],delta_state[6],delta_state[8]};
    select_lat[0]=(*Controls)[0];
    select_lat[1]=(*Controls)[3];
    // lsh.xd_long=A_Long*lsh.x_long+B_Long* (*Controls).segment(1,2);
    // lsh.xd_lat=A_Lat*lsh.x_lat+B_Lat* select_lat;
    yd={lsh.xd_long[0],lsh.xd_lat[0],lsh.xd_long[1],
        lsh.xd_lat[1],lsh.xd_long[2],lsh.xd_lat[2],
        lsh.xd_lat[3],lsh.xd_long[3],lsh.xd_lat[4]};
    *states+=delta_state;
    // h_calculation(state_history[step]);
    float x = arm_cos_f32(delta_state[0]);
    return x;
    }
  }
float normalized_atan2( float y, float x ){
    static const uint32_t sign_mask = 0x80000000;
    static const float b = 0.596227f;

    // Extract the sign bits
    uint32_t ux_s  = sign_mask & (uint32_t &)x;
    uint32_t uy_s  = sign_mask & (uint32_t &)y;

    // Determine the quadrant offset
    float q = (float)( ( ~ux_s & uy_s ) >> 29 | ux_s >> 30 );

    // Calculate the arctangent in the first quadrant
    float bxy_a = ::fabs( b * x * y );
    float num = bxy_a + y * y;
    float atan_1q =  num / ( x * x + bxy_a + num );

    // Translate it to the proper quadrant
    uint32_t uatan_2q = (ux_s ^ uy_s) | (uint32_t &)atan_1q;
    return q + (float &)uatan_2q;
}
void h_calculation(float state[9]){
    str_h->alpha=std::atan2(state(2),state(0));
    str_h->v_tot=std::hypot(state(0),state(2));
    // str_h->beta=std::atan2(state(1),str_h->v_tot);
    str_h->gamma=state(7)-str_h->alpha;
    str_h->delta_h_dot=str_h->v_tot*std::sin(str_h->gamma);
    str_h->h+=dt*str_h->delta_h_dot;
}
void free(){
  delete[] state_history;
}

