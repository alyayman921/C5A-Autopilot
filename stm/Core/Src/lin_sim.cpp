#include "lin_sim.hpp"
#include <cmath>
extern struct flight_path str_h;//check if it needs to be a pointer or no
// extern float Controls[4];
void solve(int N_steps, float* states){
  for (int i=0;i<N_steps;i++){
    solve_step(states);
  }
}

void solve_step(float* states){
   // if(step==0){
   //   for(int i=0;i<9;i++){
   //       states[i]=S0[i];
   //   }
   //   return 1;
   // }else{
    for(int i=0;i<9;i++){
        delta_state[i]=yd[i]*0.01; // was *dt but checking something
    }
    // Verified, Don't look at them for too long
    lsh.x_long[0]=delta_state[0];lsh.x_long[1]=delta_state[2];
    lsh.x_long[2]=delta_state[4];lsh.x_long[3]=delta_state[7];
    lsh.x_lat[0]=delta_state[1];lsh.x_lat[1]=delta_state[3];lsh.x_lat[2]=delta_state[5];
    lsh.x_lat[3]=delta_state[6];lsh.x_lat[4]=delta_state[8];
    // xd_long = A_Long*x_long + B_Long*[Controls[1], Controls[2]]
    seg[0]=Controls[1];seg[1]=Controls[2];
    arm_matrix_instance_f32 x_long_m = {4, 1, lsh.x_long};
    arm_matrix_instance_f32 seg_long_m = {2, 1, seg};
    arm_matrix_instance_f32 xd_long_m = {4, 1, lsh.xd_long};
    arm_mat_mult_f32(&ALo, &x_long_m, &xd_long_m);
    float b_long_out[4];
    arm_matrix_instance_f32 b_long_m = {4, 1, b_long_out};
    arm_mat_mult_f32(&BLo, &seg_long_m, &b_long_m);
    for(int i=0;i<4;i++){
        lsh.xd_long[i]+=b_long_out[i];
    }
    // xd_lat = A_Lat*x_lat + B_Lat*[Controls[0], Controls[3]]
    seg[0]=Controls[0];seg[1]=Controls[3];
    arm_matrix_instance_f32 x_lat_m = {5, 1, lsh.x_lat};
    arm_matrix_instance_f32 seg_lat_m = {2, 1, seg};
    arm_matrix_instance_f32 xd_lat_m = {5, 1, lsh.xd_lat};
    arm_mat_mult_f32(&ALa, &x_lat_m, &xd_lat_m);
    float b_lat_out[5];
    arm_matrix_instance_f32 b_lat_m = {5, 1, b_lat_out};
    arm_mat_mult_f32(&BLa, &seg_lat_m, &b_lat_m);
    for(int i=0;i<5;i++){
        lsh.xd_lat[i]+=b_lat_out[i];
    }
    yd[0]=lsh.xd_long[0];yd[1]=lsh.xd_lat[0];yd[2]=lsh.xd_long[1];
    yd[3]=lsh.xd_lat[1];yd[4]=lsh.xd_long[2];yd[5]=lsh.xd_lat[2];
    yd[6]=lsh.xd_lat[3];yd[7]=lsh.xd_long[3];yd[8]=lsh.xd_lat[4];
    for(int i=0;i<9;i++){
        states[i]+=delta_state[i];
    }
    // }
  }
float atan2_2(float y, float x){
  // Thanks to that guy on stackoverflow
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
    return (q + (float &)uatan_2q)*1.57007963;
}
float hypot_2(float y, float x){
  float result;
  arm_sqrt_f32(x*x+y*y,&result);
  return result;
}
void h_calculation(float state[9]){
    str_h.alpha=atan2_2(state[2],state[0]);
    str_h.v_tot=hypot_2(state[0],state[2]);
    // str_h.beta=std::atan2(state[1],str_h.v_tot);
    str_h.gamma=state[7]-str_h.alpha;
    str_h.delta_h_dot=str_h.v_tot*std::sin(str_h.gamma);
    str_h.h+=dt*str_h.delta_h_dot;
}
