#pragma once

const float deg2rad=3.1415f/180.0f;
const float rad2deg=180.0f/3.1415f;
float da,de,dth,dr;
int step;

struct autopilot_inputs{
    int alt_override;
    int head_override;
    int ext_controller;
    float set_pitch;
    float set_vel;
    float set_alt;
    float set_heading;
    float set_roll;
};

struct flight_path{
    float h, v_tot, delta_h_dot, alpha, beta, gamma;
};

struct tf_leadlag{
    float y_prev;
    float r_prev;
};

struct tf_lag{
    float y_prev;
};

float y_pitch;
float de_max=  25.0f * deg2rad;
float de_min= -25.0f * deg2rad;
struct tf_leadlag pitch_tf_state;
struct tf_lag pitch_servo_state;
float pitch_initial;
int pitch_initialized;

float y_vel;
float dth_max= 167509.0f;
float dth_min= -40800.0f;
struct tf_leadlag vel_tf_state;
struct tf_lag throttle_valve_state;
struct tf_lag engine_lag_state;

float y_alt;
struct tf_leadlag alt_tf_state;

float da_max=  25.0f * deg2rad;
float da_min= -25.0f * deg2rad;
struct tf_leadlag pi_roll_state;
struct tf_leadlag pd_roll_state;
struct tf_lag roll_servo_state;

float coordinated_roll;
struct tf_leadlag yaw_damper_state;
struct tf_lag yaw_servo_state;

float solve_leadlag(struct tf_leadlag *s, float a0, float a1, float b1, float r){
    float y = b1 * s->y_prev + a1 * r + a0 * s->r_prev;
    s->r_prev = r;
    s->y_prev = y;
    return y;
}

float solve_lag(struct tf_lag *s, float a, float b, float r){
    float y = a * s->y_prev + b * r;
    s->y_prev = y;
    return y;
}

void pitch_controller(float *results, struct autopilot_inputs *cmd){
    if (cmd->ext_controller){
        if (!pitch_initialized) { pitch_initial = results[7]; pitch_initialized = 1; }
        float delta_theta = results[7] - pitch_initial;
        y_pitch = solve_leadlag(&pitch_tf_state, 0.009974f, 0.009974f, 1.0f,
                                cmd->set_pitch - delta_theta);
        de = -(y_pitch - ((delta_theta * 1.734f + results[4]) * 1.5236f));
        // de = solve_lag(&pitch_servo_state, 0.9f, 0.1f, de);
        if (de > de_max) de = de_max;
        if (de < de_min) de = de_min;
    }
}

void velocity_controller(float *results, struct autopilot_inputs *cmd){
    if (cmd->ext_controller){
        y_vel = solve_leadlag(&vel_tf_state, 0.0f, 1.252f, 1.0f,
                              cmd->set_vel - results[0]);
        dth = solve_lag(&throttle_valve_state, 0.999f, 0.001f, y_vel);
        dth = solve_lag(&engine_lag_state, 0.999f, 0.001f, dth);
        if (dth > dth_max) dth = dth_max;
        if (dth < dth_min) dth = dth_min;
    }
}

void alt_controller(struct autopilot_inputs *cmd, struct flight_path *fp){
    if (!cmd->alt_override && cmd->ext_controller){
        y_alt = solve_leadlag(&alt_tf_state, -0.010928f, 0.010971f, 0.9048f,
                              cmd->set_alt - fp->h);
        cmd->set_pitch = y_alt;
    }
}

void roll_controller(float *results, struct autopilot_inputs *cmd){
    if (cmd->ext_controller){
        float pi_out = solve_leadlag(&pi_roll_state, -0.13398f, 0.16047f, 1.0f,
                                     cmd->set_roll - results[6]);
        float pd_out = solve_leadlag(&pd_roll_state, -111.97f, 113.43f, 0.8605f,
                                     results[6]);
        da = pi_out - pd_out;
        da = solve_lag(&roll_servo_state, 0.9f, 0.1f, da);
        if (da > da_max) da = da_max;
        if (da < da_min) da = da_min;
    }
}

void yaw_controller(float *results, struct autopilot_inputs *cmd){
    if (!cmd->head_override && cmd->ext_controller){
        coordinated_roll = (cmd->set_heading - results[8]) * results[0] / 9.81f / 10.0f;
        if (coordinated_roll > 25.0f) coordinated_roll = 25.0f;
        if (coordinated_roll < -25.0f) coordinated_roll = -25.0f;
        cmd->set_roll = coordinated_roll;
        dr = solve_leadlag(&yaw_damper_state, -0.88667f, 0.88667f, 0.999f,
                           results[5]);
        dr = solve_lag(&yaw_servo_state, 0.9f, 0.1f, dr);
    }
}

/*
LongPitchController_PI_Ts10_z =
 
  0.009974 (1+z^-1)
  -----------------
      (1-z^-1)
      
LongVelocityController_PI_Ts67_z =
 
  1549.2 (1-z^-1)
  ---------------
     (1-z^-1)


LongAltitude_Lead_Ts28_z =
 
  0.010971 (1-0.9963z^-1)
  -----------------------
      (1-0.9048z^-1)

PI_Roll_z =
 
  0.16047 (1-0.8349z^-1)
  ----------------------
         (1-z^-1)

PD_Roll_z =
 
  113.43 (1-0.9871z^-1)
  ---------------------
     (1-0.8605z^-1)

HPF_YawDamper_z =
 
  0.88667 (1-z^-1)
  ----------------
   (1-0.999z^-1)

G_Servo_z =
 
  0.047619 (1+z^-1)
  -----------------
   (1-0.9048z^-1)

G_EngineTimeLag_z =
 
  0.00049975 (1+z^-1)
  -------------------
     (1-0.999z^-1)


G_Servo_EngineTimeLag_tf =
 
  2.38e-05 + 4.76e-05 z^-1 + 2.38e-05 z^-2
  ----------------------------------------
        1 - 1.904 z^-1 + 0.9039 z^-2
 
*/