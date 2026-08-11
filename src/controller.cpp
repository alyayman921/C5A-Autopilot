#include "controller.hpp"

extern double tfinal;


controller::controller(Eigen::Matrix<double,4,1>* ptrControls, Eigen::Matrix<double,9,1>* ptrResults
    ,aircraft_data *ac, flight_path *ptrstr_h,autopilot_inputs *ptrCommands){
  this->ptrControls=ptrControls;
  this->ptrstr_h=ptrstr_h;
  this->ptrResults=ptrResults;
  this->ac=ac;
  this->ptrCommands=ptrCommands;

#ifdef USE_SERIAL

  if(ptrCommands->ext_controller){
    SP= new mySerial(); // to get the string first
    std::string sp_string=SP->getSerialPort();
    delete SP;
    SP=new mySerial(sp_string);
    std::cout<<"Connected to Port\n";
  }
#endif

  // Pitch
  this->pitch_tf=transferFunction(1 , 2 , pitch_num , pitch_den );
  this->pitch_servo=transferFunction(1 , 2, servo_num, servo_den );

  // velocity
  this->vel_tf=transferFunction(2 , 2, vel_num, vel_den );
  this->throttle_valve=transferFunction(1 , 2, 10.0, throttle_den );
  this->engine_lag=transferFunction(1 , 2, engine_num, engine_den );

  // altitude
  this->alt_tf=transferFunction(2 , 2, alt_num, alt_den );

  // roll
  this->PI_Roll=transferFunction(2 , 2, pir_num, pir_den);
  this->PD_Roll=transferFunction(2 , 2, pdr_num, pdr_den);
  this->roll_servo=transferFunction(1 , 2, servo_num, servo_den);

  // yaw
  this->yaw_damper=transferFunction(2 , 2, yaw_num, yaw_den);
  this->yaw_servo=transferFunction(1 , 2, servo_num, servo_den);

  *ptrControls={0,0,0,0};
}

controller::~controller(){
#ifdef USE_SERIAL
if (SP) {
  SP->close();
  delete SP;
}
#endif
}

void controller::updateControllers(){
  if(Autopiloted&&!ptrCommands->ext_controller){
    if (!ptrCommands->alt_override) controller::altitude_controller();
    controller::pitch_controller();
    controller::velocity_controller();
    if (!ptrCommands->head_override) controller::yaw_controller();
    controller::roll_controller();
  }else if(Autopiloted&&ptrCommands->ext_controller){
    // STM32 Controller
    controller::send_states();
  }
}

/*------------------------- Controllers ----------------------------------------*/
void controller::pitch_controller(){
    delta_theta=(*ptrResults)(7)-ac->states0(7);
    y_pitch=pitch_tf.solve(((ptrCommands->set_pitch)-delta_theta));
    de=-(y_pitch - ((delta_theta*1.734+(*ptrResults)(4))*1.5236));
    de=pitch_servo.solve(de);
    if (de>de_max){de=de_max;}
    if (de<de_min){de=de_min;}
    *ptrControls={da,de,dth,dr};
}

void controller::altitude_controller(){
  /*
      0.011498 (s+0.3709)
      -------------------
            (s+10)
  */
      y_alt=alt_tf.solve(((ptrCommands->set_alt)-ptrstr_h->h));
      ptrCommands->set_pitch=y_alt;
}

void controller::velocity_controller(){
  /*
      1549.1 (s+0.008082)
      -------------------
              s
  */
    y_vel=vel_tf.solve(((ptrCommands->set_vel)-(*ptrResults)(0)));
    dth=throttle_valve.solve(y_vel);
    dth=engine_lag.solve(dth);
    if (dth>dth_max){dth=dth_max; vel_tf.clamp_state(dth_min, dth_max);}
    if (dth<dth_min){dth=dth_min; vel_tf.clamp_state(dth_min, dth_max);}
    *ptrControls={da,de,dth,dr};
}
// -------------------------------------- LATERAL CONTROLLERS ---------------------------------------

void controller::roll_controller(){
  /*
       PI_Roll                  PD_Roll
    0.14722 (s+18)          121.15 (s+1.3)
    --------------          --------------
          s                     (s+15)
  */
    da=PI_Roll.solve(((ptrCommands->set_roll)-(*ptrResults)(6)))-PD_Roll.solve((*ptrResults)(6));
    da=roll_servo.solve(da);
    if (da>da_max){da=da_max;}
    if (da<da_min){da=da_min;}
    *ptrControls={da,de,dth,dr};
}
void controller::yaw_controller(){
  /*
      damper
    0.88711 s
    ---------
      (s+0.1)
  */
    coordinated_roll=((ptrCommands->set_heading)-(*ptrResults)(8))*(ac->states0[0])/ac->g/10;
  if (coordinated_roll>25*deg2rad){coordinated_roll=25*deg2rad;}
  if (coordinated_roll<-25*deg2rad){coordinated_roll=-25*deg2rad;}
    ptrCommands->set_roll=coordinated_roll;
    //roll_controller();
    dr=yaw_damper.solve((*ptrResults)(5));
    dr=yaw_servo.solve(dr);
    if (dr>dr_max){dr=dr_max;}
    if (dr<dr_min){dr=dr_min;}
    *ptrControls={da,de,dth,dr};

}

/* --------------- Conv Floats to 9 char Send To Serial -----------------------*/
void controller::pack_serial_data(char buffer[256]) {

  char *ptr = buffer;

  // ---- First 10 chars: booleans as '0' or '1' ----
  *ptr++ = Autopiloted ? '1' : '0';
  *ptr++ = ptrCommands->alt_override ? '1' : '0';
  *ptr++ = ptrCommands->head_override ? '1' : '0';
  *ptr++ = ptrCommands->ext_controller ? '1' : '0';
  *ptr++ = ptrCommands->onboard ? '1' : '0';
  // pad remaining 5 positions with '0'
  for (int i = 0; i < 5; ++i) *ptr++ = '0';

  // ---- Helper lambda for floats: exactly 9 chars with 2 decimals ----
  // Format: S DDDDD . DD (sign + 5 digits + dot + 2 decimals = 9 chars)
  // Range: ±99999.99
  auto append_float = [&](double val) {
      // Clamp to ±99999.99
      if (val > 99999.99) val = 99999.99;
      if (val < -99999.99) val = -99999.99;

      char buf[15];
      snprintf(buf, sizeof(buf), "%+09.2f", val);

      // buf is now exactly 9 chars: "+0000.00" or "-99999.99"
      for (int i = 0; i < 9; i++) {
          *ptr++ = buf[i];
      }
  };

  // ---- 5 setpoints (5 × 9 = 45 chars) ----
  append_float(ptrCommands->set_pitch);
  append_float(ptrCommands->set_vel);
  append_float(ptrCommands->set_alt);
  append_float(ptrCommands->set_heading);
  append_float(ptrCommands->set_roll);

  // ---- 9 ptrResults from state vector (9 × 9 = 81 chars) ----
  for (int i = 0; i < 9; ++i) {
      append_float((*ptrResults)(i));
  }

  // ---- 6 flight‑path variables (6 × 9 = 54 chars) ----
  append_float(ptrstr_h->h);
  // In onboard mode the STM runs the whole solve itself, so the number of
  // integration steps must match the PC's linear sim. Stash it in the
  // otherwise-unused v_tot slot (the STM recomputes v_tot each step).
  append_float(ptrCommands->onboard ? (double)((int)(tfinal / dt)) : ptrstr_h->v_tot);
  append_float(ptrstr_h->delta_h_dot);
  append_float(ptrstr_h->alpha);
  append_float(ptrstr_h->beta);
  append_float(ptrstr_h->gamma);

  // ---- Fill the rest of the 256‑byte buffer with zeros ----
  size_t used = ptr - buffer;
  if (used < 256) {
      memset(ptr, 0, 256 - used);
  }
}

void controller::send_states(){
#ifdef USE_SERIAL
  if (SP->read_string(received_CA)) {
    *ptrControls = {std::atof(received_CA),
                  std::atof(received_CA + 9),
                  std::atof(received_CA + 18),
                  std::atof(received_CA + 27)};
  }
  pack_serial_data(serial_states);
  SP->write_string(serial_states);
#endif
}

/* ---------------  THIS SHOULD BE A SEPERATE FILE-----------------------*/
void controller::onboard_sim(){
#ifdef USE_SERIAL
  if(ptrCommands->ext_controller && ptrCommands->onboard){
  step = 0; // << Check this out later idk wwhat it does
  // The STM runs the solve with the flight-path altitude as its initial h,
  // same as the linear sim does on the PC (-z0). Without this the packet
  // carries h=0 and the altitude controller drives the wrong way.
  ptrstr_h->h = -ac->z0;

  // A result packet always starts with '+' or '-'; an ACK is empty.
  auto is_result = [&]{ return serial_states[0]=='+' || serial_states[0]=='-'; };

  // Phase 1: send the onboard packet and wait for any reply (ACK or the
  // result itself). A single packet can be lost after a previous run, so
  // resend until the MCU acknowledges (up to ~3 s).
  bool ok = false;
  bool replied = false;
  for (int tries = 0; tries < 3000 && !replied; tries++) {
    if (SP->read_string(serial_states)) {
      replied = true;
      ok = is_result();
    } else {
      if (tries % 500 == 0) {
        pack_serial_data(serial_states);
        SP->write_string(serial_states);
      }
      usleep(1000);
    }
  }
  if (!replied) {
    std::cerr << "Onboard solve: no reply from MCU (check the USB connection)\n";
    return;
  }
  // Phase 2: ACK came in, now wait for the solved states
  for (int tries = 0; tries < 10000 && !ok; tries++) {
    ok = SP->read_string(serial_states) && is_result();
    if (!ok) usleep(1000);
  }
  if (ok){
    // Onboard result packet uses 11-char fields (see stm/Core/Src/main.c):
    // ! + 10 floats * 11 chars + $ = 113 bytes.
    for (int i = 0; i < 9; i++) {
      (*ptrResults)(i) = std::atof(serial_states + i * 11);
    }
    ptrstr_h->h = std::atof(serial_states + 9 * 11);
  } else {
    std::cerr << "Onboard solve: MCU replied but never returned results\n";
  }
  }
#endif
}
