#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdlib>
#include "serial.hpp"
#include "tf.hpp"
#include "serial.hpp"
#include "serial_scanner.hpp"
extern double deg2rad;
extern double rad2deg;

class controller{
	private:
		mySerial SP;
		char serial_states[256];
		char received_CA[256]; // Control action
		bool Autopiloted;
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* Controls;
		Eigen::Matrix<double, 9, 1>* results;
		float y,delta_theta;
		double dt=0.01;
		int current_step;
		int* step;
		aircraft_data *ac;
		flight_path *str_h;
		autopilot_inputs *commands;

		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0; 
		float de_max=  25 * deg2rad;
		float de_min= -25 * deg2rad;
		transferFunction pitch_tf;
		transferFunction pitch_servo;

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		float dth_max= 167509;
		float dth_min= -40800;
		transferFunction vel_tf;
		transferFunction throttle_valve;
		transferFunction engine_lag;

		// Alt Controller
		double y_alt=0.0,yd_alt=0.0;
		transferFunction alt_tf;

		// Roll Controller
		float da_max=     25 * deg2rad;
		float da_min=    -25 * deg2rad;
		transferFunction PI_Roll;
		transferFunction PD_Roll;
		transferFunction roll_servo;

		// Heading Controller 
		double coordinated_roll=0.0;
		transferFunction yaw_damper;
		transferFunction yaw_servo;


	public:
		int missed_ticks = 0;
		controller(Eigen::Matrix<double,4,1>* Controls,aircraft_data *ac ,flight_path *str_h ,
			double dt,int *step,autopilot_inputs *commands ,bool Autopiloted){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->dt=dt;
			this->str_h=str_h;
			this->step=step;
			this->ac=ac;
			this->commands=commands;
			if(commands->ext_controller){
				std::string sp=getSerialPort();
	            this->SP= mySerial(sp);
			}
			// Pitch
			double servo_num=10; double servo_den[2]={10 ,1.0};
			double pitch_num=1.9948; double pitch_den[2]={0.0 ,1.0};
			this->pitch_tf=transferFunction(1 , 2 , pitch_num , pitch_den ,step ,dt);
			this->pitch_servo=transferFunction(1 , 2, servo_num, servo_den , step ,dt);
			// velocity
			double vel_num[2]={125.2,1549.1}; double vel_den[2]={0.0 ,1.0};
			double engine_num=0.1; double engine_den[2]={0.1 ,1.0};
			this->vel_tf=transferFunction(2 , 2, vel_num, vel_den , step ,dt);
			double throttle_den[2]={10 ,1.0};
			this->throttle_valve=transferFunction(1 , 2, 10.0, throttle_den , step ,dt);
			this->engine_lag=transferFunction(1 , 2, engine_num, engine_den , step ,dt);
			// altitude
			double alt_num[2]={0.011498*0.3709 , 0.011498}; double alt_den[2]={10 ,1.0};
			this->alt_tf=transferFunction(1 , 2, alt_num, alt_den , step ,dt);
			// roll
			double pir_num[2]={0.14722*18,0.14722};double pir_den[2]={0,1};
			this->PI_Roll=transferFunction(2 , 2, pir_num, pir_den, step, dt);
			double pdr_num[2]={121.15*1.3,121.15};double pdr_den[2]={15,1};
			this->PD_Roll=transferFunction(2 , 2, pdr_num, pdr_den, step, dt);
			this->roll_servo=transferFunction(1 , 2, servo_num, servo_den, step, dt);
			// yaw
			double yaw_num[2]={0,0.88711};double yaw_den[2]={0.1,1};
			this->yaw_damper=transferFunction(2 , 2, yaw_num, yaw_den, step, dt);
			this->yaw_servo=transferFunction(1 , 2, servo_num, servo_den, step, dt);

			*Controls={0,0,0,0};
		}
		void rk4_pointers(Eigen::Matrix<double, 9, 1>* results){
			this->results=results; // store a pointer to results vector
		}

		void pitch_controller(){

			if (Autopiloted&&!(commands->ext_controller)){
				delta_theta=results[*step][7]-results[0][7];
				y_pitch=pitch_tf.solve(((commands->set_pitch)-delta_theta));
				de=-(y_pitch - ((delta_theta*1.734+results[*step][4])*1.5236));
				de=pitch_servo.solve(de);
				if (de>de_max){de=de_max;}
				if (de<de_min){de=de_min;}
				*Controls={da,de,dth,dr};
				}
			}
		
		void altitude_controller(){
			/*
			    0.011498 (s+0.3709)
  				-------------------
        				(s+10)
			*/
			if (Autopiloted&&!(commands->alt_override)&&!(commands->ext_controller)){
					y_alt=alt_tf.solve(((commands->set_alt)-str_h->h));
					commands->set_pitch=y_alt;
				}
			}

		void velocity_controller(){
			/*
			    1549.1 (s+0.008082)
  				------------------- = 1549.1 + 125.2/s
           				s
			*/
			if (Autopiloted&&!(commands->ext_controller)){
				y_vel=vel_tf.solve(((commands->set_vel)-results[*step][0]));
				dth=throttle_valve.solve(y_vel);
				dth=engine_lag.solve(dth);
				if (dth>dth_max){dth=dth_max; vel_tf.clamp_state(dth_min, dth_max);}
				if (dth<dth_min){dth=dth_min; vel_tf.clamp_state(dth_min, dth_max);}
				*Controls={da,de,dth,dr};
				//std::cout<<"current Thrust"<<dth<<"\n";
			}
		}
// -------------------------------------- LATERAL CONTROLLERS ---------------------------------------
		
		void roll_controller(){
			/*
			     PI_Roll 
			  0.14722 (s+18)
			  -------------- 
			        s

				  PD_Roll
			  121.15 (s+1.3)
			  --------------
			      (s+15)
 
			*/
			if(Autopiloted&&!(commands->ext_controller)){
				da=PI_Roll.solve(((commands->set_roll)-results[*step][6]))-PD_Roll.solve(results[*step][6]);
				da=roll_servo.solve(da);
				*Controls={da,de,dth,dr};
			}
		}
		void yaw_controller(){
			/*
			   damper
			  0.88711 s
			  ---------
			   (s+0.1)
			*/
			if(Autopiloted&&!(commands->head_override)&&!(commands->ext_controller)){
				coordinated_roll=((commands->set_heading)-results[*step][8])*(results[0][0])/ac->g/10;
			if (coordinated_roll>25*deg2rad){coordinated_roll=25*deg2rad;}
			if (coordinated_roll<-25*deg2rad){coordinated_roll=-25*deg2rad;}
				commands->set_roll=coordinated_roll;
				//roll_controller();
				dr=yaw_damper.solve(results[*step][5]);
				dr=yaw_servo.solve(dr);
				*Controls={da,de,dth,dr};
			}

		}
// --------------- conv floats to 9 char Send To Serial

	// Memory Inefficient Black Majic from the ai, don't @ me 
	void pack_serial_data(char buffer[256]) {
	    char *ptr = buffer;

	    // ---- First 10 chars: booleans as '0' or '1' ----
	    *ptr++ = Autopiloted ? '1' : '0';
	    *ptr++ = commands->alt_override ? '1' : '0';
	    *ptr++ = commands->head_override ? '1' : '0';
	    *ptr++ = commands->ext_controller ? '1' : '0';
	    // pad remaining 6 positions with '0'
	    for (int i = 0; i < 6; ++i) *ptr++ = '0';

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
	    append_float(commands->set_pitch);
	    append_float(commands->set_vel);
	    append_float(commands->set_alt);
	    append_float(commands->set_heading);
	    append_float(commands->set_roll);

	    // ---- 9 results from state vector (9 × 9 = 81 chars) ----
	    for (int i = 0; i < 9; ++i) {
	        append_float(results[*step](i));
	    }

	    // ---- 6 flight‑path variables (6 × 9 = 54 chars) ----
	    append_float(str_h->h);
	    append_float(str_h->v_tot);
	    append_float(str_h->delta_h_dot);
	    append_float(str_h->alpha);
	    append_float(str_h->beta);
	    append_float(str_h->gamma);

	    // ---- Fill the rest of the 256‑byte buffer with zeros ----
	    size_t used = ptr - buffer;
	    if (used < 256) {
	        memset(ptr, 0, 256 - used);
	    }
	}
	Eigen::Matrix<double,4,1>* get_controls() { return Controls; }
	void send_states(){
		if(commands->ext_controller){
			if (SP.read_string(received_CA)) {
				*Controls = {std::atof(received_CA),
				             std::atof(received_CA + 9),
				             std::atof(received_CA + 18),
				             std::atof(received_CA + 27)};
			}
			pack_serial_data(serial_states);
			SP.write_string(serial_states);
		}
	}
};