/*
Controller Definition
Takes a pointer to the Controls vector defined in flightsim.hpp, pointer to the states vector,
pointer to the aircraft data, pointer to the flight_path and the autopilot settings

If you need to implement your own controllers, put the block diagram infornt of you, and
replace the transfer functions with the one you desire in the private variables here, float
check the Transfer function implementation, you have to match the order of the transfer function
*/
#pragma once
#include <iostream>
#include "structs.h"
#include "tf.hpp"
#include <cstring>
#ifdef USE_SERIAL
#include "serial.hpp"
#endif

extern float deg2rad;
extern float rad2deg;
extern float dt;
extern int step;
extern float tfinal;
extern bool Autopiloted;

class controller{
	public:
		int missed_ticks = 0;
		controller(struct Matrix* ptrControls,struct Matrix* ptrResults,
      aircraft_data *ac,flight_path *ptrstr_h, autopilot_inputs *ptrCommands);

		~controller();
    void updateControllers();
    void pitch_controller();
		void altitude_controller();
		void velocity_controller();
		void roll_controller();
		void yaw_controller();
    void pack_serial_data(char buffer[256]);
    struct Matrix* get_controls();
    void onboard_sim();
    void send_states();

  private:
#ifdef USE_SERIAL
		mySerial* SP=nullptr;
		char serial_states[256];
		char received_CA[256]; // Control action
#endif
		float da,de,dth,dr;
		struct Matrix* ptrControls;
		struct Matrix* ptrResults;
		float y,delta_theta;
    aircraft_data *ac;
		flight_path *ptrstr_h;
		autopilot_inputs *ptrCommands;

    // IF THE CONTROLLER TYPE CHANGES YOU HAVE TO CHANGE THE transferFunction IMPLEMENTATION TO MATCH IT
		// Pitch Controller
		float y_pitch=0.0,yd_pitch=0.0;
		float de_max=  25 * deg2rad;
		float de_min= -25 * deg2rad;
		transferFunction pitch_tf;
		transferFunction pitch_servo;
    float servo_num=10; float servo_den[2]={10 ,1.0};
    float pitch_num=1.9948; float pitch_den[2]={0.0 ,1.0};

		// Velocity Controller
		float y_vel=0.0,yd_vel=0.0;
		float dth_max= 167509;
		float dth_min= -40800;
		transferFunction vel_tf;
		transferFunction throttle_valve;
		transferFunction engine_lag;
    float vel_num[2]={125.2,1549.1}; float vel_den[2]={0.0 ,1.0};
    float engine_num=0.1; float engine_den[2]={0.1 ,1.0};
    float throttle_den[2]={10 ,1.0};

		// Alt Controller
		float y_alt=0.0,yd_alt=0.0;
		transferFunction alt_tf;
    float alt_num[2]={0.011498*0.3709 , 0.011498}; float alt_den[2]={10 ,1.0};

		// Roll Controller
		float da_max=     50 * deg2rad;
		float da_min=    -50 * deg2rad;
		transferFunction PI_Roll;
		transferFunction PD_Roll;
		transferFunction roll_servo;
    float pir_num[2]={0.14722*18,0.14722};float pir_den[2]={0,1};
    float pdr_num[2]={121.15*1.3,121.15};float pdr_den[2]={15,1};

		// Heading Controller
		float coordinated_roll=0.0;
		float dr_max=      15 * deg2rad;
		float dr_min=     -15 * deg2rad;
		transferFunction yaw_damper;
		transferFunction yaw_servo;
    float yaw_num[2]={0,0.88711};float yaw_den[2]={0.1,1};

};
