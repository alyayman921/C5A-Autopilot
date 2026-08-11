/*
Controller Definition
Takes a pointer to the Controls vector defined in flightsim.hpp, pointer to the states vector,
pointer to the aircraft data, pointer to the flight_path and the autopilot settings

If you need to implement your own controllers, put the block diagram infornt of you, and
replace the transfer functions with the one you desire in the private variables here, double
check the Transfer function implementation, you have to match the order of the transfer function
*/
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "structs.h"
#include "tf.hpp"

#ifdef USE_SERIAL
#include "serial.hpp"
#endif

extern double deg2rad;
extern double rad2deg;
extern double dt;
extern int step;
extern bool Autopiloted;

class controller{
	public:
		int missed_ticks = 0;
		controller(Eigen::Matrix<double,4,1>* ptrControls,Eigen::Matrix<double,9,1>* ptrResults,
      aircraft_data *ac,flight_path *ptrstr_h, autopilot_inputs *ptrCommands);

		~controller();
    void updateControllers();
    void pitch_controller();
		void altitude_controller();
		void velocity_controller();
		void roll_controller();
		void yaw_controller();
    void pack_serial_data(char buffer[256]);
    Eigen::Matrix<double,4,1>* get_controls();
    void onboard_sim();
    void send_states();

  private:
#ifdef USE_SERIAL
		mySerial* SP=nullptr;
		char serial_states[256];
		char received_CA[256]; // Control action
#endif
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* ptrControls;
		Eigen::Matrix<double, 9, 1>* ptrResults;
		float y,delta_theta;
    aircraft_data *ac;
		flight_path *ptrstr_h;
		autopilot_inputs *ptrCommands;

    // IF THE CONTROLLER TYPE CHANGES YOU HAVE TO CHANGE THE transferFunction IMPLEMENTATION TO MATCH IT
		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0;
		float de_max=  25 * deg2rad;
		float de_min= -25 * deg2rad;
		transferFunction pitch_tf;
		transferFunction pitch_servo;
    double servo_num=10; double servo_den[2]={10 ,1.0};
    double pitch_num=1.9948; double pitch_den[2]={0.0 ,1.0};

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		float dth_max= 167509;
		float dth_min= -40800;
		transferFunction vel_tf;
		transferFunction throttle_valve;
		transferFunction engine_lag;
    double vel_num[2]={125.2,1549.1}; double vel_den[2]={0.0 ,1.0};
    double engine_num=0.1; double engine_den[2]={0.1 ,1.0};
    double throttle_den[2]={10 ,1.0};

		// Alt Controller
		double y_alt=0.0,yd_alt=0.0;
		transferFunction alt_tf;
    double alt_num[2]={0.011498*0.3709 , 0.011498}; double alt_den[2]={10 ,1.0};

		// Roll Controller
		float da_max=     50 * deg2rad;
		float da_min=    -50 * deg2rad;
		transferFunction PI_Roll;
		transferFunction PD_Roll;
		transferFunction roll_servo;
    double pir_num[2]={0.14722*18,0.14722};double pir_den[2]={0,1};
    double pdr_num[2]={121.15*1.3,121.15};double pdr_den[2]={15,1};

		// Heading Controller
		double coordinated_roll=0.0;
		float dr_max=      15 * deg2rad;
		float dr_min=     -15 * deg2rad;
		transferFunction yaw_damper;
		transferFunction yaw_servo;
    double yaw_num[2]={0,0.88711};double yaw_den[2]={0.1,1};

};
