/*
Transfer Function Definition, solves the euler integration output for one timestep
doesn't need dt
Needs to define
1. Number of Terms Numerator and Denumerator, including zero if a higher order term is there
2. Numerator and Denumerator coefficients, where the Numerator polynomial has the coefficients (a1*s + a0)
If you ever used matlab, this is close to how you define in it
*/
#pragma once
#include <iostream>

extern int step;
extern double dt;

class transferFunction{
	private:
		double a0,a1;
		double b0,b1;
		double diffed=0,y=0,yd=0,rd=0;
		int size_num,size_denum;

		struct prev_store{
			double state=0.0;
			double state_prev=0.0;
		};

		prev_store input_history;
		prev_store servo1;
		prev_store servo2;

	public:
		transferFunction();
		transferFunction(int size_num,int size_denum,double *num, double *denum);
		transferFunction(int size_num,int size_denum,double num, double *denum);
		double solve(double r);
		double diffrentiate(prev_store &t);
		void clamp_state(double min, double max);

};
