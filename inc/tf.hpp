#pragma once
extern int step;
extern double dt;
#include <iostream>
class transferFunction{
	private:
		double a0,a1; // Numerator coefficients, where the polynomial has the coefficients (a1*s + a0)
		double b0,b1; // denum coefficients
		double diffed=0,y=0,yd=0,rd=0;
		int size_num,size_denum;

		struct prev_store{
			double state=0.0;
			double state_prev=0.0; // or diffrentiated in case of servo
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
