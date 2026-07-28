#pragma once
#include <iostream>
class transferFunction{
	private:
		double a0,a1; // Numerator coefficients, where the polynomial has the coefficients (a1*s + a0)
		double b0,b1; // denum coefficients
		double diffed=0,dt=0,y=0,yd=0,rd=0;
		int* step;
		int size_num,size_denum; 

		struct prev_store{
			double state=0.0;
			double state_prev=0.0; // or diffrentiated in case of servo
		};
		prev_store input_history;
		prev_store servo1;
		prev_store servo2;
	public:
		transferFunction(){}
		transferFunction(int size_num,int size_denum,double *num, double *denum,int *step,double dt){
			// general constructor
			this->size_num= size_num;
			this->size_denum= size_denum;
			this->step=step;
			this->dt=dt;
			if (size_num==1){
				this->a0=num[0];
			}else if(size_num==2){
				this->a0=num[0];
				this->a1=num[1];
			}else{
				std::cerr<<"Unsupported num Size, Max = 2\n";
			}
			if (size_denum==1){
				this->b0=denum[0];
			}else if(size_denum==2){
				this->b0=denum[0];
				this->b1=denum[1];
			}else{
				std::cerr<<"Unsupported denum Size, Max = 2\n";
			}

		}
		transferFunction(int size_num,int size_denum,double num, double *denum,int *step,double dt){
			// Constructor for simple lag
			this->size_num= size_num;
			this->size_denum= size_denum;
			this->step=step;
			this->dt=dt;
			if (size_num==1){
				this->a0=num;
			}else{
				std::cerr<<"This is the simple lag constructor, what even got you here\n";
			}
			if (size_denum==1){
				this->b0=denum[0];
			}else if(size_denum==2){
				this->b0=denum[0];
				this->b1=denum[1];
			}else{
				std::cerr<<"Unsupported denum Size, Max = 2\n";
			}

		}

		double solve(double r){
			// Simple Lag case
			if (size_num == 1 && size_denum == 2){
				if (*step<1){
					yd=a0*r/b1; // (-b0*y + a0*r)/b1   but y is 0 
					return 0;
				}else{
					y+=yd*dt;
					yd=(-b0*y+a0*r)/b1;
					return y;	
				}
				return 0;
			}
			// Lead-Lag case
			if (size_num == 2 && size_denum == 2){
				if (*step<1){
					input_history.state=r;
					rd=diffrentiate(input_history);
					input_history.state_prev=r;
					yd=(a1*rd+a0*r)/b1; // yd=(-b0*y+a1*rd+a0*r)/b1  but y is 0 
					return 0;
				}else{
					y+=yd*dt;
					input_history.state=r;
					rd=diffrentiate(input_history);
					input_history.state_prev=r;
					yd=(-b0*y+a1*rd+a0*r)/b1;
					return y;	
				}
				return 0;
			}
			return 0;
		}

		double diffrentiate(prev_store &t){
			if (*step<1){
				return 0.00;
			}else{
				// (K_n - K_n-1) / dt
			   diffed= (t.state-t.state_prev)/dt;
			   t.state_prev=t.state; // update t.state when implementing
			    return diffed;
			}
		}

		void clamp_state(double min, double max){
			if (y < min) y = min;
			if (y > max) y = max;
		}

};