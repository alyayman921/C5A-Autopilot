#include "tf.hpp"
transferFunction::transferFunction(){}
transferFunction::transferFunction(int size_num,int size_denum,float *num, float *denum){
  // general constructor
  this->size_num= size_num;
  this->size_denum= size_denum;
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
transferFunction::transferFunction(int size_num,int size_denum,float num, float *denum){
  // Constructor for simple lag
  this->size_num= size_num;
  this->size_denum= size_denum;
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

float transferFunction::solve(float r){
  // Simple Lag case
  if (size_num == 1 && size_denum == 2){
    if (step<1){
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
    if (step<1){
      input_history.state=r;
      rd=transferFunction::diffrentiate(input_history);
      input_history.state_prev=r;
      yd=(a1*rd+a0*r)/b1; // yd=(-b0*y+a1*rd+a0*r)/b1  but y is 0
      return 0;
    }else{
      y+=yd*dt;
      input_history.state=r;
      rd=transferFunction::diffrentiate(input_history);
      input_history.state_prev=r;
      yd=(-b0*y+a1*rd+a0*r)/b1;
      return y;
    }
    return 0;
  }
  return 0;
}

float transferFunction::diffrentiate(prev_store &t){
  if (step<1){
    return 0.00;
  }else{
    // (K_n - K_n-1) / dt
      diffed= (t.state-t.state_prev)/dt;
      t.state_prev=t.state; // update t.state when implementing
      return diffed;
  }
}

void transferFunction::clamp_state(float min, float max){
  if (y < min) y = min;
  if (y > max) y = max;
}

