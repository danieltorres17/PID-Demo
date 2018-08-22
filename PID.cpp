#include "PID.h"
#include <math.h>

#define PI_QUART 0.78539816339

PID::PID(){
  Init();
}

PID::~PID(){}

void PID::Init(){
  Kp_ = 0;
  Ki_ = 0;
  Kd_ = 0;
  integral_ = 0;
  prev_error_ = 0;
  angle_ = 0;
  throttle_ = 0;
}

void PID::SetGains(double Kp, double Ki, double Kd){
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::CalculateAngle(double setpoint, double pv, double dt){

  double error = setpoint - pv;
  integral_ += error * dt;
  double derivative = (error-prev_error_)/dt;

  prev_error_ = error;

  angle_ = (Kp_*error) + (Ki_*integral_) + (Kd_*derivative);

  if(angle_ > PI_QUART){
    angle_ = PI_QUART;
  }
  else if(angle_ < -PI_QUART){
    angle_ = -PI_QUART;
  }
}

double PID::Angle(){
  return angle_;
}

//void PID::CalculateThrottle(double setpoint, double pv, double dt){

//  double error = setpoint - pv;
//  integral_ += error * dt;
//  double derivative = (error-prev_error_) * dt;

//  prev_error_ = error;

//  throttle_ = (Kp_*error) + (Ki_*integral_) + (Kd_*derivative);
//}

//double PID::Out()(){
//  return throttle_;
//}
