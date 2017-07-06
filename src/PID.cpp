#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  pid_p = 0.0;
  pid_i = 0.0;
  pid_d = 0.0;
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  timeI = 0;
}

void PID::UpdateError(double cte) {
  pid_d  = cte - pid_p;
  pid_p  = cte;
  pid_i += cte;
  timeI += 1;
}

double PID::GetUpdateValue(){
  double updateV = pid_p * Kp + pid_i * Ki + pid_d * Kd;
  return updateV;
}
