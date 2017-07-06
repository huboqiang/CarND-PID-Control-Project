#ifndef PID_H
#define PID_H

class PID {
public:

  /*
  * Parameters
  */
  double pid_p;
  double pid_i;
  double pid_d;

  /*
  * Coefficients of CTE
  */
  double Kp;
  double Ki;
  double Kd;

  int timeI;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Update value of PID controller.
  */
  double GetUpdateValue();
};

#endif /* PID_H */
