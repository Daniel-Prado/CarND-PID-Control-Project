#ifndef PID_H
#define PID_H

#define LENGTH_AVERAGE_SAMPLES 100
#define LENGTH_TOTAL_ERR_SAMPLES 3000

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  /*
  * Iterations total count and error variables
  * accum_error is the same as the i_error, but I prefered to keep a separate variable
  * for monitoring purposes that could be reset, separate from the PID system internal variables.
  */ 
  long iter;
  double accum_error;
  double average_error;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the manipulated input variable (steer) of the PID system.
  * Note that this method is equivalent to the given TotalError in the project,
  * but I found that naming misleading, because the total error (CTE) is given by the
  * simulator.
  */
  double GetSteerInput();
};

#endif /* PID_H */
