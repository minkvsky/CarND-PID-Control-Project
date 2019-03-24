#include "PID.h"
// using std::cout;
// using std::endl;
#include <iostream>
#include <math.h>
using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   p_error = i_error = d_error = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // return 0.0;  // TODO: Add your total error calc here!
  return - p_error * Kp - i_error * Ki - d_error * Kd;
}

PID twiddle(PID pid, double cte, double tol, double *dp) {
  // static double p[3] = {0.0, 0.0, 0.0}; as glo
  double p[3] = {pid.Kp, pid.Ki, pid.Kd};
  // double dp[3] = {1, 1, 1}; // maybe adjust this value, need global
  // pid.Init(p[0], p[1], p[2]);
  pid.UpdateError(cte); // pid init already
  double best_err = fabs(pid.TotalError());
  cout << "zero update " << best_err << endl;
  double err;

  while (dp[0] + dp[1] + dp[2] > tol){
    // only ajust once for one call twiddle, so replace while by if
    // cout << "p,i,d: " << pid.Kp << " " << pid.Ki << " " << pid.Kd << endl;
    for (int i =0; i < 3; i++ ){
      p[i] += dp[i];
      pid.Init(p[0], p[1], p[2]);
      pid.UpdateError(cte);
      err = fabs(pid.TotalError());
      cout << "first update " << err << endl;
      if (err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      }else {
        p[i] -= 2 * dp[i];
        pid.Init(p[0], p[1], p[2]);
        pid.UpdateError(cte);
        err = fabs(pid.TotalError());
        cout << "second update " << err << endl;
        if (err < best_err){
          best_err = err;
          dp[i] *= 1.1;
        }else{
          p[i] += dp[i];
          dp[i] *= 0.9;
        };

      };
    };
  };

  cout << "sum dp" << dp[0] + dp[1] + dp[2] << endl;
  cout <<
  cout << "p,i,d: " << pid.Kp << " " << pid.Ki << " " << pid.Kd << endl;
  return pid;
}
