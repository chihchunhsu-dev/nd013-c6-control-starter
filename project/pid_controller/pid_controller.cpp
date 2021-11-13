/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  kp = Kpi;
  ki = Kii;
  kd = Kdi;

  p_error = i_error = d_error = 0;

  output_limit_max = output_lim_maxi;
  output_limit_min = output_lim_mini;

  delta_time = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/

  double previous_cte = p_error;
  p_error = cte;
  i_error += cte * delta_time;
  d_error = delta_time == 0 ? 0 : (cte - previous_cte) / delta_time;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control = -kp * p_error - ki * i_error - kd * d_error;
  return max(output_limit_min, min(control, output_limit_max));
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}