/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
  Ki = Kii;
  Kdi= Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  cte = 0;
  prev_cte = 0;
  int_cte = 0;
  diff_cte;
}


void PID::UpdateError(double ctei) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  cte = ctei;
  diff_cte = ctei - prev_cte;
  prev_cte = ctei;
  int_cte += ctei;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = -Kp*cte - Kd*diff_cte/dt - Ki*int_cte*dt;
    return (control > output_lim_max ? output_lim_min : (control < output_lim_min ? output_lim_min : control));
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
}