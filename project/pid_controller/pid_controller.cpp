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
  this->Kp = Kpi;
  this->Ki = Kii;
  this->Kd= Kdi;
  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;
  this->cte = 0.0;
  this->prev_cte = 0.0;
  this->int_cte = 0.0;
  this->diff_cte = 0.0;
}


void PID::UpdateError(double ctei) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  this->cte = ctei;
  this->diff_cte = (ctei - this->prev_cte)/std::max(this->dt, 0.01);
  this->prev_cte = ctei;
  this->int_cte += ctei*this->dt;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = Kp*cte + Kd*diff_cte + Ki*int_cte;
   cout << "# Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << ", cte = " << cte << ", diff_cte = " << diff_cte << ", int_cte = " << int_cte << endl;
   //return (control > output_lim_max ? output_lim_min : (control < output_lim_min ? output_lim_min : control));
   if (control < this->output_lim_min){
      control = this->output_lim_min;
   } else if(control > this->output_lim_max){
      control = this->output_lim_max;
   }
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  this->dt = new_delta_time;
  return new_delta_time;
}