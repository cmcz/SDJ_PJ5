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
   this->K_p = Kpi;
   this->K_i = Kii;
   this->K_d = Kdi;
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini; 

   this->err_p = 0.0;
   this->err_i = 0.0;
   this->err_d = 0.0;

   this->delta_time = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

   if (this->delta_time > 0.0001){
      this->err_d = (cte - err_p) / this->delta_time;
   } else {
      this->err_d = 0; 
   }

   this->err_p = cte;
   this->err_i += cte * delta_time;
   
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = (this->K_p * this->err_p + this->K_i * this->err_i + this->K_d * this->err_d);
   if (control > this->output_lim_max) {
      control = this->output_lim_max;
   } else if (control < this->output_lim_min){
      control = this->output_lim_min;
   }

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_time = new_delta_time;
   return this->delta_time;
}