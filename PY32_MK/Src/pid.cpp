/*
*   File: pid.c
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include "pid.h"

float constrain(float input_val, float min_val, float max_val) 
{
    float output_val;
    
    if (input_val < min_val) output_val = min_val;
    else if (input_val > max_val) output_val = max_val;
    else output_val = input_val;
    
    return output_val;
}

// slew_rate : 255 negates slew rate limiting
void pid_controller_init(pid_controller_t *pid, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate) {
  // Clear controller variables
  pid->integrator = 0.0f;
  pid->prevMesurement = 0.0f;
  pid->out = 0.0f;
  // Set from parameters
  pid->setpoint = setpoint;
  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;
  // Set from defines
  pid->lim_max = pid_max;
  pid->lim_min = PID_LIM_MIN;

  pid->slew_rate = slew_rate;
}

void pid_controller_compute(pid_controller_t *pid, float measurement) {
  // Error Signal
  float error = pid->setpoint - measurement;
  // Compute Integral
  //pid->integrator += (pid->k_i * error);
  pid->integrator += error;
  if (pid->integrator > pid->lim_max) {
    pid->integrator = pid->lim_max;
  } else if (pid->integrator < pid->lim_min) {
    pid->integrator = pid->lim_min;
  }
  // Compute Differential on Input
  float d_input = measurement - pid->prevMesurement;
  // Compute PID Output
  float out = pid->k_p * error + pid->k_i * pid->integrator - pid->k_d * d_input;

  // Slew Rate Limiting
  if (out - pid->out > pid->slew_rate) {
    out = pid->out + pid->slew_rate;
  } else if (pid->out - out > pid->slew_rate) {
    out = pid->out - pid->slew_rate;
  }

  if (out > pid->lim_max) {
    out = pid->lim_max;
  } else if (out < pid->lim_min) {
    out = pid->lim_min;
  } 
  
  // Set output
  pid->out = out;
  // Keep Track of for Next Execution
  pid->prevMesurement = measurement;
}

bool compute(pid_controller_t *pid, float measurement) {
  float input = measurement;
  float dInput = input - pid->prevMesurement;
  //if (action == Action::reverse) dInput = -dInput;
  pid->error = pid->setpoint - input;
  //if (action == Action::reverse) error = -error;
  float dError = pid->error - pid->lastError;

  float peTerm = pid->k_p * pid->error;
  float pmTerm = pid->k_p * dInput;
  pmTerm = 0;
  /*
  if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    }*/
  pid->pTerm = peTerm - pmTerm;

  pid->iTerm =  pid->k_i  * pid->error;
  pid->dTerm = pid->k_d * dError;
/*
  if (dmode == dMode::dOnError) 
    else dTerm = -kd * dInput; // dOnMeas
*/

  /*
  //condition anti-windup (default)
  if (iawmode == iAwMode::iAwCondition) {
    bool aw = false;
    float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
    if (iTermOut > outMax && dError > 0) aw = true;
    else if (iTermOut < outMin && dError < 0) aw = true;
    if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
  }*/

  pid->outputSum += pid->dTerm; 
    
/*
  if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);  
*/
  pid->outputSum -= pmTerm;
  pid->out = constrain(pid->outputSum + peTerm + pid->dTerm, 0, 255);
  pid->lastError = pid->error;
  pid->lastInput = input;
  return true;

}