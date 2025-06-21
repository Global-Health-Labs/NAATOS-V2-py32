/*
*   File: pid.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include "pid.h"
#include "timers.h"
#include "alarm.h"

// PID structures
pid_controller_t pid_data[NUM_HEATERS];

pid_init_t H1_pid_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},
  {STAGE1_H1_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE2_H1_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE3_H1_TARGET_C,0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0}
};
pid_init_t H2_pid_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},
  {STAGE1_H2_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE2_H2_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE3_H2_TARGET_C,0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0}
};

pid_init_t H3_pid_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},
  {STAGE1_H3_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE2_H3_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE3_H3_TARGET_C, 0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0}
};
pid_init_t H4_pid_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},
  {STAGE1_H4_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE2_H4_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {STAGE3_H4_TARGET_C, 0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0}
};

float constrain(float input_val, float min_val, float max_val) 
{
    float output_val;
    
    if (input_val < min_val) output_val = min_val;
    else if (input_val > max_val) output_val = max_val;
    else output_val = input_val;
    
    return output_val;
}

// slew_rate : 255 negates slew rate limiting
void pid_controller_init(heater_t heater, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate) {
	if (heater > NUM_HEATERS-1) {
		APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
		return;
	}
	
    // Clear controller variables
    pid_data[heater].integrator = 0.0f;
    pid_data[heater].prevMesurement = 0.0f;
    pid_data[heater].out = 0.0f;
    pid_data[heater].ramp_start = true;
    // Set from parameters
    pid_data[heater].setpoint = setpoint;
    if (setpoint < 0 || setpoint > OVERTEMP_ERR_C)
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
      
    pid_data[heater].k_p = k_p;
    pid_data[heater].k_i = k_i;
    pid_data[heater].k_d = k_d;
    // Set from defines
    pid_data[heater].lim_max = pid_max;
    pid_data[heater].lim_min = PID_LIM_MIN;

    pid_data[heater].slew_rate = slew_rate;
}

void pid_controller_compute(heater_t heater, float measurement) {
    float out;
#ifndef DEBUG_REDUCE_MEMORY
    
    // Don't perform the PID calculation if the setpoint is 0.
    if (pid_data[heater].setpoint <= PID_LIM_MIN) {
        pid_data[heater].out = 0;
        return;
    }
    
    // Error Signal
    float error = pid_data[heater].setpoint - measurement;
  
    // disable the integrator until we are close to the setpoint.
    if (pid_data[heater].ramp_start) {
        out = pid_data[heater].lim_max;
        pid_data[heater].integrator = 0.0f; 
        if (error < PID_ANTIWINDUP_ERR_RANGE) {
            pid_data[heater].ramp_start = false;
        }
    } else {
        // Compute Integral
        pid_data[heater].integrator += pid_data[heater].k_i * error * PID_DELTA_T;
  
        if (pid_data[heater].integrator > pid_data[heater].lim_max) {
            pid_data[heater].integrator = pid_data[heater].lim_max;
        } else if (pid_data[heater].integrator < pid_data[heater].lim_min) {
            pid_data[heater].integrator = pid_data[heater].lim_min;
        }
        // Compute Differential on Input
        pid_data[heater].dTerm = pid_data[heater].k_d * (measurement - pid_data[heater].prevMesurement);
        // Compute PID Output
        out = pid_data[heater].k_p * error + pid_data[heater].integrator - pid_data[heater].dTerm;
        
        // Slew Rate Limiting
        if (out - pid_data[heater].out > pid_data[heater].slew_rate) {
            out = pid_data[heater].out + pid_data[heater].slew_rate;
        } else if (pid_data[heater].out - out > pid_data[heater].slew_rate) {
            out = pid_data[heater].out - pid_data[heater].slew_rate;
        }

        if (out > pid_data[heater].lim_max) {
            out = pid_data[heater].lim_max;
        } else if (out < pid_data[heater].lim_min) {
            out = pid_data[heater].lim_min;
        } 
    }

    // Set output
    pid_data[heater].out = out;
    // Keep Track of for Next Execution
    pid_data[heater].prevMesurement = measurement;
#endif
}
