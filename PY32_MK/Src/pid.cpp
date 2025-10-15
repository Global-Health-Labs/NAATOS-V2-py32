/*
*   File: pid.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include "pid.h"
#include "timers.h"
#include "alarm.h"
#include "error_handler.h"
#include "heater_types.h"

pid_controller_t pid_data[NUM_HEATERS];

extern "C" {

/**
 * @brief Constrains a value between specified minimum and maximum limits.
 * @param input_val The value to constrain.
 * @param min_val The minimum allowed value.
 * @param max_val The maximum allowed value.
 * @return The constrained value, clamped between min_val and max_val.
 */
float constrain(float input_val, float min_val, float max_val) 
{
    float output_val;
    
    if (input_val < min_val) output_val = min_val;
    else if (input_val > max_val) output_val = max_val;
    else output_val = input_val;
    
    return output_val;
}

/**
 * @brief Initializes a PID controller for a specific heater.
 * @param heater The heater to initialize (SAMPLE_HEATER or VALVE_HEATER).
 * @param setpoint The target temperature in degrees Celsius.
 * @param k_p Proportional gain coefficient.
 * @param k_i Integral gain coefficient.
 * @param k_d Derivative gain coefficient.
 * @param pid_max Maximum output value (upper limit).
 * @param slew_rate Maximum rate of change for the output (255 disables slew rate limiting).
 * 
 * Initializes PID controller state variables and parameters for the specified heater.
 * Sets up anti-windup limits, initial conditions, and validates setpoint range.
 * Will trigger ERR_FIRMWARE_CONFIG if setpoint is invalid.
 */
void pid_controller_init(heater_t heater, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate) {
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

/**
 * @brief Computes PID control output for a specified heater.
 * @param heater The heater to compute control for (SAMPLE_HEATER or VALVE_HEATER).
 * @param measurement The current temperature measurement in degrees Celsius.
 * 
 * Implements a PID controller with the following features:
 * - Initial ramp mode with maximum output until near setpoint
 * - Anti-windup on integral term
 * - Derivative on measurement (not error) to avoid derivative kick
 * - Slew rate limiting on output changes
 * - Output value clamping
 * 
 * The controller uses the following formula in normal operation:
 * output = Kp * error + Ki * ∫error dt - Kd * d(measurement)/dt
 * 
 * Updates the controller state and output value in the pid_data structure.
 */
void pid_controller_compute(heater_t heater, float measurement) {
    float out;
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
}

} // extern "C"
