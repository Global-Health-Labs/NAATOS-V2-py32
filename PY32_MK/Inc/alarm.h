/*
*   File: alarm.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef ALARM_H_
#define ALARM_H_

enum error_state {
    ERR_NONE = 0,
    ERR_PREHEAT_TIMEOUT,
    ERR_INVALID_USB_POWER_SOURCE,
    ERR_SELFTEST1_FAILED,
    ERR_SELFTEST2_FAILED,
    ERR_MIN_AMPLIFICATION_TEMP,
    ERR_MIN_ACTUATION_TEMP,
    ERR_OVERTEMP_SHUTDOWN,
    ERR_ADC_CONFIG,
    ERR_TIMER_CONFIG,
    ERR_PID_CONFIG
};

enum alarm_state {
    // define alarm states here, probably related to LED indicators
    no_alarm = 0,
    sample_min_temp_not_reached,
    valve_min_temp_not_reached
};

#endif /*ALARM_H_*/