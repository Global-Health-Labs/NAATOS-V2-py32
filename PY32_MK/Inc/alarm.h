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
    ERR_SELFTEST_FAILED,
    ERR_MIN_STAGE1_TEMP,
    ERR_MIN_ACTUATION_TEMP,
    ERR_OVERTEMP_SHUTDOWN,
    ERR_POWER_SUPPLY,
    ERR_FIRMWARE_CONFIG
};

enum alarm_state {
    // define alarm states here, probably related to LED indicators
    no_alarm = 0,
    sample_min_temp_not_reached,
    valve_min_temp_not_reached
};

#endif /*ALARM_H_*/