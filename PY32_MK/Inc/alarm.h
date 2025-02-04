/*
*   File: alarm.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef ALARM_H_
#define ALARM_H_


enum alarm_state {
    // define alarm states here, probably related to LED indicators
    no_alarm = 0,
    valve_min_temp_not_reached
};

#endif /*ALARM_H_*/