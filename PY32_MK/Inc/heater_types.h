/*
*   File: heater_types.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef HEATER_TYPES_H
#define HEATER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    // define alarm states here, probably related to LED indicators
    SAMPLE_HEATER = 0,
    VALVE_HEATER = 1
} heater_t;

#ifdef __cplusplus
}
#endif

#endif // HEATER_TYPES_H