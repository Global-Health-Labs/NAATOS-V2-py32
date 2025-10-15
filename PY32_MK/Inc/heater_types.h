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
    SAMPLE_HEATER = 0,    // Sample heater control channel
    VALVE_HEATER = 1,     // Valve heater control channel
    NUM_HEATERS           // Total number of heaters (always keep last)
} heater_t;

#ifdef __cplusplus
}
#endif

#endif // HEATER_TYPES_H