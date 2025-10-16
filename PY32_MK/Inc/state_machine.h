/*
*   File: state_machine.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

/**
 * @brief Main application state machine states.
 * 
 * Defines the operational states of the NAATOS system. The order of these states
 * is significant as they map directly to indices in the control structure array.
 * The state progression is typically:
 * 1. low_power -> self_test_1 -> self_test_2 -> preheat (during startup)
 * 2. preheat -> amplification_ramp -> amplification (normal operation)
 * 3. amplification -> actuation_ramp -> actuation -> detection (test sequence)
 * 4. detection -> low_power (completion)
 */
typedef enum {
    low_power = 0,        /*!< System idle, heaters disabled */
    amplification_ramp,   /*!< Initial temperature ramp to amplification target */
    amplification,        /*!< Maintain amplification temperature */
    actuation,           /*!< Actuation phase at elevated temperature */
    detection,           /*!< Final detection phase */
    actuation_ramp,      /*!< Temperature ramp to actuation target */
    self_test_1,         /*!< Initial self-test phase (low power) */
    self_test_2,         /*!< Secondary self-test phase (high power) */
    preheat,             /*!< Initial heating phase */
    NUM_STATES           /*!< Total number of states (always keep last) */
} state_machine_t;       

#endif /* STATE_MACHINE_H_*/