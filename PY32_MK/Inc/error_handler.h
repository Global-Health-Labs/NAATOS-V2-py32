/*
*   File: error_handler.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*
*   Description:
*   This module provides error handling functionality for critical system failures.
*   It implements a visual feedback system using LED blink patterns to indicate
*   error codes and ensures safe shutdown of heater systems on failure.
*/

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error code definitions for system failures
 */
enum ErrorCodes {
    ERR_FIRMWARE_CONFIG             = 1,  /*!< Invalid firmware configuration detected */
    ERR_POWER_SUPPLY                = 2,  /*!< Power supply voltage out of range */
    ERR_INVALID_USB_POWER_SOURCE    = 3,  /*!< USB power source cannot supply required current */
    ERR_SELFTEST_FAILED             = 4,  /*!< Self-test sequence failed */
    ERR_HEATER_TIMEOUT              = 5,  /*!< Heater failed to reach target temperature in time */
    ERR_OVERTEMP_SHUTDOWN           = 6,  /*!< Over-temperature condition detected */
    ERR_MIN_AMPLIFICATION_TEMP      = 7,  /*!< Minimum amplification temperature not reached */
    ERR_MIN_ACTUATION_TEMP          = 8   /*!< Minimum actuation temperature not reached */
};

void APP_ErrorHandler(uint8_t errnum);

#ifdef __cplusplus
}
#endif

#endif // ERROR_HANDLER_H