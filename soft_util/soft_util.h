/**
 * @file soft_util.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides software utilities
 */

#ifndef _SOFT_UTIL_H_
#define _SOFT_UTIL_H_

// Standard libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ble headers
#include "ble_nus.h"
#include "ble_types.h"

/* ---------------- public definitions ----------------*/

/* ------------------- public enums -------------------*/

/* ----------------- public functions -----------------*/

ble_nus_t * soft_get_nus(void);
uint16_t * soft_get_conn_handle(void);

#endif /* _SOFT_UTIL_H_ */