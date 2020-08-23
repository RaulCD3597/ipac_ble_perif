/**
 * @file conn_manager.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage BLE connection for IPAC_BLE central device
 */

#ifndef _CONN_MANAGER_H_
#define _CONN_MANAGER_H_

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

void conn_init(void);

void conn_advertising_start(void);

ble_nus_t * conn_get_nus(void);

uint16_t * conn_get_conn_handle(void);

bool conn_is_ready(void);

void conn_set_ready(void);

#endif /* _CONN_MANAGER_H_ */