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
#include <stdbool.h>

// ble headers
#include "ble_nus.h"
#include "ble_types.h"
#include "ble_acs.h"
#include "ble_bas.h"

/* ---------------- public definitions ----------------*/

/* ------------------- public enums -------------------*/

/* ----------------- public functions -----------------*/

void conn_init(void);

void conn_advertising_start(void);

ble_nus_t * conn_get_nus_instace(void);

uint16_t * conn_get_conn_handle(void);

ble_acs_t * conn_get_acs_instance(void);

ble_bas_t * conn_get_bas_instance(void);

bool conn_on_call(void);

#endif /* _CONN_MANAGER_H_ */