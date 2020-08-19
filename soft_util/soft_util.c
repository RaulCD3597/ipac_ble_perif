/**
 * @file soft_util.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides software utilities
 */

// ipac headers
#include "soft_util.h"

// Nordic common library
#include "nordic_common.h"

/* ----------------  local definitions ----------------*/

/* -----------------  local variables -----------------*/

/**
 * BLE NUS service instance.
 */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
/**
 * Handle of the current connection.
 */
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;

/* ------------ local functions prototypes ------------*/

/* ----------------- public functions -----------------*/

/**
 * @brief fuction for get the nus instace
 */
ble_nus_t * soft_get_nus(void)
{
    return ((ble_nus_t *)&m_nus);
}
/**
 * @brief fuction for get the connection handle
 */
uint16_t * soft_get_conn_handle(void)
{
    return ((uint16_t *)&m_conn_handle);
}

/* -----------------  local functions -----------------*/