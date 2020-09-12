/** 
 * @file main.c
 * @author Raul Camacho
 * @date July 2020
 */

// ipac headers
#include "conn_manager.h"
#include "hardware.h"

// Nordic common library
#include "nordic_common.h"

// nrf drivers
#include "nrf.h"
#include "nrf_pwr_mgmt.h"

// nrf app headers
#include "app_error.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "app_timer.h"

/* ----------------  local definitions ----------------*/

#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

/* -----------------  local variables -----------------*/

/* -----------------  local functions -----------------*/

/**
 * @brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling the idle state (main loop).
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    nrf_pwr_mgmt_run();
}

/* ----------------- public functions -----------------*/

/**
 * @brief Application main function.
 */
int main(void)
{
    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    hardware_init();
    power_management_init();
    conn_init();

    // Start execution.
    conn_advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */