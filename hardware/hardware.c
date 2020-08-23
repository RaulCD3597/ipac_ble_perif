/**
 * @file hardware.c
 * @author Raul Camacho
 * @date July 2020
 */

// ipac headers
#include "hardware.h"
#include "conn_manager.h"

// Nordic common library
#include "nordic_common.h"

// nrf drivers
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_pwr_mgmt.h"

// nrf app headers
#include "app_timer.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "app_button.h"

/* ----------------  local definitions ----------------*/

#define BUTTON_DETECTION_DELAY      APP_TIMER_TICKS(50)
#define RX_PIN_NUMBER               8
#define TX_PIN_NUMBER               6
#define RTS_PIN_NUMBER              5
#define CTS_PIN_NUMBER              7
#define UART_TX_BUF_SIZE            256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE            256 /**< UART RX buffer size. */
#define REGISTER_CMD                "{\"id\": 4 , \"bed\": %d}"
#define EMERGENCY_CMD               "{\"id\": 1 , \"bed\": %d}"
#define SERVICE_CMD                 "{\"id\": 2 , \"bed\": %d}"

/* -----------------  local variables -----------------*/

/* ------------ local functions prototypes ------------*/

static void uart_init(void);
static void uart_event_handle(app_uart_evt_t * p_event);
static void timers_init(void);
static void buttons_leds_init(void);
static void button_event_handler(uint8_t pin_no, uint8_t button_action);

/* ----------------- public functions -----------------*/

/**
 * @brief Initialize hardware needed for the application
 */
void hardware_init(void)
{
    uart_init();
    timers_init();
    buttons_leds_init();
}

/**
 * @brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void hardware_sleep_mode_enter(void)
{
    uint32_t err_code;


    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/* -----------------  local functions -----------------*/

/**
 * @brief  Function for initializing the UART module.
 */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
static void uart_event_handle(app_uart_evt_t * p_event)
{
#ifdef BLE_UART_ECHO
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(conn_get_nus(), data_array, &length, *(conn_get_conn_handle()));
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
#endif
}

/**
 * @brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    // Inicializo los leds
    nrf_gpio_cfg_output(CONNECTED_LED);
    nrf_gpio_pin_set(CONNECTED_LED);

    nrf_gpio_cfg_output(TEST_LED);
    nrf_gpio_pin_set(TEST_LED);

    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {EMERGENCY_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {SERVICE_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;
    APP_TIMER_DEF(m_button_tick_timer);
    static uint32_t pressed_time;
    static uint32_t released_time;
    static bool config = false;
    bool ready = conn_is_ready();
    static uint8_t bed = 1;

    switch (pin_no)
    {
        case EMERGENCY_BUTTON:
            if (APP_BUTTON_RELEASE == button_action)
            {
                released_time = app_timer_cnt_get();
                err_code = app_timer_stop(m_button_tick_timer);
                APP_ERROR_CHECK(err_code);
                pressed_time = app_timer_cnt_diff_compute(released_time, pressed_time);
                if (30000 < pressed_time)
                {
                    config = config? false : true;
                    nrf_gpio_pin_toggle(TEST_LED);
                    break;
                }
                if (!config)
                {
                    if (!ready)
                    {
                        break;
                    }
                    uint8_t cmd_str[20];
                    sprintf((char * restrict)cmd_str, EMERGENCY_CMD, bed);
                    do
                    {
                        uint16_t length = strlen((const char *)cmd_str);
                        err_code = ble_nus_data_send(conn_get_nus(), cmd_str, &length, *(conn_get_conn_handle()));
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }
                else if (!ready)
                {
                    bed++;
                    bed = (7 < bed)? 1 : bed;
                    for (uint8_t i = 0; i < (bed * 2); i++)
                    {
                        nrf_gpio_pin_toggle(TEST_LED);
                        nrf_delay_ms(500);
                    }
                }
            }
            else
            {
                err_code = app_timer_start(m_button_tick_timer, UINT16_MAX, NULL);
                APP_ERROR_CHECK(err_code);
                pressed_time = app_timer_cnt_get();
            }
            break;

        case SERVICE_BUTTON:
            if (APP_BUTTON_RELEASE == button_action)
            {
                if (!config)
                {
                    if (!ready)
                    {
                        break;
                    }
                    uint8_t cmd_str[20];
                    sprintf((char * restrict)cmd_str, SERVICE_CMD, bed);
                    do
                    {
                        uint16_t length = strlen((const char *)cmd_str);
                        err_code = ble_nus_data_send(conn_get_nus(), cmd_str, &length, *(conn_get_conn_handle()));
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }
                else if (!ready)
                {
                    uint8_t cmd_str[20];
                    sprintf((char * restrict)cmd_str, REGISTER_CMD, bed);
                    do
                    {
                        uint16_t length = strlen((const char *)cmd_str);
                        err_code = ble_nus_data_send(conn_get_nus(), cmd_str, &length, *(conn_get_conn_handle()));
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}