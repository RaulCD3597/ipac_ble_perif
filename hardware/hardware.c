/**
 * @file hardware.c
 * @author Raul Camacho
 * @date July 2020
 */

#pragma GCC optimize ("O0")

// ipac headers
#include "hardware.h"
#include "conn_manager.h"
#include "drv_mic.h"

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
#include "nrf_drv_saadc.h"

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
#define EMERGENCY_CMD               "{\"id\": 1}"
#define SERVICE_CMD                 "{\"id\": 2}"
#define SAMPLES_IN_BUFFER           5
#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(2000) /**< Battery level measurement interval (ticks). */
#define FULL_BATTERY_MEAS           450

/* -----------------  local variables -----------------*/

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
APP_TIMER_DEF(m_battery_timer_id);     /**< Battery timer. */

/* ------------ local functions prototypes ------------*/

static void uart_init(void);
static void uart_event_handle(app_uart_evt_t * p_event);
static void timers_init(void);
static void buttons_leds_init(void);
static void button_event_handler(uint8_t pin_no, uint8_t button_action);
static u_int32_t drv_mic_data_handler(m_audio_frame_t * p_frame);
static void saadc_init(void);
static void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
static void battery_level_meas_timeout_handler(void * p_context);

/* ----------------- public functions -----------------*/

/**
 * @brief Initialize hardware needed for the application
 */
void hardware_init(void)
{
    uint32_t err_code;

    uart_init();
    timers_init();
    buttons_leds_init();
    err_code = drv_mic_init(drv_mic_data_handler);
    APP_ERROR_CHECK(err_code);
    saadc_init();
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
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
                        err_code = ble_nus_data_send(conn_get_nus_instace(), data_array, &length, *(conn_get_conn_handle()));
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
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
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

    switch (pin_no)
    {
        case EMERGENCY_BUTTON:
            if (APP_BUTTON_RELEASE == button_action)
            {
                if ( !conn_on_call() )
                {
                    return;
                }
                uint8_t cmd_str[20];
                sprintf((char * restrict)cmd_str, EMERGENCY_CMD);
                do
                {
                    uint16_t length = strlen((const char *)cmd_str);
                    err_code = ble_nus_data_send(conn_get_nus_instace(), cmd_str, &length, *(conn_get_conn_handle()));
                    if ((err_code != NRF_ERROR_INVALID_STATE) &&
                        (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_RESOURCES);
            }
            break;

        case SERVICE_BUTTON:
            if (APP_BUTTON_RELEASE == button_action)
            {
                if ( !conn_on_call() )
                {
                    return;
                }
                uint8_t cmd_str[20];
                sprintf((char *restrict)cmd_str, SERVICE_CMD);
                do
                {
                    uint16_t length = strlen((const char *)cmd_str);
                    err_code = ble_nus_data_send(conn_get_nus_instace(), cmd_str, &length, *(conn_get_conn_handle()));
                    if ((err_code != NRF_ERROR_INVALID_STATE) &&
                        (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_RESOURCES);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

static u_int32_t drv_mic_data_handler(m_audio_frame_t * p_frame)
{
    ble_acs_mic_set(conn_get_acs_instance(), p_frame->data, p_frame->data_size);

    return NRF_SUCCESS;
}

static void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        if ( !conn_on_call() )
        {
            return;
        }

        uint16_t battery_level = 0;

        for(int8_t i = 0; i< SAMPLES_IN_BUFFER; i++)
        {
            battery_level += p_event->data.done.p_buffer[i];
        }
        battery_level /= SAMPLES_IN_BUFFER;

        battery_level = (battery_level * 100) / FULL_BATTERY_MEAS;
        if (battery_level > 100)
        {
            battery_level = 100;
        }

        err_code = ble_bas_battery_level_update(conn_get_bas_instance(), (uint8_t)battery_level, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_drv_saadc_sample();
}