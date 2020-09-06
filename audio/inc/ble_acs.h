/**
 * @file ble_acs.h
 * @author Raul Camacho
 * @date September 2020
 * @brief Provides Audio Custom Service for sending mic data with BLE.
 */

#ifndef _BLE_ACS_H_
#define _BLE_ACS_H_

#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include "drv_audio_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef BLE_ACS_BLE_OBSERVER_PRIO
#define BLE_ACS_BLE_OBSERVER_PRIO 2
#endif

#define BLE_ACS_DEF(_name) \
static ble_acs_t _name

#define BLE_UUID_ACS_SERVICE 0x0500                             /**< The UUID of the Audio Custom Service. */
#define BLE_ACS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3)     /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Audio Custom service module. */

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

#define BLE_ACS_MIC_FRAME_SIZE               CONFIG_AUDIO_FRAME_SIZE_BYTES

typedef PACKED( union
{
    uint8_t spl;
    uint8_t raw[BLE_ACS_MIC_FRAME_SIZE];
}) ble_acs_mic_t;

typedef enum
{
    BLE_ACS_MIC_MODE_FIRST,
    BLE_ACS_MIC_MODE_ADPCM,
    BLE_ACS_MIC_MODE_SPL,
    BLE_ACS_MIC_MODE_LAST
} ble_acs_mic_mode_t;

typedef PACKED( struct
{
    ble_acs_mic_mode_t  mic_mode;
}) ble_acs_config_t;

#define BLE_ACS_CONFIG_MIC_MODE_MIN       (BLE_ACS_MIC_MODE_FIRST + 1)
#define BLE_ACS_CONFIG_MIC_MODE_MAX       (BLE_ACS_MIC_MODE_LAST - 1)

typedef enum
{
    BLE_ACS_EVT_NOTIF_MIC,
    BLE_ACS_EVT_CONFIG_RECEIVED
}ble_acs_evt_type_t;

/* Forward declaration of the ble_acs_t type. */
typedef struct ble_acs_s ble_acs_t;

/**@brief Audio Custom Service event handler type. */
typedef void (*ble_acs_evt_handler_t) (ble_acs_t        * p_acs,
                                       ble_acs_evt_type_t evt_type,
                                       uint8_t          * p_data,
                                       uint16_t           length);

/**@brief Audio Custom Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_acs_init function.
 */
typedef struct
{
    ble_acs_config_t      * p_init_config;
    ble_acs_evt_handler_t   evt_handler; /**< Event handler to be called for handling received data. */
} ble_acs_init_t;

/**@brief Audio Custom Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_acs_s
{
    uint8_t                  uuid_type;                    /**< UUID type for Audio Custom Service Base UUID. */
    uint16_t                 service_handle;               /**< Handle of Audio Custom Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t mic_handles;                  /**< Handles related to the microphone characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t config_handles;               /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    uint16_t                 conn_handle;                  /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_mic_notif_enabled;         /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    ble_acs_evt_handler_t    evt_handler;                  /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Audio Custom Service.
 *
 * @param[out] p_acs      Audio Custom Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_acs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_acs or p_acs_init is NULL.
 */
uint32_t ble_acs_init(ble_acs_t * p_acs, const ble_acs_init_t * p_acs_init);

/**@brief Function for handling the Audio Custom Service's BLE events.
 *
 * @details The Audio Custom Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Audio Custom Service event handler of the
 * application if necessary.
 *
 * @param[in] p_acs       Audio Custom Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_acs_on_ble_evt(ble_acs_t * p_acs, ble_evt_t * p_ble_evt);

/**@brief Function for sending microphone data.
 *
 * @details This function sends the microphone input as an microphone characteristic notification to the peer.
 *
 * @param[in] p_acs       Pointer to the Audio Custom Service structure.
 * @param[in] p_data      Pointer to the mic data.
 * @param[in] size        Mic data size.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_acs_mic_set(ble_acs_t * p_acs, uint8_t * p_data, uint16_t size);

#endif /* _BLE_ACS_H_ */