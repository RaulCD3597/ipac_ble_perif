/**
 * @file ble_acs.c
 * @author Raul Camacho
 * @date September 2020
 * @brief Provides Audio Custom Service for sending mic data with BLE.
 */

#include "ble_acs.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_ACS_CONFIG_CHAR      0x0501                      /**< The UUID of the config Characteristic. */
#define BLE_UUID_ACS_MIC_CHAR         0x0502                      /**< The UUID of the microphone Characteristic. */

#define BLE_ACS_MAX_RX_CHAR_LEN        BLE_ACS_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_ACS_MAX_TX_CHAR_LEN        BLE_ACS_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

// 50E9xxxx-693A-4E74-B11F-C91C353C4263
 #define ACS_BASE_UUID  {{0x63, 0x42, 0x3C, 0x35, 0x1C, 0xC9, 0x1F, 0xB1, \
                          0x74, 0x4E, 0x3A, 0x69, 0x00, 0x00, 0xE9, 0x50}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_acs     Audio Custom Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_acs_t * p_acs, ble_evt_t * p_ble_evt)
{
    p_acs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_acs     Audio Custom Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_acs_t * p_acs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_acs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_acs     Audio Custom Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_acs_t * p_acs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;


    if ( (p_evt_write->handle == p_acs->mic_handles.cccd_handle) &&
         (p_evt_write->len == 2) )
    {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_acs->is_mic_notif_enabled != notif_enabled)
        {
            p_acs->is_mic_notif_enabled = notif_enabled;

            if (p_acs->evt_handler != NULL)
            {
                p_acs->evt_handler(p_acs, BLE_ACS_EVT_NOTIF_MIC, p_evt_write->data, p_evt_write->len);
            }
        }
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

static void on_authorize_req(ble_acs_t * p_acs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type  == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (p_evt_rw_authorize_request->request.write.handle == p_acs->config_handles.value_handle)
        {
            ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
            bool                                  valid_data = true;

            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_acs_config_t))
            {
                valid_data = false;
            }
            else
            {
                ble_acs_config_t * p_config = (ble_acs_config_t *)p_evt_rw_authorize_request->request.write.data;

                if ( (p_config->mic_mode < BLE_ACS_CONFIG_MIC_MODE_MIN)      ||
                     (p_config->mic_mode > BLE_ACS_CONFIG_MIC_MODE_MAX) )
                {
                    valid_data = false;
                }
            }

            rw_authorize_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

            if (valid_data)
            {
                rw_authorize_reply.params.write.update      = 1;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                rw_authorize_reply.params.write.p_data      = p_evt_rw_authorize_request->request.write.data;
                rw_authorize_reply.params.write.len         = p_evt_rw_authorize_request->request.write.len;
                rw_authorize_reply.params.write.offset      = p_evt_rw_authorize_request->request.write.offset;
            }
            else
            {
                rw_authorize_reply.params.write.update      = 0;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
            }

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);

            if ( valid_data && (p_acs->evt_handler != NULL))
            {
                p_acs->evt_handler(p_acs,
                                   BLE_ACS_EVT_CONFIG_RECEIVED,
                                   p_evt_rw_authorize_request->request.write.data,
                                   p_evt_rw_authorize_request->request.write.len);
            }
        }
    }
}

/**@brief Function for adding microphone characteristic.
 *
 * @param[in] p_acs       Audio Custom Service structure.
 * @param[in] p_acs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t mic_char_add(ble_acs_t * p_acs, const ble_acs_init_t * p_acs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_acs_mic_t       init_mic;

    memset(&init_mic, 0, sizeof(ble_acs_mic_t));
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify   = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    ble_uuid.type = p_acs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACS_MIC_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_acs_mic_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&init_mic;
    attr_char_value.max_len   = sizeof(ble_acs_mic_t);

    return sd_ble_gatts_characteristic_add(p_acs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_acs->mic_handles);
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_acs       Audio Custom Service structure.
 * @param[in] p_acs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t config_char_add(ble_acs_t * p_acs, const ble_acs_init_t * p_acs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_acs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACS_CONFIG_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_acs_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_acs_init->p_init_config;
    attr_char_value.max_len   = sizeof(ble_acs_config_t);

    return sd_ble_gatts_characteristic_add(p_acs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_acs->config_handles);
}

void ble_acs_on_ble_evt(ble_acs_t * p_acs, ble_evt_t * p_ble_evt)
{
    if ((p_acs == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_acs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_acs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_acs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_authorize_req(p_acs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_acs_init(ble_acs_t * p_acs, const ble_acs_init_t * p_acs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t acs_base_uuid = ACS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_acs);
    VERIFY_PARAM_NOT_NULL(p_acs_init);

    // Initialize the service structure.
    p_acs->conn_handle                  = BLE_CONN_HANDLE_INVALID;
    p_acs->evt_handler                  = p_acs_init->evt_handler;
    p_acs->is_mic_notif_enabled         = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&acs_base_uuid, &p_acs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_acs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_acs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    err_code = config_char_add(p_acs, p_acs_init);
    VERIFY_SUCCESS(err_code);

    // Add the microphone Characteristic.
    err_code = mic_char_add(p_acs, p_acs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_acs_mic_set(ble_acs_t * p_acs, uint8_t * p_data, uint16_t size)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_acs);

    if ((p_acs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_acs->is_mic_notif_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (size > BLE_ACS_MIC_FRAME_SIZE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_acs->mic_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = &size;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_acs->conn_handle, &hvx_params);
}