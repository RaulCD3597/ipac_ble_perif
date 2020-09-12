#ifndef __DRV_MIC_H__
#define __DRV_MIC_H__

#include <stdint.h>
#include "drv_audio_config.h"

/**@brief Compressed audio frame representation.
 */
typedef struct
{
    u_int8_t     data[CONFIG_AUDIO_FRAME_SIZE_BYTES];
    u_int16_t    data_size;
} m_audio_frame_t;

/**@brief Microphone driver data handler callback type.
 */
typedef u_int32_t (*drv_mic_data_handler_t)(m_audio_frame_t * p_frame);

/**@brief Function for starting the microphone driver.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
u_int32_t drv_mic_start(void);

/**@brief Function for stopping the microphone driver.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
u_int32_t drv_mic_stop(void);

/**@brief Function for initializing the microphone driver.
 *
 * @param[in] data_handler      Pointer data handler callback.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
u_int32_t drv_mic_init(drv_mic_data_handler_t data_handler);

#endif