#ifndef _DRV_AUDIO_H_
#define _DRV_AUDIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "sdk_errors.h"
#include "sdk_macros.h"
#include "drv_audio_config.h"
#include "nrf_balloc.h"

/**@brief Audio buffer handler. */
typedef void (*drv_audio_buffer_handler_t)(int16_t * p_buffer);

/**@brief Enable audio source.
 *
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
ret_code_t drv_audio_enable(void);

/**@brief Disable audio source.
 *
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
ret_code_t drv_audio_disable(void);

/**@brief Initialization.
 *
 * @param[in] buffer_handler    Handler which will be called when buffer with RAW audio is ready.
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_INTERNAL
 */
ret_code_t dvr_audio_init(drv_audio_buffer_handler_t buffer_handler);

#endif /* _DRV_AUDIO_H_ */