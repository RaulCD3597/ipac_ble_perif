#pragma GCC optimize ("O0")

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrfx_pdm.h"

#include "drv_audio.h"

#if CONFIG_AUDIO_ENABLED

// Verify SDK configuration.
STATIC_ASSERT(PDM_ENABLED);

// Check pin configuration.
STATIC_ASSERT(IS_IO_VALID(CONFIG_IO_PDM_CLK));
STATIC_ASSERT(IS_IO_VALID(CONFIG_IO_PDM_DATA));
#if CONFIG_PDM_MIC_PWR_CTRL_ENABLED
STATIC_ASSERT(IS_IO_VALID(CONFIG_IO_PDM_MIC_PWR_CTRL));
#endif

// Sampling rate depends on PDM configuration.
STATIC_ASSERT(PDM_CONFIG_CLOCK_FREQ == NRF_PDM_FREQ_1032K);
#define SAMPLING_RATE   (1032 * 1000 / 64)

static drv_audio_buffer_handler_t   m_buffer_handler;
static nrf_balloc_t const *         mp_buffer_pool;
static uint8_t                      m_skip_buffers;

ret_code_t drv_audio_enable(void)
{
#if CONFIG_PDM_MIC_PWR_CTRL_ENABLED
#if CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW
    nrf_gpio_pin_clear(CONFIG_IO_PDM_MIC_PWR_CTRL);
#else /* !CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
    nrf_gpio_pin_set(CONFIG_IO_PDM_MIC_PWR_CTRL);
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ENABLED */

    // Skip buffers with invalid data.
    m_skip_buffers = MAX(1, ROUNDED_DIV((CONFIG_PDM_TRANSIENT_STATE_LEN * SAMPLING_RATE),
                                        (1000 * CONFIG_AUDIO_FRAME_SIZE_SAMPLES)));

    return nrfx_pdm_start();
}

ret_code_t drv_audio_disable(void)
{
#if CONFIG_PDM_MIC_PWR_CTRL_ENABLED
#if CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW
    nrf_gpio_pin_set(CONFIG_IO_PDM_MIC_PWR_CTRL);
#else /* !CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
    nrf_gpio_pin_clear(CONFIG_IO_PDM_MIC_PWR_CTRL);
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ENABLED */

    return nrfx_pdm_stop();
}

static void drv_audio_pdm_event_handler(nrfx_pdm_evt_t const *const p_evt)
{
    ret_code_t ret;
    int16_t * p_buffer_released = p_evt->buffer_released;

    if(p_evt->buffer_requested)
    {
        int16_t * p_buffer = nrf_balloc_alloc(mp_buffer_pool);

        if(!p_buffer)
        {
            /*
             * Se tiene que asignar un buffer libre para mantener el PDM activo.
             * Como el pool esta vacio, la unica solucion es usar el buffer que acaba de soltar.
             */
            p_buffer            = p_buffer_released;
            p_buffer_released   = NULL;
        }
        if(p_buffer)
        {
            ret = nrfx_pdm_buffer_set(p_buffer, CONFIG_PDM_BUFFER_SIZE_SAMPLES);
            APP_ERROR_CHECK(ret);
        }
        if(p_buffer_released)
        {
            if (m_skip_buffers)
            {
                m_skip_buffers -= 1;
                nrf_balloc_free(mp_buffer_pool, p_buffer_released);
            }
            else
            {
                m_buffer_handler(p_buffer_released);
            }
        }
    }
}

ret_code_t dvr_audio_init(nrf_balloc_t const * p_buffer_pool, drv_audio_buffer_handler_t buffer_handler)
{
    nrfx_pdm_config_t pdm_cfg = NRFX_PDM_DEFAULT_CONFIG(CONFIG_IO_PDM_CLK,
                                                        CONFIG_IO_PDM_DATA);

    m_buffer_handler    = buffer_handler;
    mp_buffer_pool      = p_buffer_pool;
    pdm_cfg.gain_l      = CONFIG_PDM_GAIN;
    pdm_cfg.gain_r      = CONFIG_PDM_GAIN;

    pdm_cfg.mode        = NRF_PDM_MODE_MONO;

#if   (CONFIG_PDM_MIC == CONFIG_PDM_MIC_LEFT)
    pdm_cfg.edge        = NRF_PDM_EDGE_LEFTFALLING;
#elif (CONFIG_PDM_MIC == CONFIG_PDM_MIC_RIGHT)
    pdm_cfg.edge        = NRF_PDM_EDGE_LEFTRISING;
#else
#error "Value of CONFIG_PDM_MIC is not valid!"
#endif /* (CONFIG_PDM_MIC == CONFIG_PDM_MIC_LEFT) */

#if CONFIG_PDM_MIC_PWR_CTRL_ENABLED
#if CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW
    nrf_gpio_pin_set(CONFIG_IO_PDM_MIC_PWR_CTRL);
#else /* !CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
    nrf_gpio_pin_clear(CONFIG_IO_PDM_MIC_PWR_CTRL);
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW */
    nrf_gpio_cfg_output(CONFIG_IO_PDM_MIC_PWR_CTRL);
#endif /* CONFIG_PDM_MIC_PWR_CTRL_ENABLED */

    return nrfx_pdm_init(&pdm_cfg, drv_audio_pdm_event_handler);
}

#endif /* CONFIG_AUDIO_ENABLED */