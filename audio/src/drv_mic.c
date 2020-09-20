#pragma GCC optimize ("O0")

#include "drv_mic.h"
#include "drv_audio.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "g711.h"
#include "app_scheduler.h"


typedef struct
{
    int16_t  buf[CONFIG_AUDIO_FRAME_SIZE_BYTES];
    uint16_t samples;
    bool     free;
}pdm_buf_t;

#define PDM_BUF_NUM     6

static bool                   m_audio_enabled;          ///< Audio enabled flag.
static drv_mic_data_handler_t m_data_handler;
static pdm_buf_t              m_pdm_buf[PDM_BUF_NUM];


static void mic_power_on(void)
{
    
    nrf_gpio_cfg_input(MIC_DOUT, NRF_GPIO_PIN_NOPULL);
    // Prendo el microfono
    nrf_gpio_pin_set(MIC_PWR_CTRL);
}


static void mic_power_off(void)
{
    nrf_gpio_pin_clear(MIC_PWR_CTRL);
    nrf_gpio_cfg_input(MIC_DOUT, NRF_GPIO_PIN_PULLDOWN);
}


static void m_audio_process(void * p_event_data, uint16_t event_size)
{
    ret_code_t        status;
    m_audio_frame_t   frame_buf;
    pdm_buf_t       * p_pdm_buf = (pdm_buf_t *)(*(uint32_t *)p_event_data);
    int16_t         * p_buffer = p_pdm_buf->buf;

    APP_ERROR_CHECK_BOOL(p_event_data != NULL);
    APP_ERROR_CHECK_BOOL(event_size > 0);

    uint8_t nested;
    app_util_critical_region_enter(&nested);
    for(uint16_t i = 0; i < CONFIG_AUDIO_FRAME_SIZE_BYTES; i++)
    {
        frame_buf.data[i] = linear2ulaw(*(p_buffer++));
    }
    frame_buf.data_size = CONFIG_AUDIO_FRAME_SIZE_BYTES;
    p_pdm_buf->free = true;
    app_util_critical_region_exit(nested);

    // Schedule audio transmission. It cannot be done from this context.
    status = m_data_handler(&frame_buf);
    if (status != NRF_SUCCESS)
    {
        // Cannot schedule audio frame transmission!
        /*
         * Do not clear CONFIG_IO_DBG_PCM. This will make debugging pulse wider
         * than expected and easier to spot on the logic analyzer.
         */
    }
}


static void m_audio_buffer_handler(int16_t * p_buffer)
{
    uint32_t     err_code;
    pdm_buf_t  * p_pdm_buf = NULL;
    uint32_t     pdm_buf_addr;
    uint16_t samples = CONFIG_AUDIO_FRAME_SIZE_BYTES;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        if ( m_pdm_buf[i].free == true )
        {
            m_pdm_buf[i].free    = false;
            m_pdm_buf[i].samples = samples;

            for (uint32_t j = 0; j < samples; j++)
            {
                m_pdm_buf[i].buf[j] = p_buffer[j];
            }

            p_pdm_buf = &m_pdm_buf[i];
            pdm_buf_addr = (uint32_t)&m_pdm_buf[i];

            break;
        }
    }

    if (p_pdm_buf != NULL)
    {
        err_code = app_sched_event_put(&pdm_buf_addr, sizeof(pdm_buf_t *), m_audio_process);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // m_audio_buffer_handler: BUFFER FULL!!\r\n
    }
}


uint32_t drv_mic_start(void)
{
    ret_code_t status;

    if(m_audio_enabled == true)
    {
        return NRF_SUCCESS;
    }

    mic_power_on();

    status = drv_audio_enable();
    if (status == NRF_SUCCESS)
    {
        m_audio_enabled = true;
    }

    return status;
}


 uint32_t drv_mic_stop(void)
{
    ret_code_t status;

    if(m_audio_enabled == false)
    {
        return NRF_SUCCESS;
    }

    status = drv_audio_disable();
    if (status == NRF_SUCCESS)
    {
        m_audio_enabled = false;
    }

    mic_power_off();

    return status;
}


uint32_t drv_mic_init(drv_mic_data_handler_t data_handler)
{
    m_audio_enabled = false;
    m_data_handler  = data_handler;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        m_pdm_buf[i].free = true;
    }

    nrf_gpio_cfg_output(MIC_PWR_CTRL);

    mic_power_off();

    return drv_audio_init(m_audio_buffer_handler);
}