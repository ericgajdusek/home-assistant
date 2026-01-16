#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_check.h"
#include "es8311.h"
#include "example_config.h"

static const char *TAG = "i2s_es8311";
// static const char err_reason[][30] = {"input param is invalid",
//                                       "operation timeout"
//                                      };
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

/* Import music file as buffer */
#if CONFIG_EXAMPLE_MODE_MUSIC
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");
#endif

static void gpio_init(void)
{
    // 配置GPIO48为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PA), // 选择GPIO48
        .mode = GPIO_MODE_OUTPUT,                  // 配置为输出模式
        .pull_down_en = GPIO_PULLDOWN_DISABLE,     // 禁用下拉
        .pull_up_en = GPIO_PULLUP_DISABLE,         // 禁用上拉
        .intr_type = GPIO_INTR_DISABLE             // 禁用中断
    };
    gpio_config(&io_conf);

    // 设置GPIO48为高电平
    gpio_set_level(GPIO_OUTPUT_PA, 1);
}

static esp_err_t es8311_codec_init(void)
{
    /* Initialize I2C peripheral */
#if !defined(CONFIG_EXAMPLE_BSP)
    const i2c_config_t es_i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_NUM, &es_i2c_cfg), TAG, "config i2c failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0), TAG, "install i2c driver failed");
#else
    ESP_ERROR_CHECK(bsp_i2c_init());
#endif

    /* Initialize es8311 codec */
    es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
    ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
        .sample_frequency = EXAMPLE_SAMPLE_RATE
    };

    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
#if CONFIG_EXAMPLE_MODE_ECHO
    ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
#endif
    return ESP_OK;
}

static esp_err_t i2s_driver_init(void)
{
#if !defined(CONFIG_EXAMPLE_BSP)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
#else
    ESP_LOGI(TAG, "Using BSP for HW configuration");
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;
    ESP_ERROR_CHECK(bsp_audio_init(&std_cfg, &tx_handle, &rx_handle));
    ESP_ERROR_CHECK(bsp_audio_poweramp_enable(true));
#endif
    return ESP_OK;
}


static esp_codec_dev_sample_info_t make_sample_info(void)
{
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel     = 2,     // stereo
        .bits_per_sample = 16,
    };
    return fs;
}

static void codec_echo_task(void *arg)
{
    esp_err_t ret;

    // 1) Init audio subsystem (BSP sets I2S + power amp correctly)
    // Passing NULL uses BSP defaults on some boards; but we want 16k/16bit/stereo
    // If your BSP ignores this, it still usually configures the codec dev with a compatible format.
    i2s_std_config_t i2s_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,  // comes from BSP if available
    };
    i2s_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; // common; adjust if your BSP expects something else

    ret = bsp_audio_init(&i2s_cfg);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "bsp_audio_init failed: %s", esp_err_to_name(ret));

    // Some BSPs need explicit amp enable (some do this inside bsp_audio_init)
    // If your BSP has this API, keep it. If not, delete this line.
    (void)bsp_audio_poweramp_enable(true);

    // 2) Create codec device handles
    esp_codec_dev_handle_t spk = bsp_audio_codec_speaker_init();
    ESP_GOTO_ON_FALSE(spk, ESP_FAIL, fail, TAG, "speaker codec init returned NULL");

    esp_codec_dev_handle_t mic = bsp_audio_codec_microphone_init();
    ESP_GOTO_ON_FALSE(mic, ESP_FAIL, fail, TAG, "microphone codec init returned NULL");

    // 3) Open both devices with the same audio format
    esp_codec_dev_sample_info_t fs = make_sample_info();

    ret = esp_codec_dev_open(mic, &fs);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "esp_codec_dev_open(mic) failed: %s", esp_err_to_name(ret));

    ret = esp_codec_dev_open(spk, &fs);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "esp_codec_dev_open(spk) failed: %s", esp_err_to_name(ret));

    // 4) Set volumes/gain (optional but helpful)
    // Speaker volume is usually 0-100
    (void)esp_codec_dev_set_out_vol(spk, 70);

    // Mic gain API varies; if your version has set_in_gain / set_in_vol, use it.
    // If this doesn’t compile, tell me the function names available in your esp_codec_dev.h.
    // (void)esp_codec_dev_set_in_gain(mic, 20);

    // 5) Echo loop
    uint8_t *buf = (uint8_t *)malloc(ECHO_BUF_BYTES);
    ESP_GOTO_ON_FALSE(buf, ESP_ERR_NO_MEM, fail, TAG, "malloc failed");

    ESP_LOGI(TAG, "Echo started (codec-dev). buf=%d bytes", ECHO_BUF_BYTES);

    while (1) {
        int bytes_read = 0;
        int bytes_written = 0;

        // Read from mic
        ret = esp_codec_dev_read(mic, buf, ECHO_BUF_BYTES, &bytes_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "mic read failed: %s", esp_err_to_name(ret));
            break;
        }
        if (bytes_read <= 0) {
            // no data yet; yield a bit
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Write to speaker (write exactly what we read)
        ret = esp_codec_dev_write(spk, buf, bytes_read, &bytes_written);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spk write failed: %s", esp_err_to_name(ret));
            break;
        }

        // Optional: debug
        // ESP_LOGI(TAG, "read=%d wrote=%d", bytes_read, bytes_written);
    }

    free(buf);

    // 6) Close devices
    (void)esp_codec_dev_close(mic);
    (void)esp_codec_dev_close(spk);

    ESP_LOGW(TAG, "Echo stopped");

fail:
    // If something failed early, just stop the task
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_init();
    printf("i2s es8311 codec example start\n-----------------------------\n");
    /* Initialize i2s peripheral */
    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "i2s driver init success");
    }
    

    /* Initialize i2c peripheral and config es8311 codec by i2c */
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "es8311 codec init success");
    }
#if CONFIG_EXAMPLE_MODE_MUSIC
    /* Play a piece of music in music mode */
    xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
#else
    /* Echo the sound from MIC in echo mode */
    xTaskCreate(codec_echo_task, "codec_echo", 8192, NULL, 5, NULL);
#endif
}
