/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"
#include "bme280.h"
#include "ssd1306_default_if.h"
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#define MQTT_SENSOR_TOPIC CONFIG_MQTT_TOPIC_LOCATION "/" \
                          CONFIG_MQTT_TOPIC_ROOM "/sensor/"
#define PRINTF_TELEGRAF_LINE "sensor,location=" CONFIG_MQTT_TOPIC_LOCATION \
                             ",room=" CONFIG_MQTT_TOPIC_ROOM \
                             " temperature=%0.2f,humidity=%0.2f,pressure=%0.2f"

void user_delay_ms(uint32_t period)
{
    //printf("Waiting: %d\r\n", period);
    vTaskDelay((period/portTICK_PERIOD_MS)+1);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    // Start read cmd
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    // Write register we want to start the read from
    i2c_master_write_byte(cmd, reg_addr, true);
    // Start actual reading cmd
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, (size_t)len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data+len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    // Execute cmd
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        rslt = -1;
    }
    return rslt;
}


int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    // Start write cmd
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    // Write addr we want to start the write to
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, (size_t)len, true);
    i2c_master_stop(cmd);
    // Execute cmd
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        rslt = -1;
    }
    return rslt;
}

void print_sensor_data(struct bme280_data *comp_data) {
    printf("%0.2f, %.0f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
}

#ifdef CONFIG_SSD1306_ENABLE
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev, esp_mqtt_client_handle_t client, struct SSD1306_Device *lcd)
#else
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev, esp_mqtt_client_handle_t client)
#endif
{
    int8_t rslt;
    uint8_t settings_sel;
	uint32_t req_delay;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_1X;
    dev->settings.osr_t = BME280_OVERSAMPLING_1X;
    dev->settings.filter = BME280_FILTER_COEFF_OFF;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);

	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev->settings);

    printf("Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    bool read_once = false;
    TickType_t ticks = xTaskGetTickCount();
    while (1) {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_ms(req_delay);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        // First result from sensor is highly inaccurate
        if (read_once == false) {
            read_once = true;
            user_delay_ms(1000);
            continue;
        }
        char mqtt_data_buf[20];
        sprintf(mqtt_data_buf, "%0.2f", comp_data.temperature);
        esp_mqtt_client_publish(client, MQTT_SENSOR_TOPIC "/temperature", mqtt_data_buf, 0, 0, false);
        sprintf(mqtt_data_buf, "%0.0f", comp_data.pressure);
        esp_mqtt_client_publish(client, MQTT_SENSOR_TOPIC "/pressure", mqtt_data_buf, 0, 0, false);
        sprintf(mqtt_data_buf, "%0.2f", comp_data.humidity);
        esp_mqtt_client_publish(client, MQTT_SENSOR_TOPIC "/humidity", mqtt_data_buf, 0, 0, false);
        char telegraf[100];
        sprintf(telegraf, PRINTF_TELEGRAF_LINE,
                          comp_data.temperature,
                          comp_data.humidity,
                          comp_data.pressure);
        esp_mqtt_client_publish(client, "telegraf", telegraf, 0, 0, false);
#ifdef CONFIG_SSD1306_ENABLE
        char lcd_str[50];
        SSD1306_Clear(lcd, 0);
        sprintf(lcd_str, "%5.2f", comp_data.temperature);
        SSD1306_FontDrawString(lcd, 0, 0, lcd_str, 1);
        SSD1306_Update(lcd);
        print_sensor_data(&comp_data);
#endif
        // 30s sleep
        vTaskDelayUntil(&ticks, 30 * 1000 / portTICK_PERIOD_MS);
    }
    return rslt;
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY
    };

    i2c_param_config(CONFIG_I2C_MASTER_PORT_NUM, &i2c_cfg);
    i2c_driver_install(CONFIG_I2C_MASTER_PORT_NUM, i2c_cfg.mode, 0, 0, 0);

    bme280_init(&dev);
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_MQTT_HOST,
        .client_id = CONFIG_MQTT_CLIENT_ID,
        // .user_context = (void *)your_context
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

#ifdef CONFIG_SSD1306_ENABLE
    struct SSD1306_Device lcd;
    SSD1306_I2CMasterAttachDisplayDefault(&lcd, 128, 32, 0x3c, -1);
    SSD1306_Rotate180(&lcd);
    SSD1306_SetFont(&lcd, &Font_droid_sans_mono_16x31);

    stream_sensor_data_forced_mode(&dev, client, &lcd);
#else
    stream_sensor_data_forced_mode(&dev, client);
#endif

}
