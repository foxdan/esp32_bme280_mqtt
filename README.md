# esp32_bme280_mqtt
Simple ESP-IDF based application to read BME280 temperature; pressure; and
humidity data, and write the data to a MQTT broker. Writes seonsor data to
dedicated topics and to InfluxDB. Optionally print temperature data to attached
I2C display.

## QuickStart
  - `git clone git@github.com:foxdan/esp32_bme280_mqtt.git --recurse-submodules`
  - `idf.py menuconfig`
    - Set up WiFi
    - Set up I2C config
    - Enable/Disable SSD1306
    - Set up MQTT
  - `idf.py build`
  - `idf.py flash`

## WiFi
Uses example app WiFi component.

## I2C
Assumes a single bus used for the BME280 and SSD1306 (if present).

## MQTT
Writes BME280 sensor data to MQTT topics based on configured _location_ and
_room_. Also writes InfluxDB line protocol messages.

### Topics
  - `$location/$room/sensor` - (e.g. `home/kitchen/sensor`)
    - `$location/$room/sensor/temperature`
    - `$location/$room/sensor/pressure`
    - `$location/$room/sensor/humidity`
  - `telegraf` - Line protocol message tagged with _location_ and _room_, value
                 fields: temperature;pressure;humidity.

## SSD1306
If connected and enabled dump current temperature data to the display.
