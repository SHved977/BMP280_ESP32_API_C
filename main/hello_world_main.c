/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "bmp280_esp32_api.h"

#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_21


void app_main(void)
{
    esp_err_t err;
    i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = SCL_PIN,
    .sda_io_num = SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    
    err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err){
        printf("Programm has faild in function ***i2c_new_master_bus()*** with error code %s", esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    }

    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x76,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t bmp280_handle;
    err = i2c_master_bus_add_device(bus_handle, &bmp280_cfg, &bmp280_handle);
     if (err){
        printf("Programm has faild in function ***i2c_master_bus_add_device()*** with error code %s", esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    }

    bmp280_set_pressure_oversampling(&bmp280_handle, x16);
    bmp280_set_temperature_oversampling(&bmp280_handle, x2);
    bmp280_set_t_sb(&bmp280_handle, t_sb_0_5);
    bmp280_set_filter(&bmp280_handle, filter_16);
    bmp280_set_mode(&bmp280_handle, normal_mode);

    for (;;) {
        bmp280_measurements_t bmp280_meas = bmp280_read(&bmp280_handle);
        printf("PRESSURE\t=\t%f\n", bmp280_meas.P);
        printf("TEMPERATURE\t=\t%f\n", bmp280_meas.T);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
