
#pragma once

#include <stdint.h>
#include "esp_system.h"

#define BMP280_OSRS_P x16
#define BMP280_OSRS_T x2
#define BMP280_MODE normal_mode

typedef enum {
    t_sb_0_5,
    t_sb_62_5,
    t_sb_125,
    t_sb_250,
    t_sb_500,
    t_sb_1000,
    t_sb_2000,
    t_sb_4000
} bmp280_tsb_t;

typedef enum {
    filter_off,
    filter_2,
    filter_4,
    filter_8,
    filter_16
} bmp280_filter_t;

typedef enum {
    skipped,
    x1,
    x2,
    x4,
    x8,
    x16
} bmp280_osrs_t;

typedef enum {
    sleep_mode,
    forced_mode,
    normal_mode = 3
} bmp280_mode_t;

typedef struct {
    uint16_t    T1;
    int16_t     T2;
    int16_t     T3;
    uint16_t    P1;
    int16_t     P2;
    int16_t     P3;
    int16_t     P4;
    int16_t     P5;
    int16_t     P6;
    int16_t     P7;
    int16_t     P8;
    int16_t     P9;
} bmp280_compensation_parameters_t;

typedef struct {
    double T;
    double P;
} bmp280_measurements_t;

esp_err_t bmp280_set_pressure_oversampling(const i2c_master_dev_handle_t* bmp280_handle, bmp280_osrs_t osrs_p);

esp_err_t bmp280_set_temperature_oversampling(const i2c_master_dev_handle_t* bmp280_handle, bmp280_osrs_t osrs_t);

esp_err_t bmp280_set_mode(const i2c_master_dev_handle_t* bmp280_handle, bmp280_mode_t mode);

esp_err_t bmp280_set_t_sb(const i2c_master_dev_handle_t* bmp280_handle, bmp280_tsb_t t_sb);

esp_err_t bmp280_set_filter(const i2c_master_dev_handle_t* bmp280_handle, bmp280_filter_t filter);

bmp280_measurements_t bmp280_read(const i2c_master_dev_handle_t * bmp280_handle);