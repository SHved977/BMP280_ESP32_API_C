#pragma once

#include <stdint.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "bmp280_esp32_api.h"

esp_err_t bmp280_set_pressure_oversampling(const i2c_master_dev_handle_t* bmp280_handle, bmp280_osrs_t osrs_p){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_pressure_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~(7 << 2);
    ctrl_means |= (osrs_p << 2);
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(*bmp280_handle, ctrl_means_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_pressure_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
}      

esp_err_t bmp280_set_temperature_oversampling(const i2c_master_dev_handle_t* bmp280_handle, bmp280_osrs_t osrs_t){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_temperature_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~(7 << 5);
    ctrl_means |= (osrs_t << 5);
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(*bmp280_handle, ctrl_means_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_temperature_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

esp_err_t bmp280_set_mode(const i2c_master_dev_handle_t* bmp280_handle, bmp280_mode_t mode){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_mode()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~3;
    ctrl_means |= mode;
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(*bmp280_handle, ctrl_means_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_mode()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

esp_err_t bmp280_set_t_sb(const i2c_master_dev_handle_t* bmp280_handle, bmp280_tsb_t t_sb){
    uint8_t config;
    uint8_t config_adr = 0xF5;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &config_adr, 1, &config, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_t_sb()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    config &= ~(7 << 5);
    config |= (t_sb << 5);
    uint8_t config_write[2] = {config_adr, config};
    error_code = i2c_master_transmit(*bmp280_handle, config_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_t_sb()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
}  

esp_err_t bmp280_set_filter(const i2c_master_dev_handle_t* bmp280_handle, bmp280_filter_t filter){
    uint8_t config;
    uint8_t config_adr = 0xF5;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &config_adr, 1, &config, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_filter()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    config &= ~(7 << 2);
    config |= (filter << 2);
    uint8_t config_write[2] = {config_adr, config};
    error_code = i2c_master_transmit(*bmp280_handle, config_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_filter()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

static esp_err_t bmp280_read_compensation_parameters(const i2c_master_dev_handle_t * bmp280_handle, bmp280_compensation_parameters_t * cp){ // cp - compensation_parameters
    uint8_t parameters_adr = 0x88;
    uint8_t buf[24];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &parameters_adr, 1, buf, 24, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_compensation_parameters()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    cp->T1 = (uint16_t)buf[0 ] | (uint16_t)buf[1 ] << 8;
    cp->T2 = (int16_t )buf[2 ] | (int16_t )buf[3 ] << 8;
    cp->T3 = (int16_t )buf[4 ] | (int16_t )buf[5 ] << 8;
    cp->P1 = (uint16_t)buf[6 ] | (uint16_t)buf[7 ] << 8;
    cp->P2 = (int16_t )buf[8 ] | (int16_t )buf[9 ] << 8;
    cp->P3 = (int16_t )buf[10] | (int16_t )buf[11] << 8;
    cp->P4 = (int16_t )buf[12] | (int16_t )buf[13] << 8;
    cp->P5 = (int16_t )buf[14] | (int16_t )buf[15] << 8;
    cp->P6 = (int16_t )buf[16] | (int16_t )buf[17] << 8;
    cp->P7 = (int16_t )buf[18] | (int16_t )buf[19] << 8;
    cp->P8 = (int16_t )buf[20] | (int16_t )buf[21] << 8;
    cp->P9 = (int16_t )buf[22] | (int16_t )buf[23] << 8;
    return ESP_OK;
}

static int32_t bmp280_read_raw_pressure(const i2c_master_dev_handle_t * bmp280_handle){
    uint8_t pressure_adr = 0xF7;
    uint8_t buf[3];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &pressure_adr, 1, buf, 3, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_raw_pressure()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    int32_t raw_pressure   =  (int32_t)buf[0] << 12 | (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4;
    return raw_pressure;    
}

static int32_t bmp280_read_raw_temperature(const i2c_master_dev_handle_t * bmp280_handle){
    uint8_t temperature_adr = 0xFA;
    uint8_t buf[3];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(*bmp280_handle, &temperature_adr, 1, buf, 3, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_raw_temperature()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    int32_t raw_temperature = (int32_t)buf[0] << 12 | (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4;
    return raw_temperature;
}

bmp280_measurements_t bmp280_read(const i2c_master_dev_handle_t * bmp280_handle){
    int32_t T_raw = bmp280_read_raw_temperature(bmp280_handle);
    int32_t P_raw = bmp280_read_raw_pressure(bmp280_handle);
    bmp280_compensation_parameters_t cp;
    bmp280_read_compensation_parameters(bmp280_handle, &cp);
    double var1, var2, var3, var4;
    bmp280_measurements_t meas;
    
    int32_t t_fine;
    var1 = ((double)T_raw/16384.0 - (double)cp.T1/1024.0) * (double)cp.T2;
    var2 = ((((double)T_raw)/131072.0 - ((double)cp.T1)/8192.0) * (((double)T_raw)/131072.0 - ((double)cp.T1)/8192.0)) * ((double)cp.T3);
    t_fine = (int32_t)(var1 + var2);
    meas.T = (var1 + var2) / 5120.0;

    var3 = (double)t_fine/2.0 - 64000.0;
    var4 = var3 * var3 * (double)cp.P6/32768.0;
    var4 = var4 + var3 * (double)cp.P5*2.0;
    var4 = var4/4.0 + (double)cp.P4*65536.0;
    var3 = ((double)cp.P3*var3*var3/524288.0 + (double)cp.P2*var3) / 524288.0;
    var3 = (1.0 + var3/32768.0)*(double)cp.P1;
    if (var3 == 0.0){
        meas.P = 0.0;
        return meas;
    }
    meas.P = 1048576.0 - (double)P_raw;
    meas.P = (meas.P - var4/4096.0) * 6250.0/var3;
    var3 = (double)cp.P9 * meas.P * meas.P / 2147483648.0;
    var4 = meas.P * (double)cp.P8 / 32768.0;
    meas.P = meas.P + (var3 + var4 + (double)cp.P7) / 16.0;
    return meas;
}

