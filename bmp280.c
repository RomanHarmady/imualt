#include <math.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

int addr = 0x76;


double calculateAltitude(double pressure, double temperature) {
    // Constants
    const double sea_level_pressure = 1013.25; // hPa
    const double temperature_lapse_rate = 0.0065; // K/m
    const double standard_temperature = 288.15; // K
    const double gravity = 9.80665; // m/s²
    const double molar_mass = 0.0289644; // kg/mol
    const double gas_constant = 8.31447; // J/(mol·K)

    double altitude = -((gas_constant * standard_temperature) / (gravity * molar_mass)) *
                      log(pressure / sea_level_pressure) *
                      (1 + ((temperature_lapse_rate * altitude) / standard_temperature));
    
    return altitude;
}



int16_t* bmp280Calib(){

    uint8_t calib_raw_data[24];
    int16_t data[12];
    
    i2c_write_blocking(i2c0, addr, "\x88", 1, true);
    i2c_read_blocking(i2c0, addr, calib_raw_data, 24, false);

    data[0] = (calib_raw_data[1] << 8) | calib_raw_data[0];
    data[1] = (calib_raw_data[3] << 8) | calib_raw_data[2];
    data[2] = (calib_raw_data[5] << 8) | calib_raw_data[4];
    data[3] = (calib_raw_data[7] << 8) | calib_raw_data[6];
    data[4]= (calib_raw_data[9] << 8) | calib_raw_data[8];
    data[5] = (calib_raw_data[11] << 8) | calib_raw_data[10];
    data[6] = (calib_raw_data[13] << 8) | calib_raw_data[12];
    data[7] = (calib_raw_data[15] << 8) | calib_raw_data[14];
    data[8] = (calib_raw_data[17] << 8) | calib_raw_data[16];
    data[9] = (calib_raw_data[19] << 8) | calib_raw_data[18];
    data[10] = (calib_raw_data[21] << 8) | calib_raw_data[20];
    data[11] = (calib_raw_data[23] << 8) | calib_raw_data[22];

    return data;
}

int32_t bmp280CalcTemp(int32_t adc_T, int16_t dig_T1, int16_t dig_T2, int16_t dig_T3){

    int32_t var1t = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    int32_t var2t = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    int32_t t_fine =(var1t + var2t);
    return t_fine;
}

int64_t bmp280CalcPressure(int32_t t_fine, int32_t adc_P, int16_t dig_P1, int16_t dig_P2, int16_t dig_P3, int16_t dig_P4, int16_t dig_P5, int16_t dig_P6, int16_t dig_P7, int16_t dig_P8, int16_t dig_P9){

    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var1 == 0)
    {
    return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);

   int64_t pressure = (p/256);


    return pressure;
}

void bmp280Config(){

    uint8_t config[2] = {0xF4, 0x77};
    i2c_write_blocking(i2c0, addr, config, 2, false);
    config[0] = 0xF5;
    config[1] = 0xE0;
    i2c_write_blocking(i2c0, addr, config, 2, false);

}