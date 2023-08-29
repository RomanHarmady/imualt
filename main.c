#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

// random comment

double calculate_altitude(double pressure, double temperature) {
    // Constants
    const double sea_level_pressure = 1013.25; // hPa
    const double temperature_lapse_rate = 0.0065; // K/m
    const double standard_temperature = 288.15; // K
    const double gravity = 9.80665; // m/s²
    const double molar_mass = 0.0289644; // kg/mol
    const double gas_constant = 8.31447; // J/(mol·K)

    // Calculate altitude
    double altitude = -((gas_constant * standard_temperature) / (gravity * molar_mass)) *
                      log(pressure / sea_level_pressure) *
                      (1 + ((temperature_lapse_rate * altitude) / standard_temperature));
    
    return altitude;
}


int main() {
    stdio_init_all();

    i2c_init(i2c0, 100000);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);

    int addr = 0x76; // BMP280 I2C address

    uint8_t calib_data[24];
    uint16_t dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t adc_T, adc_P;

    // Open the I2C bus
    i2c_write_blocking(i2c0, addr, "\x88", 1, true);
    i2c_read_blocking(i2c0, addr, calib_data, 24, false);

    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];

    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];

    // Select control measurement register(0xF4)
    uint8_t config[2] = {0xF4, 0x77}; //0x27
    i2c_write_blocking(i2c0, addr, config, 2, false);

    // Select config register(0xF5)
    config[0] = 0xF5;
    config[1] = 0xE0;
    i2c_write_blocking(i2c0, addr, config, 2, false);
    sleep_ms(1000);
    while(true){
    uint8_t data[8];
    // Read 8 bytes of data from register(0xF7)
    i2c_write_blocking(i2c0, addr, "\xF7", 1, true);
    i2c_read_blocking(i2c0, addr, data, 8, false);

    adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // chaged 
    int32_t var1t = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    int32_t var2t = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    int32_t t_fine =(var1t + var2t);
    double cTemp = (var1t + var2t) / 5120.0;    

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
   int pres = pressure;

    //double altitude = calculate_altitude(pressure, cTemp);

    printf("Pressure : %lld Pa\n", pressure);
    printf("Pres : %d Pa\n", pres);
    printf("Temperature in Celsius : %.2f C\n", cTemp);
    //printf("Altitude: %.2f meters\n", altitude);

    sleep_ms(500);
    }
    return 0;
}

