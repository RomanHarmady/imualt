#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp280.h"
#include <math.h>


int main() {

    int32_t tempReading, pressureReading;
    int addr = 0x76; // BMP280 I2C address
    uint8_t data[8];
    int16_t* calib_data;
    int32_t t_fine;
    
    stdio_init_all();
    
    //set i2c0 for BMP280
    i2c_init(i2c0, 100000);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);

    bmp280Config();
    calib_data = bmp280Calib();

    sleep_ms(1000);

    while(true){

    uint8_t data[8];
    i2c_write_blocking(i2c0, addr, "\xF7", 1, true);
    i2c_read_blocking(i2c0, addr, data, 8, false);
    pressureReading = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    tempReading = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    
    int32_t t_fine = bmp280CalcTemp(tempReading, calib_data[0], calib_data[1], calib_data[2]);
    double cTemp = t_fine / 5120.0;    

    int64_t pressure = bmp280CalcPressure(t_fine, pressureReading, calib_data[3], calib_data[4], calib_data[5], calib_data[6], calib_data[7], calib_data[8], calib_data[9], calib_data[10], calib_data[11]);
   
    printf("Pressure : %dPa\n", pressure);
    printf("Temperature in Celsius : %.2f C\n", cTemp);
   
    sleep_ms(5000);
    }
    return 0;
}

