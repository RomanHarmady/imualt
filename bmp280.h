#ifndef BMP280_H
#define BMP280_H





double calculateAltitude(double pressure, double temperature);
void bmp280Config();
int16_t bmp280Calib();
int32_t bmp280CalcTemp(int32_t pressureReading, int16_t dig_T1, int16_t dig_T2, int16_t dig_T3);
int64_t bmp280CalcPressure(int32_t t_fine, int32_t adc_P, int16_t dig_P1, int16_t dig_P2, int16_t dig_P3, int16_t dig_P4, int16_t dig_P5, int16_t dig_P6, int16_t dig_P7, int16_t dig_P8, int16_t dig_P9);
//int32_t calcTemp(int32_t adc_T);


#endif