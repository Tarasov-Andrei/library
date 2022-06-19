#ifndef DS3231_H
#define DS3231_H

#include <stdio.h>
#include "stdint.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t days_of_week;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} rtc_t;
/**
 * @brief Инициализация устройства
 * @param i2c_port Номер порта I2C. (I2C_NUM_0 или I2C_NUM_1)
 */
void ds3231_init(i2c_port_t i2c_port);

/**
 * @brief Считать температуру
 * @return Температура в *С
 */
float ds3231_read_temp(void);

/**
 * @brief Установить дату/время
 * @param *rtc Указатель на структуру с датой/временем
 * @return ESP_OK при успешной установки
 */
esp_err_t ds3231_set_time(rtc_t *rtc);

/**
 * @brief Получить дату/время
 * @param *rtc Указатель на структуру с датой/временем
 * @return ESP_OK при успешном получении
 */
esp_err_t ds3231_get_time(rtc_t *rtc);

#endif // DS3231_H