#ifndef DS18B20_H_
#define DS18B20_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

/**
 * @brief Инициализация ds18b20
 * @param gpio_num Номер gpio
 * @param resolution Разрешение в битах (9, 10, 11, 12)
 */
void ds18b20_init(uint8_t gpio_num, uint8_t resolution);

/**
 * @brief Запрос на чтение температуры
 */
void ds18b20_req_temp(void);

/**
 * @brief Считать температуру
 * @return Температура в *С
 */
float ds18b20_read_temp();

#endif /* DS18B20_H_ */