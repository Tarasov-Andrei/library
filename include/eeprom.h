#ifndef EEPROM_H
#define EEPROM_H

#include <stdio.h>
#include "stdint.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

/**
 * @brief Инициализация устройства
 * @param i2c_port Номер порта I2C. (I2C_NUM_0 или I2C_NUM_1)
 */
void eeprom_init(i2c_port_t i2c_port);

/**
 * @brief Записать byte в ячейку EEPROM
 * @param address Адрес ячейки
 * @param value Значение (данные)
 */
void eeprom_write_byte(uint8_t address, uint8_t value);

/**
 * @brief Записать short в ячейку EEPROM
 * @param address Адрес ячейки
 * @param value Значение (данные)
 */
void eeprom_write_short(uint8_t address, uint16_t value);

/**
 * @brief Считать byte из ячейки EEPROM
 * @param address Адрес ячейки
 *
 * @return Значение (данные)
 */
uint8_t eeprom_read_byte(uint8_t address);

/**
 * @brief Считать short из ячейки EEPROM
 * @param address Адрес ячейки
 *
 * @return Значение (данные)
 */
uint16_t eeprom_read_short(uint8_t address);

#endif // EEPROM_H