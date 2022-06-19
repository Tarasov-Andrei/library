#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include "stdint.h"
#include "stdbool.h"
#include "driver/pcnt.h"

/**
 * @brief Инициализация Энкодера
 * @param clk Номер GPIO энкодера
 * @param dt Номер GPIO энкодера
 * @param sw Номер GPIO кнопки энкодера
 */
void encoder_init(uint8_t clk, uint8_t dt, uint8_t sw);

/**
 *  @brief Контроль энкодера
 */
void encoder(void);

/**
 * @brief Поворот энкодера влево
 * @return 1 при повороте,
 *         0 при отсутствии
 */
bool enc_l(void);

/**
 * @brief Поворот энкодера вправо
 * @return 1 при повороте,
 *         0 при отсутствии
 */
bool enc_r(void);

/**
 * @brief Клик кнопки энкодера
 * @return 1 при клике,
 *         0 при отсутствии
 */
bool enc_click(void);

/**
 *  @brief Длительное нажатие энкодера
 */
bool enc_long_press(void);

#endif // ENCODER_H