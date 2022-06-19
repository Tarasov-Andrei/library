#include "encoder.h"
#include "driver/timer.h"

#define BTN_DEBOUNCE (50 * 1000)    // Антидребезг кнопки, мкс
#define BTN_LONG_PRESS (600 * 1000) // Длител. нажатие кнопки, мкс

volatile static uint8_t enc_state = 0;
volatile static bool sw_intr_state = 0; // Статус прерывания
static int16_t cnt_val = 0;
volatile static uint8_t sw_flag = 0;
volatile static bool sw_long = 0;
uint64_t curr_time = 0;
uint64_t prev_time = 0;
uint8_t btn_pin;

/*********************************************************/
static void isr_event(void *pvParameters)
{
    pcnt_intr_disable(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_0, &cnt_val);
    if (cnt_val == 1)
    {
        enc_state = 2;
        bool enc_r();
    }
    else if (cnt_val == -1)
    {
        enc_state = 1;
        bool enc_l();
    }
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_0);
}
/*********************************************************/
static void isr_sw(void *pvParameters)
{
    sw_intr_state = 1;
    gpio_intr_disable(btn_pin);
    if (gpio_get_level(btn_pin) == 1 && sw_long == 0)
    {
        sw_flag = 1;
    }
    if (gpio_get_level(btn_pin) == 0 && sw_long == 0)
    {
        sw_flag = 2;
        bool enc_click();
    }
    else if (gpio_get_level(btn_pin) == 0 && sw_long == 1)
    {
        sw_long = 0;
        sw_flag = 0;
    }
}
/*********************************************************/
bool enc_l(void)
{
    if (enc_state == 1)
    {
        enc_state = 0;
        return true;
    }
    else
        return false;
}
/*********************************************************/
bool enc_r(void)
{
    if (enc_state == 2)
    {
        enc_state = 0;
        return true;
    }
    else
        return false;
}
/*********************************************************/
bool enc_click(void)
{
    if (sw_flag == 2 && sw_long == 0)
    {
        sw_flag = 0;
        return true;
    }
    else
        return false;
}
/*********************************************************/
bool enc_long_press(void)
{
    if (sw_flag == 1 && (curr_time - prev_time) >= BTN_LONG_PRESS)
    {
        sw_flag = 0;
        sw_long = 1;
        prev_time = curr_time;
        return true;
    }
    else
        return false;
}
/*********************************************************/
void encoder(void)
{
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &curr_time);
    if (sw_intr_state == 1 && (curr_time - prev_time) >= BTN_DEBOUNCE)
    {
        gpio_intr_enable(btn_pin); // Разрешить прерывания
        sw_intr_state = 0;
        prev_time = curr_time;
    }
}
/*********************************************************/
void encoder_init(uint8_t clk, uint8_t dt, uint8_t sw)
{
    btn_pin = sw;

    gpio_set_direction(btn_pin, GPIO_MODE_INPUT);
    gpio_pullup_dis(btn_pin);
    gpio_pulldown_dis(btn_pin);
    gpio_set_intr_type(btn_pin, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(sw, isr_sw, NULL); // Добавить обработчик прерываний кнопки энкодера

    pcnt_config_t config_pcnt;
    config_pcnt.pulse_gpio_num = dt;
    config_pcnt.ctrl_gpio_num = clk;
    config_pcnt.pos_mode = PCNT_COUNT_INC;      // При восход. фронте на pulse_gpio_num - инкремент
    config_pcnt.neg_mode = PCNT_COUNT_DIS;      // При нисход. фронте на pulse_gpio_num - не изменять
    config_pcnt.lctrl_mode = PCNT_MODE_REVERSE; // При low на ctrl_gpio_num - инвертировать счетчик
    config_pcnt.hctrl_mode = PCNT_MODE_KEEP;    //  При high на ctrl_gpio_num сохранить текущий режим
    config_pcnt.counter_h_lim = 10;             // Макс. значение счетчика
    config_pcnt.counter_l_lim = -10;            // Мин. значение счетчика
    config_pcnt.unit = PCNT_UNIT_0;
    config_pcnt.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&config_pcnt);

    pcnt_set_filter_value(PCNT_UNIT_0, 1023);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_0, -1); // Уст. значения на прерывание (-1)
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0);

    pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_1, 1); // Уст. значения на прерывание (1)
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, isr_event, NULL); // Добавить обработчик прерываний энкодера

    pcnt_counter_resume(PCNT_UNIT_0);
}
