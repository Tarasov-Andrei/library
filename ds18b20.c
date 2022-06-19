#include "ds18b20.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

static uint8_t _resolution = 0; // Разрешение (9,10,11,12 бит) на вычисление
static uint8_t _gpio_num = 0;

#define PIN_LO (*(volatile uint32_t *)(DR_REG_GPIO_BASE + 0x0004)) &= ~(BIT(_gpio_num)) // CLEAR_PERI_REG_MASK(GPIO_OUT_REG, BIT(_gpio_num))
#define PIN_HI (*(volatile uint32_t *)(DR_REG_GPIO_BASE + 0x0004)) |= BIT(_gpio_num)    // SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(_gpio_num))
#define PIN_IN READ_PERI_REG(GPIO_IN_REG) & BIT(_gpio_num)

/********************************************************************/
static bool ds18b20_reset(void)
{
    uint16_t status;
    PIN_LO;
    ets_delay_us(480);
    PIN_HI;
    ets_delay_us(65);
    status = PIN_IN;
    ets_delay_us(500);
    return (status ? 1 : 0);
}
/********************************************************************/
static void one_wire_write(uint8_t data)
{
    for (uint8_t p = 8; p; p--)
    {
        PIN_LO;
        if (data & 1)
        {
            ets_delay_us(5);
            PIN_HI;
            ets_delay_us(90);
        }
        else
        {
            ets_delay_us(90);
            PIN_HI;
            ets_delay_us(5);
        }
        data >>= 1;
    }
}
/********************************************************************/
void ds18b20_init(uint8_t gpio_num, uint8_t resolution)
{
    _gpio_num = gpio_num;
    _resolution = resolution;
    gpio_config_t gpio_conf = {
        .pin_bit_mask = BIT(gpio_num),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);

    uint8_t reg = ((resolution - 9) << 5) | 0x1F;
    if (ds18b20_reset())
        return;
    one_wire_write(0xCC);
    one_wire_write(0x4E);
    one_wire_write(0x00);
    one_wire_write(0x00);
    one_wire_write(reg);
}
/********************************************************************/
static uint8_t one_wire_read(void)
{
    uint8_t data = 0;
    for (uint8_t p = 8; p; p--)
    {
        data >>= 1;
        PIN_LO;
        ets_delay_us(2);
        PIN_HI;
        ets_delay_us(8);
        bool data_bit = PIN_IN;
        ets_delay_us(80);
        if (data_bit)
            data |= 0x80;
    }
    return data;
}
/********************************************************************/
void ds18b20_req_temp(void)
{
    if (ds18b20_reset())
        return;
    one_wire_write(0xCC);
    one_wire_write(0x44);
}
/********************************************************************/
float ds18b20_read_temp(void)
{
    uint8_t data[2];
    float temp = 0;

    if (ds18b20_reset())
        return 0;
    one_wire_write(0xCC);
    one_wire_write(0xBE);
    data[0] = one_wire_read();
    data[1] = one_wire_read();
    switch (_resolution)
    {
    case 9:
        temp = (float)((data[1] << 8) | data[0]) * 0.5f;
        break;
    case 10:
        temp = (float)((data[1] << 8) | data[0]) * 0.25f;
        break;
    case 11:
        temp = (float)((data[1] << 8) | data[0]) * 0.125f;
        break;
    case 12:
        temp = (float)((data[1] << 8) | data[0]) * 0.0625f;
        break;
    }
    return temp;
}
/********************************************************************/

/********************************************************************/

/********************************************************************/
