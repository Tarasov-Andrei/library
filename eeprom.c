#include "eeprom.h"

#define EEPROM_ADDRESS (0x50 << 1) // A0=A1=A2=GND

#define ACK_CHECK_EN 0x1  // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0 // I2C master will not check ack from slave
#define ACK_VAL 0x0       // I2C ack value
#define NACK_VAL 0x1      // I2C nack value

static i2c_port_t EEPROM_PORT = 0;

/********************************************************************/
uint8_t eeprom_read_byte(uint8_t address)
{
    uint8_t value = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDRESS | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDRESS | I2C_MASTER_READ, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, &value, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(EEPROM_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return value;
}
/********************************************************************/
uint16_t eeprom_read_short(uint8_t address)
{
    uint8_t HB = eeprom_read_byte(address);
    uint8_t LB = eeprom_read_byte(address + 1);
    uint16_t value = ((uint16_t)HB << 8) | LB;

    return value;
}
/********************************************************************/
void eeprom_write_byte(uint8_t address, uint8_t value)
{
    if (eeprom_read_byte(address) != value)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, EEPROM_ADDRESS | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, address, ACK_CHECK_EN);

        i2c_master_write_byte(cmd, value, ACK_CHECK_EN);

        i2c_master_stop(cmd);
        i2c_master_cmd_begin(EEPROM_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
}
/********************************************************************/
void eeprom_write_short(uint8_t address, uint16_t value)
{
    uint8_t HB = value >> 8;
    uint8_t LB = value & 0xFF;

    eeprom_write_byte(address, HB);
    eeprom_write_byte(address + 1, LB);
}
/********************************************************************/
void eeprom_init(i2c_port_t i2c_port)
{
    EEPROM_PORT = i2c_port;
}