#include "bmp180.h"

#define BMP180_PORT I2C_NUM_0
#define BMP180_ADDRESS 0x77 // I2C address of BMP180

#define ACK_CHECK_EN 0x1  // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0 // I2C master will not check ack from slave
#define ACK_VAL 0x0		  // I2C ack value
#define NACK_VAL 0x1	  // I2C nack value

#define BMP180_ULTRA_LOW_POWER 0
#define BMP180_STANDARD 1
#define BMP180_HIGH_RES 2
#define BMP180_ULTRA_HIGH_RES 3

#define BMP180_CAL_AC1 0xAA // Calibration data (16 bits)
#define BMP180_CAL_AC2 0xAC // Calibration data (16 bits)
#define BMP180_CAL_AC3 0xAE // Calibration data (16 bits)
#define BMP180_CAL_AC4 0xB0 // Calibration data (16 bits)
#define BMP180_CAL_AC5 0xB2 // Calibration data (16 bits)
#define BMP180_CAL_AC6 0xB4 // Calibration data (16 bits)
#define BMP180_CAL_B1 0xB6	// Calibration data (16 bits)
#define BMP180_CAL_B2 0xB8	// Calibration data (16 bits)
#define BMP180_CAL_MB 0xBA	// Calibration data (16 bits)
#define BMP180_CAL_MC 0xBC	// Calibration data (16 bits)
#define BMP180_CAL_MD 0xBE	// Calibration data (16 bits)

#define BMP180_CONTROL 0xF4			  // Control register
#define BMP180_DATA_TO_READ 0xF6	  // Read results here
#define BMP180_READ_TEMP_CMD 0x2E	  // Request temperature measurement
#define BMP180_READ_PRESSURE_CMD 0x34 // Request pressure measurement

// Calibration parameters
static int16_t ac1;
static int16_t ac2;
static int16_t ac3;
static uint16_t ac4;
static uint16_t ac5;
static uint16_t ac6;
static int16_t b1;
static int16_t b2;
static int16_t mb;
static int16_t mc;
static int16_t md;
static uint8_t oversampling = BMP180_ULTRA_HIGH_RES;

/********************************************************************/
static void bmp180_master_write_slave(uint8_t *data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(BMP180_PORT, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}
/********************************************************************/
static void bmp180_write_reg(uint8_t reg, uint8_t cmd)
{
	uint8_t data_wr[] = {reg, cmd};
	bmp180_master_write_slave(data_wr, 2);
}
/********************************************************************/
static void bmp180_master_read_slave(uint8_t *data_rd, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	if (size > 1)
	{
		i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(BMP180_PORT, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}
/********************************************************************/
static void bmp180_read_int16(uint8_t reg, int16_t *value)
{
	bmp180_master_write_slave(&reg, 1);
	uint8_t data_rd[2] = {0};
	bmp180_master_read_slave(data_rd, 2);
	*value = (int16_t)((data_rd[0] << 8) | data_rd[1]);
}
/********************************************************************/
static void bmp180_read_uint16(uint8_t reg, uint16_t *value)
{
	bmp180_master_write_slave(&reg, 1);
	uint8_t data_rd[2] = {0};
	bmp180_master_read_slave(data_rd, 2);
	*value = (uint16_t)((data_rd[0] << 8) | data_rd[1]);
}
/********************************************************************/
static void bmp180_read_uint32(uint8_t reg, uint32_t *value)
{
	bmp180_master_write_slave(&reg, 1);
	uint8_t data_rd[3] = {0};
	bmp180_master_read_slave(data_rd, 3);
	*value = (uint32_t)((data_rd[0] << 16) | (data_rd[1] << 8) | data_rd[2]);
}
/********************************************************************/
static void bmp180_read_uncompensated_temperature(int16_t *temp)
{
	bmp180_write_reg(BMP180_CONTROL, BMP180_READ_TEMP_CMD);
	TickType_t xDelay = 5 / portTICK_PERIOD_MS;
	if (xDelay == 0)
	{
		xDelay = 1;
	}
	vTaskDelay(xDelay);
	bmp180_read_int16(BMP180_DATA_TO_READ, temp);
}
/********************************************************************/
static void bmp180_calculate_b5(int32_t *b5)
{
	int16_t ut;
	int32_t x1, x2;

	bmp180_read_uncompensated_temperature(&ut);
	x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
	x2 = ((int32_t)mc << 11) / (x1 + md);
	*b5 = x1 + x2;
}
/********************************************************************/
static void bmp180_read_uncompensated_pressure(uint32_t *up)
{
	bmp180_write_reg(BMP180_CONTROL, BMP180_READ_PRESSURE_CMD + (oversampling << 6));
	TickType_t xDelay = (2 + (3 << oversampling)) / portTICK_PERIOD_MS;
	if (xDelay == 0)
	{
		xDelay = 1;
	}
	vTaskDelay(xDelay);
	bmp180_read_uint32(BMP180_DATA_TO_READ, up);
	*up >>= (8 - oversampling);
}
/********************************************************************/
float bmp180_read_temp(void)
{
	int32_t b5;
	bmp180_calculate_b5(&b5);
	float temp = ((b5 + 8) >> 4) / 10.0;
	return temp; // Температура в Цельсиях
}
/********************************************************************/
uint16_t bmp180_read_press(void)
{
	int32_t b3, b5, b6, x1, x2, x3, p;
	uint32_t up, b4, b7;

	bmp180_calculate_b5(&b5);

	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6) >> 12) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int32_t)ac1) * 4 + x3) << oversampling) + 2) >> 2;

	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

	bmp180_read_uncompensated_pressure(&up);

	b7 = ((uint32_t)(up - b3) * (50000 >> oversampling));
	if (b7 < 0x80000000)
	{
		p = (b7 << 1) / b4;
	}
	else
	{
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;						 // Давление в Pa
	uint16_t press = (uint16_t)((float)p * 0.0075f); // Давление в mm Hg
	return press;
}
/********************************************************************/
void bmp180_init(void)
{
	uint8_t reg = 0x00;
	bmp180_master_write_slave(&reg, 1);

	bmp180_read_int16(BMP180_CAL_AC1, &ac1);
	bmp180_read_int16(BMP180_CAL_AC2, &ac2);
	bmp180_read_int16(BMP180_CAL_AC3, &ac3);
	bmp180_read_uint16(BMP180_CAL_AC4, &ac4);
	bmp180_read_uint16(BMP180_CAL_AC5, &ac5);
	bmp180_read_uint16(BMP180_CAL_AC6, &ac6);
	bmp180_read_int16(BMP180_CAL_B1, &b1);
	bmp180_read_int16(BMP180_CAL_B2, &b2);
	bmp180_read_int16(BMP180_CAL_MB, &mb);
	bmp180_read_int16(BMP180_CAL_MC, &mc);
	bmp180_read_int16(BMP180_CAL_MD, &md);
}

// #include "bmp180.h"
// //#include "math.h"

// #define BMP180_I2CADDR (0x77 << 1)
// #define I2C_PORT I2C_NUM_0

// //#define BMP180_ULTRALOWPOWER 0
// #define BMP180_STANDARD 1
// //#define BMP180_HIGHRES       2
// //#define BMP180_ULTRAHIGHRES  3

// #define BMP180_CAL_AC1 0xAA // R   Calibration data (16 bits)
// #define BMP180_CAL_AC2 0xAC // R   Calibration data (16 bits)
// #define BMP180_CAL_AC3 0xAE // R   Calibration data (16 bits)
// #define BMP180_CAL_AC4 0xB0 // R   Calibration data (16 bits)
// #define BMP180_CAL_AC5 0xB2 // R   Calibration data (16 bits)
// #define BMP180_CAL_AC6 0xB4 // R   Calibration data (16 bits)
// #define BMP180_CAL_B1 0xB6	// R   Calibration data (16 bits)
// #define BMP180_CAL_B2 0xB8	// R   Calibration data (16 bits)
// #define BMP180_CAL_MB 0xBA	// R   Calibration data (16 bits)
// #define BMP180_CAL_MC 0xBC	// R   Calibration data (16 bits)
// #define BMP180_CAL_MD 0xBE	// R   Calibration data (16 bits)

// #define BMP180_CONTROL 0xF4
// #define BMP180_TEMPDATA 0xF6
// #define BMP180_PRESSUREDATA 0xF6
// #define BMP180_READTEMPCMD 0x2E
// #define BMP180_READPRESSURECMD 0x34

// #define BMP180_SOFT_RESET_REG 0xE0 // Soft reset control

// static uint8_t oversampling = BMP180_STANDARD;

// int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
// uint16_t ac4, ac5, ac6;

// /* static */ void bmp180_write(uint8_t *data, size_t size)
// {
// 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// 	i2c_master_start(cmd);
// 	i2c_master_write_byte(cmd, (BMP180_I2CADDR | I2C_MASTER_WRITE), I2C_MASTER_NACK);
// 	i2c_master_write(cmd, data, size, I2C_MASTER_NACK);
// 	i2c_master_stop(cmd);
// 	i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
// 	i2c_cmd_link_delete(cmd);
// }
// /***********************************************************/
// /* static */ void bmp180_write_reg(uint8_t reg, uint8_t cmd)
// {
// 	uint8_t data[2] = {reg, cmd};
// 	bmp180_write(data, 2);
// }
// /***********************************************************/
// /* static */ void bmp180_read(uint8_t *data, size_t size)
// {
// 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// 	i2c_master_start(cmd);
// 	i2c_master_write_byte(cmd, (BMP180_I2CADDR | I2C_MASTER_READ), I2C_MASTER_NACK);
// 	if (size > 1)
// 	{
// 		i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
// 	}
// 	i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
// 	i2c_master_stop(cmd);
// 	i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
// 	i2c_cmd_link_delete(cmd);
// }
// /***********************************************************/
// /* static */ void bmp180_read_int16(uint8_t reg, int16_t *value)
// {
// 	bmp180_write(&reg, 1);
// 	uint8_t data[2] = {0};
// 	bmp180_read(data, 2);
// 	*value = (int16_t)((data[0] << 8) | data[1]);
// }
// /***********************************************************/
// /* static */ void bmp180_read_uint16(uint8_t reg, uint16_t *value)
// {
// 	bmp180_write(&reg, 1);
// 	uint8_t data[2] = {0};
// 	bmp180_read(data, 2);
// 	*value = (uint16_t)((data[0] << 8) | data[1]);
// }
// /***********************************************************/
// /* static */ void bmp180_read_uint32(uint8_t reg, uint32_t *value)
// {
// 	bmp180_write(&reg, 1);
// 	uint8_t data[3] = {0};
// 	bmp180_read(data, 3);
// 	*value = (uint32_t)((data[0] << 16) | (data[1] << 8) | data[2]);
// }
// /***********************************************************/
// /* static */ void bmp180_read_raw_temp(int16_t *temp)
// {
// 	bmp180_write_reg(BMP180_CONTROL, BMP180_READTEMPCMD);
// 	vTaskDelay(5 / portTICK_PERIOD_MS);
// 	bmp180_read_int16(BMP180_TEMPDATA, temp);
// }
// /***********************************************************/
// /* int32_t bmp180_calculate_b5(int32_t UT)
// {
// 	int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
// 	int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
// 	return X1 + X2;
// } */
// /***********************************************************/
// /* static */ void bmp180_read_raw_press(uint32_t *up)
// {
// 	bmp180_write_reg(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));
// 	// TickType_t xDelay = (2 + (3 << oversampling)) / portTICK_PERIOD_MS;
// 	vTaskDelay(8 / portTICK_PERIOD_MS);
// 	bmp180_read_uint32(BMP180_PRESSUREDATA, up);

// 	*up >>= (8 - oversampling);
// }
// /* static */ void bmp180_calculate_b5(int32_t *b5)
// {
// 	int16_t ut = 0;
// 	int32_t x1 = 0, x2 = 0;

// 	bmp180_read_raw_temp(&ut);
// 	x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
// 	x2 = ((int32_t)mc << 11) / (x1 + md);
// 	*b5 = x1 + x2;
// }

// /***********************************************************/
// void bmp180_read_temp(float *temp)
// {
// 	int32_t b5 = 0;
// 	//float temp = 0;
// 	bmp180_calculate_b5(&b5);
// 	*temp = ((b5 + 8) >> 4) / 10.0;

// 	// return temp;
// }
// /***********************************************************/
// void bmp180_read_press(uint32_t *press)
// {
// 	int32_t b3 = 0, b5 = 0, b6 = 0, x1 = 0, x2 = 0, x3 = 0, p = 0;
// 	uint32_t up = 0, b4 = 0, b7 = 0;
// 	// bmp180_read_raw_press(&up);

// 	bmp180_calculate_b5(&b5);

// 	// bmp180_read_raw_temp(&ut);

// 	b6 = b5 - 4000;
// 	x1 = (b2 * (b6 * b6) >> 12) >> 11;
// 	x2 = (ac2 * b6) >> 11;
// 	x3 = x1 + x2;
// 	b3 = (((((int32_t)ac1) * 4 + x3) << oversampling) + 2) >> 2;

// 	x1 = (ac3 * b6) >> 13;
// 	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
// 	x3 = ((x1 + x2) + 2) >> 2;
// 	b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

// 	bmp180_read_raw_press(&up);

// 	b7 = ((uint32_t)(up - b3) * (50000 >> oversampling));
// 	if (b7 < 0x80000000)
// 	{
// 		p = (b7 << 1) / b4;
// 	}
// 	else
// 	{
// 		p = (b7 / b4) << 1;
// 	}

// 	x1 = (p >> 8) * (p >> 8);
// 	x1 = (x1 * 3038) >> 16;
// 	x2 = (-7357 * p) >> 16;
// 	p += (x1 + x2 + 3791) >> 4; // Давление в Паскалях

// 	// p /= 100; // Давление в ГектоПаскалях
// 	*press = p;
// 	// return p;
// }
// /***********************************************************/
// void bmp180_init(void)
// {
// 	bmp180_read_int16(BMP180_CAL_AC1, &ac1);
// 	bmp180_read_int16(BMP180_CAL_AC2, &ac2);
// 	bmp180_read_int16(BMP180_CAL_AC3, &ac3);
// 	bmp180_read_uint16(BMP180_CAL_AC4, &ac4);
// 	bmp180_read_uint16(BMP180_CAL_AC5, &ac5);
// 	bmp180_read_uint16(BMP180_CAL_AC6, &ac6);
// 	bmp180_read_int16(BMP180_CAL_B1, &b1);
// 	bmp180_read_int16(BMP180_CAL_B2, &b2);
// 	bmp180_read_int16(BMP180_CAL_MB, &mb);
// 	bmp180_read_int16(BMP180_CAL_MC, &mc);
// 	bmp180_read_int16(BMP180_CAL_MD, &md);
// }
