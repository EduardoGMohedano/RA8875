#ifndef __FT5316_H__
#define __FT5316_H__

#include "stdint.h"
#include "stdbool.h"
#include "esp_check.h"

//Configurable stuff, pins, interruption pin and I2C bus speed
#define FT5316_SDA_PIN                  21
#define FT5316_SCL_PIN                  22
#define FT5316_INT_PIN                  15
#define FT5316_RST_PIN                  4
#define FT5316_I2C_SPEED                (400000)
//Configurable stuff, pins, interruption pin and I2C bus speed

#define FT5316_I2C_ADDRESS              (0x70)
#define I2C_MASTER_NUM                  0       // I2C port number

esp_err_t ft5316_init(void);
esp_err_t write_ft5316_register(uint8_t reg, uint8_t register_value);
esp_err_t read_ft5316_register(uint8_t reg, uint8_t* register_value);
uint8_t ft5x06_read_fw_ver();
bool ft5316_getTouch(uint16_t* x, uint16_t* y);

#endif