#include "FT5316.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "pin_configuration.h"

//Registers configuration
#define FT5x06_DEVICE_MODE                0x00
#define FT5x06_GESTURE_ID                 0x01
#define FT5x06_TOUCH_POINTS               0x02

#define FT5x06_TOUCH1_EV_FLAG             0x03
#define FT5x06_TOUCH1_XH                  0x03
#define FT5x06_TOUCH1_XL                  0x04
#define FT5x06_TOUCH1_YH                  0x05
#define FT5x06_TOUCH1_YL                  0x06

#define FT5X0X_REG_THGROUP                0x80   /* touch threshold related to sensitivity */
#define FT5X0X_REG_THPEAK                 0x81
#define FT5X0X_REG_THCAL                  0x82
#define FT5X0X_REG_THWATER                0x83
#define FT5X0X_REG_THTEMP                 0x84
#define FT5X0X_REG_THDIFF                 0x85
#define FT5X0X_REG_CTRL                   0x86
#define FT5X0X_REG_TIMEENTERMONITOR       0x87
#define FT5X0X_REG_PERIODACTIVE           0x88   /* report rate */
#define FT5X0X_REG_PERIODMONITOR          0x89
#define FT5X0X_REG_HEIGHT_B               0x8a
#define FT5X0X_REG_MAX_FRAME              0x8b
#define FT5X0X_REG_DIST_MOVE              0x8c
#define FT5X0X_REG_DIST_POINT             0x8d
#define FT5X0X_REG_FEG_FRAME              0x8e
#define FT5X0X_REG_SINGLE_CLICK_OFFSET    0x8f
#define FT5X0X_REG_DOUBLE_CLICK_TIME_MIN  0x90
#define FT5X0X_REG_SINGLE_CLICK_TIME      0x91
#define FT5X0X_REG_LEFT_RIGHT_OFFSET      0x92
#define FT5X0X_REG_UP_DOWN_OFFSET         0x93
#define FT5X0X_REG_DISTANCE_LEFT_RIGHT    0x94
#define FT5X0X_REG_DISTANCE_UP_DOWN       0x95
#define FT5X0X_REG_ZOOM_DIS_SQR           0x96
#define FT5X0X_REG_RADIAN_VALUE           0x97
#define FT5X0X_REG_MAX_X_HIGH             0x98
#define FT5X0X_REG_MAX_X_LOW              0x99
#define FT5X0X_REG_MAX_Y_HIGH             0x9a
#define FT5X0X_REG_MAX_Y_LOW              0x9b
#define FT5X0X_REG_K_X_HIGH               0x9c
#define FT5X0X_REG_K_X_LOW                0x9d
#define FT5X0X_REG_K_Y_HIGH               0x9e
#define FT5X0X_REG_K_Y_LOW                0x9f
#define FT5X0X_REG_AUTO_CLB_MODE          0xa0
#define FT5X0X_REG_LIB_VERSION_H          0xa1
#define FT5X0X_REG_LIB_VERSION_L          0xa2
#define FT5X0X_REG_CIPHER                 0xa3
#define FT5X0X_REG_MODE                   0xa4
#define FT5X0X_REG_PMODE                  0xa5   /* Power Consume Mode        */
#define FT5X0X_REG_FIRMID                 0xa6   /* Firmware version */
#define FT5X0X_REG_STATE                  0xa7
#define FT5X0X_REG_FT5201ID               0xa8
#define FT5X0X_REG_ERR                    0xa9
#define FT5X0X_REG_CLB                    0xaa
#define TOUCH_MAX_POINT_NUMBER            (2)

const char* TAG = "FT5316";

esp_err_t ft5316_init(void){
    esp_log_level_set(TAG, ESP_LOG_WARN);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = FT5316_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = FT5316_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FT5316_I2C_SPEED,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    gpio_set_direction(FT5316_RST_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_pulldown_dis(FT5316_RST_PIN);
    gpio_pullup_dis(FT5316_RST_PIN);

    gpio_set_level(FT5316_RST_PIN, 0);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    gpio_set_level(FT5316_RST_PIN, 1);
    vTaskDelay(150 / portTICK_PERIOD_MS);

    esp_err_t ret = write_ft5316_register(FT5X0X_REG_THGROUP, 25); //default value is 30
 
    // valid touching peak detect threshold
    ret |= write_ft5316_register(FT5X0X_REG_THPEAK, 0x3C);

    // Touch focus threshold
    ret |= write_ft5316_register(FT5X0X_REG_THCAL, 16);

    // threshold when there is surface water
    ret |= write_ft5316_register(FT5X0X_REG_THWATER, 0x01);

    // threshold of temperature compensation
    ret |= write_ft5316_register(FT5X0X_REG_THTEMP, 0x01);

    // Touch difference threshold
    ret |= write_ft5316_register(FT5X0X_REG_THDIFF, 0xA0);

    // Delay to enter 'Monitor' status (s)
    ret |= write_ft5316_register(FT5X0X_REG_TIMEENTERMONITOR, 0x0A);

    // Period of 'Active' status (ms)
    ret |= write_ft5316_register(FT5X0X_REG_PERIODACTIVE, 0x0A);

    // Timer to enter 'idle' when in 'Monitor' (ms)
    ret |= write_ft5316_register(FT5X0X_REG_PERIODMONITOR, 0x28);
    
    ret |= write_ft5316_register(FT5X0X_REG_CTRL, 1); //enable monitor

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if( ret == ESP_OK )
        ESP_LOGI(TAG, "Initial successful FT5316 TOUCH DRIVER INIT");
    
    return ret;
}

esp_err_t read_ft5316_register(uint8_t reg, uint8_t* register_value){
    esp_err_t ret;
    
    // 1. Write transaction to set the register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                              // Start condition
    i2c_master_write_byte(cmd, FT5316_I2C_ADDRESS, true);      // Slave address (write bit)
    i2c_master_write_byte(cmd, reg, true); // Register address
    
    // 2. Write data byte
    i2c_master_start(cmd);                              // Repeated start condition
    i2c_master_write_byte(cmd, FT5316_I2C_ADDRESS | 0x01, true); // Slave address (read bit)
    i2c_master_read_byte(cmd, register_value, I2C_MASTER_NACK);  // Read one byte with NACK
    i2c_master_stop(cmd);                               // Stop condition
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); 
    return ret;
}

esp_err_t write_ft5316_register(uint8_t reg, uint8_t register_value){
    esp_err_t ret;
    
    // 1. Write transaction to set the register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                              // Start condition
    i2c_master_write_byte(cmd, FT5316_I2C_ADDRESS, true);      // Slave address (write bit)
    i2c_master_write_byte(cmd, reg, true); // Register address
    
    // 2. Write data byte
    i2c_master_write_byte(cmd, register_value, true); // 
    i2c_master_stop(cmd);                               // Stop condition
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); 
    return ret;
}

esp_err_t read_ft5316_firmware_version(uint8_t *version)
{
    esp_err_t ret;
    
    // 1. Write transaction to set the register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                              // Start condition
    i2c_master_write_byte(cmd, FT5316_I2C_ADDRESS, true);      // Slave address (write bit)
    i2c_master_write_byte(cmd, FT5X0X_REG_FIRMID, true); // Register address
    
    // 2. Read transaction with repeated start
    i2c_master_start(cmd);                              // Repeated start condition
    i2c_master_write_byte(cmd, FT5316_I2C_ADDRESS | 0x01, true); // Slave address (read bit)
    i2c_master_read_byte(cmd, version, I2C_MASTER_NACK);  // Read one byte with NACK
    i2c_master_stop(cmd);                               // Stop condition
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

bool ft5316_getTouch(uint16_t* x, uint16_t* y){

    uint8_t points = 0;
    esp_err_t ret = read_ft5316_register(FT5x06_TOUCH_POINTS, &points);
    points &= 0x07;
    
    uint8_t data[2] = {0,0};
    if (points > 0 ) {
        read_ft5316_register(FT5x06_TOUCH1_XH, &data[0]);
        read_ft5316_register(FT5x06_TOUCH1_XL, &data[1]);

        *x = 0x0fff & ((uint16_t)(data[0]) << 8 | data[1]);

        read_ft5316_register(FT5x06_TOUCH1_YH, &data[0]);
        read_ft5316_register(FT5x06_TOUCH1_YL, &data[1]);
        *y = ((uint16_t)(data[0]) << 8 | data[1]);
        ESP_LOGI(TAG, "Point was (%d,%d)", *x, *y);
    }  

    return points > 0;
}