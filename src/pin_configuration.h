#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__

//TFT Screen Related pin configurations
#define TFT_PIN_MISO                (19) //Pin number
#define TFT_PIN_MOSI                (23) //Pin number
#define TFT_PIN_CLK                 (18)
#define TFT_PIN_CS                  (2)

//Configurable Pins for I2C bus related to Touch Screen device
#define FT5316_SDA_PIN               (21)
#define FT5316_SCL_PIN               (22)
#define FT5316_RST_PIN               (4)
// #define FT5316_INT_PIN               15  //This pin is actually not needed in the implementation

#endif