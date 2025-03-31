/**
 * SPI driver definitions for display
 * @file disp_spi.h
 *
 */

#ifndef DISP_SPI_H
#define DISP_SPI_H

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/
#define SPI_TFT_CLOCK_SPEED_HZ  (1*100*1000)
#define TFT_PIN_MISO            (19)
#define TFT_PIN_MOSI            (23)
#define TFT_PIN_CLK             (18)
#define TFT_PIN_CS              (2)
// #define TFT_SPI_HALF_DUPLEX    1 //Uncomment to enable half duplex communication

#define DEBUG   1


/**********************
 * GLOBAL PROTOTYPES
 **********************/

void disp_spi_init(int clock_speed_hz);
void disp_spi_send_t(uint8_t data, uint8_t data2, bool read, uint8_t* res);
void disp_spi_send_buffer(uint8_t* data, size_t length);
void disp_acquire_bus();
void disp_release_bus();
// uint8_t readData(uint8_t reg);



#endif /*DISP_SPI_H*/
