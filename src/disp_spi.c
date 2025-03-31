/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include <string.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/semphr.h>
// #include <freertos/task.h>

// #ifdef LV_LVGL_H_INCLUDE_SIMPLE
// #include "lvgl.h"
// #else
// #include "lvgl/lvgl.h"
// #endif

#include "disp_spi.h"

static const char* TAG = "disp_spi";

/*********************
 *      DEFINES
 *********************/
#define SPI_TRANSACTION_POOL_SIZE 50	/* maximum number of DMA transactions simultaneously in-flight */

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_host_device_t spi_host = SPI2_HOST;
static spi_device_handle_t spi;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/


void disp_spi_init(int clock_speed_hz)
{
    ESP_LOGI(TAG, "Adding SPI device with speed %d", clock_speed_hz);

    spi_bus_config_t buscfg = {
        .miso_io_num = TFT_PIN_MISO,
        .mosi_io_num = TFT_PIN_MOSI,
        .sclk_io_num = TFT_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10,
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = clock_speed_hz,
        .mode = 0,
        .spics_io_num= TFT_PIN_CS,              // CS pin
        .input_delay_ns= 0 ,
        .queue_size=25,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .address_bits = 0,
        .command_bits = 0,
        .dummy_bits = 0,
        .flags = 0
    };

    //Initialize the SPI bus
    esp_err_t bus_ret = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
    #ifdef DEBUG
        ESP_LOGI(TAG, "SPI bus init has returned val%d", bus_ret);
        ESP_ERROR_CHECK(bus_ret);
    #endif
        
        bus_ret = spi_bus_add_device(spi_host, &devcfg, &spi);
        ESP_ERROR_CHECK(bus_ret);

    #ifdef DEBUG
        ESP_LOGI(TAG, "SPI bus after adding a new device %d", bus_ret);
    #endif

}

void disp_spi_send_t(uint8_t data, uint8_t data2, bool read, uint8_t* res){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 2*8; //in bytes
    t.flags = SPI_TRANS_USE_TXDATA;
    if(read){
        t.flags |= SPI_TRANS_USE_RXDATA;
        t.rxlength = 1*8;
    }
    t.tx_data[0] = data;
    t.tx_data[1] = data2;

    spi_device_polling_transmit(spi, &t);

    if(res)
        *res = t.rx_data[1];
}


void disp_spi_send_buffer(uint8_t* data, size_t length){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = length*8;            //length in bytes
    t.tx_buffer = data;

    spi_device_polling_transmit(spi, &t);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

void disp_acquire_bus(){
    spi_device_acquire_bus(spi, portMAX_DELAY);
}

void disp_release_bus(){
    spi_device_release_bus(spi);
}

