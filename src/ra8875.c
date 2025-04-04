/**
 * @file ra8875.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ra8875.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include <string.h>

/*********************
 *      DEFINES
 *********************/

#define TAG "RA8875"

#define RA8875_MODE_DATA_WRITE  (0x00)
#define RA8875_MODE_DATA_READ   (0x40)
#define RA8875_MODE_CMD_WRITE   (0x80)
#define RA8875_MODE_STATUS_READ (0xC0)

#if (LV_COLOR_DEPTH == 8)
    #define SYSR_VAL (0x00)
#elif (LV_COLOR_DEPTH == 16)
    #warning "COLOR DEPTH SELECTED IS 16"
    #define SYSR_VAL (0x0C)
#else
    #error "Unsupported color depth (LV_COLOR_DEPTH)"
#endif
#define BYTES_PER_PIXEL (LV_COLOR_DEPTH / 8)

#define HDWR_VAL (LV_HOR_RES_MAX/8 - 1)
#define VDHR_VAL (LV_VER_RES_MAX - 1)

#define VDIR_MASK (1 << 2)
#define HDIR_MASK (1 << 3)

// #if ( CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED || CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED )
//     #if CONFIG_LV_INVERT_DISPLAY
//         #define DPCR_VAL (VDIR_MASK)
//     #else
//         #define DPCR_VAL (VDIR_MASK | HDIR_MASK)
//     #endif
// #else
//     #if CONFIG_LV_INVERT_DISPLAY
//         #define DPCR_VAL (HDIR_MASK)
//     #else
//         #define DPCR_VAL (0x00)
//     #endif
// #endif
#define DPCR_VAL (0x00) //DISPLAY NOT INVERTED, USE ABOVE LOGIC TO SET VALUES AND POSIBLY SET DIRECTIONS

// #if CONFIG_LV_DISP_RA8875_PCLK_INVERT
//     #define PCSR_VAL (0x80 | CONFIG_LV_DISP_RA8875_PCLK_MULTIPLIER)
// #else
//     #define PCSR_VAL (CONFIG_LV_DISP_RA8875_PCLK_MULTIPLIER)
// #endif
#define CONFIG_PCLK_RISING_EDGE     (0x00)
#define CONFIG_PCLK_FALLING_EDGE    (0x80)
#define CONFIG_PCLK_SYS_1           (0x00)
#define CONFIG_PCLK_SYS_2           (0x01)
#define CONFIG_PCLK_SYS_4           (0x02)
#define CONFIG_PCLK_SYS_8           (0x03)

#define PCSR_VAL (CONFIG_PCLK_RISING_EDGE | CONFIG_PCLK_SYS_1) //BETWEEN 0 -3 VALUES INCREASING 

// Calculate horizontal display parameters
#define CONFIG_LV_DISP_RA8875_HORI_NON_DISP_PERIOD 12  //default value for non display period RANGE 12- 274
#if (CONFIG_LV_DISP_RA8875_HORI_NON_DISP_PERIOD >= 260)
    #define HNDR_VAL (31)
#else
    #define HNDR_VAL ((CONFIG_LV_DISP_RA8875_HORI_NON_DISP_PERIOD-12) / 8)
#endif

#define HNDFT (CONFIG_LV_DISP_RA8875_HORI_NON_DISP_PERIOD-(8*HNDR_VAL)-12)

#if LVGL_DISP_RA8875_DE_POLARITY   //Set to make data being enabled on low or high
    #define HNDFTR_VAL (0x80 | HNDFT)
#else
    #define HNDFTR_VAL (HNDFT)
#endif

#define CONFIG_LV_DISP_RA8875_HSYNC_START   32   //range 8 - 256
#define CONFIG_LV_DISP_RA8875_HSYNC_PW      96
#define HSTR_VAL (CONFIG_LV_DISP_RA8875_HSYNC_START/8 - 1) //used to configure start position of hsync
#define HPW (CONFIG_LV_DISP_RA8875_HSYNC_PW/8 - 1)          //used to configure hsync pulse
#if LVGL_DISP_RA8875_HSYNC_POLARITY
    #define HPWR_VAL (0x80 | HPW)
#else
    #define HPWR_VAL (HPW)
#endif

// Calculate vertical display parameters
#define CONFIG_LV_DISP_RA8875_VERT_NON_DISP_PERIOD  32
#define CONFIG_LV_DISP_RA8875_VSYNC_START           23
#define CONFIG_LV_DISP_RA8875_VSYNC_PW              2
#define VNDR_VAL (CONFIG_LV_DISP_RA8875_VERT_NON_DISP_PERIOD - 1)
#define VSTR_VAL (CONFIG_LV_DISP_RA8875_VSYNC_START - 1)
#define VPW (CONFIG_LV_DISP_RA8875_VSYNC_PW - 1)
#if LVGL_DISP_RA8875_VSYNC_POLARITY
    #define VPWR_VAL (0x80 | VPW)
#else
    #define VPWR_VAL (VPW)
#endif

//CONFIGURATION FOR INTERNAL OR EXTERNAL BACKLIGHT USING PWM CHIP PINS
#ifdef  CONFIG_BACKLIGHT_INTERNAL
    #define BACKLIGHT_INTERNAL  1
#else
    #define BACKLIGHT_EXTERNAL  1
#endif

// #define PIXEL_TRANS_SIZE        (510) //PIXELS SENT EACH TIME
// #define SPI_PIXEL_TRANS_SIZE    (PIXEL_TRANS_SIZE*8) //SIZE IN BITS
#define PIXEL_TRANS_SIZE        (64) //PIXELS SENT EACH TIME
#define SPI_PIXEL_TRANS_SIZE    (PIXEL_TRANS_SIZE*16) //SIZE IN BITS

/**********************
 *      TYPEDEFS
 **********************/

/**********************/

static void ra8875_configure_clocks(bool high_speed);
 /**********************
 *  STATIC VARIABLES
 **********************/ 
spi_host_device_t spi_host = SPI3_HOST;
spi_device_handle_t spi;
spi_device_handle_t fast_spi;
spi_device_interface_config_t fastdevcfg;

/**********************
 *      MACROS
 **********************/
#define SPI_TRANSACTION_POOL_SIZE 50	/* maximum number of DMA transactions simultaneously in-flight */
#define SPI_MODE_BUS        3

inline uint8_t disp_spi_send_t(uint8_t data, uint8_t data2)__attribute__((always_inline));
inline uint8_t disp_spi_send_t(uint8_t data, uint8_t data2){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 16; //in bytes
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.rxlength = 8;

    t.tx_data[0] = data;
    t.tx_data[1] = data2;

    gpio_set_level(TFT_PIN_CS, 0);
    spi_device_polling_transmit(spi, &t);
    gpio_set_level(TFT_PIN_CS, 1);

    return t.rx_data[1];
}


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

uint8_t ra8875_init(void)
{
    struct {
        uint8_t cmd;                                   // Register address of command
        uint8_t data;                                  // Value to write to register
    } init_cmds[] = {
        {RA8875_REG_SYSR,   SYSR_VAL},                 // System Configuration Register (SYSR)
        {RA8875_REG_PCSR,   PCSR_VAL},
        {RA8875_REG_HDWR,   HDWR_VAL},                 // LCD Horizontal Display Width Register (HDWR)
        {RA8875_REG_HNDFTR, HNDFTR_VAL},               // Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)
        {RA8875_REG_HNDR,   HNDR_VAL},                 // Horizontal Non-Display Period Register (HNDR) TODO CORRECT FORMULA COULD BE 3 BY DEFA
        {RA8875_REG_HSTR,   HSTR_VAL},                 // HSYNC Start Position Register (HSTR)
        {RA8875_REG_HPWR,   HPWR_VAL},                 // HSYNC Pulse Width Register (HPWR)
        {RA8875_REG_VDHR0,  VDHR_VAL & 0x0FF},         // LCD Vertical Display Height Register (VDHR0)
        {RA8875_REG_VDHR1,  VDHR_VAL >> 8},            // LCD Vertical Display Height Register0 (VDHR1)
        {RA8875_REG_VNDR0,  VNDR_VAL & 0x0FF},         // LCD Vertical Non-Display Period Register (VNDR0)
        {RA8875_REG_VNDR1,  VNDR_VAL >> 8},            // LCD Vertical Non-Display Period Register (VNDR1)
        {RA8875_REG_VSTR0,  VSTR_VAL & 0x0FF},         // VSYNC Start Position Register (VSTR0)
        {RA8875_REG_VSTR1,  VSTR_VAL >> 8},            // VSYNC Start Position Register (VSTR1)
        {RA8875_REG_VPWR,   VPWR_VAL}                 // VSYNC Pulse Width Register (VPWR)
        //THESE COMMANDS ARE SENDING THE SAME VALUES AS DEFAULT VAL AFTER A RESET SO NO NEED THEM
        // {RA8875_REG_DPCR,   DPCR_VAL},                 // Display Configuration Register (DPCR)
        // {RA8875_REG_MWCR0,  0x00},                     // Memory Write Control Register 0 (MWCR0) //TODO MAY NEED TO MODIFY FOR ROTATION
        // {RA8875_REG_MWCR1,  0x00},                     // Memory Write Control Register 1 (MWCR1)
        // {RA8875_REG_LTPR0,  0x00},                     // Layer Transparency Register0 (LTPR0) //LAYER1 ENABLED
        // {RA8875_REG_LTPR1,  0x00},                     // Layer Transparency Register1 (LTPR1) //TRANSPARENCY FOR EACH DISPLAY
    };
    
    uint8_t init_cmd_size  = sizeof(init_cmds)/sizeof(init_cmds[0]);

    ESP_LOGI(TAG, "Initializing RA8875...");
    
    // Initialize non-SPI GPIOs
#if RA8875_USE_RST
    ESP_LOGI(TAG, "Sending reset sequence on PIN...");
    gpio_reset_pin(RA8875_RST);
    gpio_set_direction(RA8875_RST, GPIO_MODE_OUTPUT);
    gpio_pullup_dis(RA8875_RST);

    // Reset the RA8875
    gpio_set_level(RA8875_RST, 0);
    vTaskDelay( 100 / portTICK_PERIOD_MS);
    gpio_set_level(RA8875_RST, 1);
    vTaskDelay( 100 / portTICK_PERIOD_MS);
#endif

    disp_spi_init(SPI_TFT_CLOCK_SPEED_HZ);
    
    if ( ra8875_read_register(0x00) != 0x75 ){
        ESP_LOGE(TAG, "RA8875 SCREEN NOT FOUND");
        return false;
    }

    ESP_LOGI(TAG, "RA8875 SCREEN FOUND");
    
    //Initialize PLL clocks
    ra8875_configure_clocks(true);

    // Send all the commands to init the display
    for (uint8_t i = 0; i < init_cmd_size; i++) {
        ra8875_write_register(init_cmds[i].cmd, init_cmds[i].data);
    }

    //Set window area for the first time
    ra8875_set_window(0, LV_HOR_RES_MAX, 0, LV_VER_RES_MAX);

    // Perform a memory clear (wait maximum of 100 ticks) //could just be a delay 
    ra8875_write_register(RA8875_REG_MCLR, 0x80);
    for(uint8_t i = 100; i != 0; i--) {
        if ((ra8875_read_register(RA8875_REG_MCLR) & 0x80) == 0x00) {
            ESP_LOGI(TAG, "WAITING for Memory clear to be finished...");
            break;
        }
        vTaskDelay(10);
    }
    
    vTaskDelay(250 / portTICK_PERIOD_MS);

    // Enable the display
    ra8875_enable_display(true);

#ifdef BACKLIGHT_INTERNAL
    ra8875_write_register(RA8875_GPIOX, 1); //Enable pin attached to GPIOX as output to enable PWM
    configurePWM(PWM_PIN_1, true, RA8875_PWM_CLK_DIV32);
    PWMout(PWM_PIN_1, 255);
#endif

    return true;
}

void ra8875_enable_display(bool enable)
{
    ESP_LOGI(TAG, "%s display.", enable ? "Enabling" : "Disabling");
    uint8_t val = enable ? 0x80 : 0x00;
    ra8875_write_register(RA8875_REG_PWRR, val);            // Power and Display Control Register (PWRR)
}

void ra8875_set_rotation(int rotation){
    // ra8875_sleep_in(); //disable display
    
    uint8_t orientation_reg_value;
    switch(rotation){
        case 0:
            orientation_reg_value = 0; //PORTRAIT 
        break;

        case 1:
            orientation_reg_value = (VDIR_MASK | HDIR_MASK); //270
        break;

        case 2:
            orientation_reg_value = VDIR_MASK; // 180 
        break;

        case 3:
            orientation_reg_value = HDIR_MASK; //90
        break;

        default:
            orientation_reg_value = 0;
        break;
    }

    // ra8875_write_register(RA8875_REG_DPCR, orientation_reg_value); //send command to update value TODO, FIX THIS COMMAND WITH MWCR0 DATA
    // ra8875_sleep_out(); //enable display again
}

void ra8875_sleep_in(void)
{
    ESP_LOGI(TAG, "Device about to be sent to sleep...");
    ra8875_configure_clocks(false);

    ra8875_write_register(RA8875_REG_PWRR, 0x00);           // Power and Display Control Register (PWRR)
    vTaskDelay( 20 / portTICK_PERIOD_MS);
    ra8875_write_register(RA8875_REG_PWRR, 0x02);           // Power and Display Control Register (PWRR)
}

void ra8875_sleep_out(void)
{
    ESP_LOGI(TAG, "Device about to recover from sleep...");
    ra8875_write_register(RA8875_REG_PWRR, 0x00);           // Power and Display Control Register (PWRR)
    vTaskDelay( 20 / portTICK_PERIOD_MS);

    ra8875_configure_clocks(true);

    ra8875_write_register(RA8875_REG_PWRR, 0x80);           // Power and Display Control Register (PWRR)
    vTaskDelay( 20 / portTICK_PERIOD_MS);
}

uint8_t ra8875_read_register(uint8_t reg){
    // ESP_LOGI(TAG, "Device read register %02X triggered...", reg);

    writeCommand(reg);
    uint8_t rcv_buf = readData();
    return rcv_buf;
}

void ra8875_write_register(uint8_t reg, uint8_t value){
    writeCommand(reg);
    writeData(value);
}

void ra8875_configure_clocks(bool high_speed)
{
    // ESP_LOGI(TAG, "RA8875 device configuring clocks...");
    uint8_t val;
    
    val = high_speed ? ((CONFIG_LV_DISP_RA8875_PLLDIVM << 7) | CONFIG_LV_DISP_RA8875_PLLDIVN) : 0x07;
    ra8875_write_register(RA8875_REG_PLLC1, val);           // PLL Control Register 1 (PLLC1)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    val = high_speed ? CONFIG_LV_DISP_RA8875_PLLDIVK : 0x03;
    ra8875_write_register(RA8875_REG_PLLC2, val);           // PLL Control Register 2 (PLLC2)
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

void ra8875_set_window(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye){
    // ESP_LOGI(TAG, "RA8875 device setting a window..");
    ra8875_write_register(RA8875_REG_HSAW0, (uint8_t)(xs & 0x00FF)); // Horizontal Start Point 0 of Active Window (HSAW0)
    ra8875_write_register(RA8875_REG_HSAW1, (uint8_t)(xs >> 8));    // Horizontal Start Point 1 of Active Window (HSAW1)
    ra8875_write_register(RA8875_REG_VSAW0, (uint8_t)(ys & 0x00FF)); // Vertical Start Point 0 of Active Window (VSAW0)
    ra8875_write_register(RA8875_REG_VSAW1, (uint8_t)(ys >> 8));    // Vertical Start Point 1 of Active Window (VSAW1)
    ra8875_write_register(RA8875_REG_HEAW0, (uint8_t)(xe & 0x00FF)); // Horizontal End Point 0 of Active Window (HEAW0)
    ra8875_write_register(RA8875_REG_HEAW1, (uint8_t)(xe >> 8));    // Horizontal End Point 1 of Active Window (HEAW1)
    ra8875_write_register(RA8875_REG_VEAW0, (uint8_t)(ye & 0x00FF)); // Vertical End Point of Active Window 0 (VEAW0)
    ra8875_write_register(RA8875_REG_VEAW1, (uint8_t)(ye >> 8));    // Vertical End Point of Active Window 1 (VEAW1)
}

// Used to set the cursor at certain position 
void ra8875_set_memory_write_cursor(uint16_t x, uint16_t y)
{
    // ESP_LOGI(TAG, "RA8875 device setting a write cursor..");
    ra8875_write_register(RA8875_REG_CURH0, (uint8_t)(x & 0x00FF));  // Memory Write Cursor Horizontal Position Register 0 (CURH0)
    ra8875_write_register(RA8875_REG_CURH1, (uint8_t)(x >> 8));     // Memory Write Cursor Horizontal Position Register 1 (CURH1)
    ra8875_write_register(RA8875_REG_CURV0, (uint8_t)(y & 0x0FF));  // Memory Write Cursor Vertical Position Register 0 (CURV0)
    ra8875_write_register(RA8875_REG_CURV1, (uint8_t)(y >> 8));     // Memory Write Cursor Vertical Position Register 1 (CURV1)
}

void ra8875_send_buffer(uint16_t * data, size_t length)
{
    // ESP_LOGI(TAG, "RA8875 device sending a buffer of transactions");

    // uint8_t dir = 0; //fix it to contain rotation value
    // uint8_t curr_val = ra8875_read_register(RA8875_REG_MWCR0); 
    // ra8875_write_register(RA8875_REG_MWCR0, (curr_val & ~RA8875_REG_MWCR0_DIRMASK) | dir);
    writeCommand(RA8875_REG_MRWC);

    //TODO MAY NEED PARTIALLY UPDATE BUS WIDTH TO ACCELERATE TRANSFER
    // ra8875_write_register(RA8875_REG_SYSR, 0x0A); //HARDCODED TO 16BIT COLOR AND INTERFACE 
    disp_spi_send_buffer(data, length); //TODO REVIEW IF I CAN DO IT WITH THE ALINGMENT THAT IS ONLY THE ADDRESS TO SEND THE DATA
    // ra8875_write_register(RA8875_REG_SYSR, 0x08); //HARDCODED TO 16BIT COLOR AND INTERFACE 

}

static void configurePWM(uint8_t pwm_pin, bool enable, uint8_t pwm_clock){
    uint8_t register_pin = (pwm_pin == PWM_PIN_1) ? RA8875_REG_PC1R : RA8875_REG_PC2R;
    if( enable ){
        ra8875_write_register(register_pin, 0x80 | (pwm_clock & 0xF));
    }
    else{
        ra8875_write_register(register_pin, 0x00 | (pwm_clock & 0xF));
    }
}

static void PWMout(uint8_t pwm_pin, uint8_t duty_cycle){
    uint8_t register_pin = (pwm_pin == PWM_PIN_1) ? RA8875_REG_P1DCR : RA8875_REG_P2DCR;
    
    ra8875_write_register(register_pin, duty_cycle);
}

void writeCommand(uint8_t d){
    disp_spi_send_t((uint8_t)RA8875_MODE_CMD_WRITE, d);
}

uint8_t readData(){
    uint8_t val = 0;
    val = disp_spi_send_t((uint8_t)RA8875_MODE_DATA_READ, 0);
    return val;
}

void writeData(uint8_t d){
    disp_spi_send_t((uint8_t)RA8875_MODE_DATA_WRITE, d);
}


/**********************
 *** SPI BUS PROTOTYPES
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
        .max_transfer_sz = 4092,
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = clock_speed_hz,
        .mode = SPI_MODE_BUS,
        .spics_io_num= -1,              // CS pin
        .input_delay_ns= 0 ,
        .queue_size=SPI_TRANSACTION_POOL_SIZE,
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
        
    fastdevcfg = devcfg;
    fastdevcfg.clock_speed_hz = SPI_TFT_PIXEL_CLOCK_SPEED_HZ;
    bus_ret = spi_bus_add_device(spi_host, &fastdevcfg, &fast_spi);
    ESP_ERROR_CHECK(bus_ret);

    #ifdef DEBUG
        ESP_LOGI(TAG, "SPI for PIXEL bus after adding a new device %d", bus_ret);
    #endif

    gpio_set_direction(TFT_PIN_CS, GPIO_MODE_OUTPUT);
    gpio_pulldown_dis(TFT_PIN_CS);
    gpio_pullup_dis(TFT_PIN_CS);
}

//Working function for 8BIT pixel data
#if 0
void disp_spi_send_buffer(uint8_t* data, size_t length){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;            //length in bytes
    
    uint8_t mock_val = 0x00;
    gpio_set_level(TFT_PIN_CS, 0);
    t.tx_buffer = &mock_val;
    spi_device_polling_transmit(fast_spi, &t);

    //Approach by sending many bytes each time per clock transaction
    t.length = SPI_PIXEL_TRANS_SIZE;
    int i = 0;
    for(i = 0; (i + SPI_PIXEL_TRANS_SIZE) < length; i+=PIXEL_TRANS_SIZE){
        t.tx_buffer = data+i;
        spi_device_polling_transmit(fast_spi, &t);
    }

    t.length = (length-i)*8;
    t.tx_buffer = data+i;
    spi_device_polling_transmit(fast_spi, &t);

    gpio_set_level(TFT_PIN_CS, 1);
}
#endif

void disp_spi_send_buffer(uint16_t* data, size_t length){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;            //length in bytes
    
    uint8_t mock_val = 0x00;
    t.tx_buffer = &mock_val;
    gpio_set_level(TFT_PIN_CS, 0);
    spi_device_polling_transmit(fast_spi, &t);

    //Approach by sending many bytes each time per clock transaction

    int i = 0;
    uint16_t swapped[PIXEL_TRANS_SIZE];
    // for(i = 0; i+1 < length; i+=2){
    // ESP_LOGI(TAG, "Color at dispatcher first 0x%2X, and 0x%2X", (uint8_t)(*(data+i)), (uint8_t)(*(data+i) >> 8) );
    //FUCKING NEEDED TO SWAP BYTES FOR EACH ONE
    t.length = SPI_PIXEL_TRANS_SIZE;
    t.tx_buffer = &swapped[0];
    for(i = 0; (i + PIXEL_TRANS_SIZE) < length; i+=PIXEL_TRANS_SIZE){
        for(int j = 0; j < PIXEL_TRANS_SIZE; j++)
            swapped[j] = ((*(data+i) >> 8)&0x00FF) | ((*(data+i) << 8)&0xFF00);
            // t.tx_data[0] = (uint8_t)(*(data+i) >> 8);
            // t.tx_data[1] = (uint8_t)(*(data+i));
            // t.tx_data[2] = (uint8_t)(*(data+i+1) >> 8);
            // t.tx_data[3] = (uint8_t)(*(data+i+1));
            
            //send data in bundle of 32 transactions
        spi_device_polling_transmit(fast_spi, &t);
    }
    
    t.length = (length-i)*16;
    for(int j = 0; j < (length-i); j++){
        swapped[j] = ((*(data+i) >> 8) & 0x00FF) | ((*(data+i) << 8) & 0xFF00);
        i++;
    }

    // t.tx_data[0] = (uint8_t)(*(data+i) >> 8);
    // t.tx_data[1] = (uint8_t)(*(data+i));
    spi_device_polling_transmit(fast_spi, &t);

    gpio_set_level(TFT_PIN_CS, 1);
}

void disp_acquire_bus(){
    spi_device_acquire_bus(spi, portMAX_DELAY);
}

void disp_release_bus(){
    spi_device_release_bus(spi);
}

 /**********************
 *** SPI BUS PROTOTYPES
 **********************/

void fillScreen(uint16_t color) {
    // rectHelper(0, 0, _width - 1, _height - 1, color, true);

     /* Set X */
     int x = 0;
     ra8875_write_register(0x91, x);
     ra8875_write_register(0x92, x>>8);
 
     /* Set Y */
     int y = 0;
     ra8875_write_register(0x93, y);
     ra8875_write_register(0x94, y>>8);
 
     /* Set X1 */
     int w = 799;
     ra8875_write_register(0x95, w);
     ra8875_write_register(0x96, w>>8);
 
     /* Set Y1 */
     int h = 479;
     ra8875_write_register(0x97, h);
     ra8875_write_register(0x98, h>>8);
 
     /* Set Color */
     ra8875_write_register(0x63, (color & 0xf800) >> 11);
     ra8875_write_register(0x64, (color & 0x07e0) >> 5);
     ra8875_write_register(0x65, (color & 0x001f));
     
     /* Draw! */
     ra8875_write_register(0x90, 0xB0);    
     vTaskDelay( 150 / portTICK_PERIOD_MS);
}

void setCursor(uint16_t x, uint16_t y){
    /* Set cursor location */
    writeCommand(0x2A);
    writeData(x & 0xFF);
    writeCommand(0x2B);
    writeData(x >> 8);
    writeCommand(0x2C);
    writeData(y & 0xFF);
    writeCommand(0x2D);
    writeData(y >> 8);
}
  
void textTransparent(uint16_t foreColor) {
    /* Set Fore Color */
    writeCommand(0x63);
    writeData((foreColor & 0xf800) >> 11);
    writeCommand(0x64);
    writeData((foreColor & 0x07e0) >> 5);
    writeCommand(0x65);
    writeData((foreColor & 0x001f));

    /* Set transparency flag */
    writeCommand(0x22);
    uint8_t temp = readData();
    temp |= (1 << 6); // Set bit 6
    writeData(temp);
}
  
void textWrite(const char *buffer, uint16_t len) {
    if (len == 0)
      len = sizeof(buffer);
    writeCommand(0x02);
    for (uint16_t i = 0; i < len; i++)
      writeData(buffer[i]);
  
    vTaskDelay(10);
}
  
void textMode() {
    /* Set text mode */
    writeCommand(0x40);
    uint8_t temp = readData();
    temp |= 0x80; // Set bit 7
    writeData(temp);
  
    /* Select the internal (ROM) font*/
    writeCommand(0x21);
    temp = readData();
    temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
    writeData(temp);
}

void textEnlarge(uint8_t scale) {
    if (scale > 3)
      scale = 3; // highest setting is 3
  
    /* Set font size flags */
    writeCommand(0x22);
    uint8_t temp = readData();
    temp &= ~(0xF); // Clears bits 0..3
    temp |= scale << 2;
    temp |= scale;
  
    writeData(temp);
  
    // _textScale = scale;
  }

void graphicsMode() {
    /* Set text mode */
    writeCommand(0x40);
    uint8_t temp = readData();
    temp &= 0x7F; // clear bit 7
    writeData(temp);
}