/**/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres                                              */
/*    @file    spi_communication.h                                               */
/*    @version v1                                                                */
/*    @date    01-September-2020                                                 */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the spi of the ADXL343                       */
/*             accelerometer via hardware in ESP32                               */
/*********************************************************************************/
/**/

#ifndef __SPI_COMMUNICATION_H__
#define __SPI_COMMUNICATION_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_system.h"
#include "esp_event.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp32/rom/cache.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

#define ADXL_SPI_BUF_SIZE 64    // SPI register data buffer size of ESP32

#define ADXL_SPI_READ_FLAG      0x80
#define ADXL_SPI_WRITE_FLAG     0x00
#define ADXL_SPI_MULTB_FLAG     0x40

typedef enum{
    ADXL343 = 0,
    IIS3DWB = 1
}spi_accel_t;

spi_device_handle_t spi;

esp_err_t SPI_init(spi_accel_t accel);
esp_err_t SPI_deinit();
bool spi_reg_write(uint8_t reg, uint8_t *data, uint16_t len);
bool spi_reg_read(uint8_t reg, uint8_t *data, uint16_t len);
size_t spi_transfer_data(const uint8_t *mosi, uint8_t *miso, uint16_t len);

#endif