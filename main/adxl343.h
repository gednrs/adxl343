/**/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres                                              */
/*    @file    adxl343.h                                                         */
/*    @version v1                                                                */
/*    @date    01-September-2020                                                 */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the ADXL343 accelerometer                    */
/*             driver using ESP32 SPI                                            */
/*********************************************************************************/
/**/

#ifndef __ADXL343_H__
#define __ADXL343_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "spi_communication.h"

#define ADXL343_DEVID     0x00        // Device ID
#define ADXL343_RESERVED1   0x01      // Reserved. Do Not Access. 
#define ADXL343_THRESH_TAP    0x1D    // Tap Threshold. 
#define ADXL343_OFSX      0x1E        // X-Axis Offset. 
#define ADXL343_OFSY      0x1F        // Y-Axis Offset.
#define ADXL343_OFSZ      0x20        // Z- Axis Offset.
#define ADXL343_DUR       0x21        // Tap Duration.
#define ADXL343_LATENT      0x22      // Tap Latency.
#define ADXL343_WINDOW      0x23      // Tap Window.
#define ADXL343_THRESH_ACT    0x24    // Activity Threshold
#define ADXL343_THRESH_INACT  0x25    // Inactivity Threshold
#define ADXL343_TIME_INACT    0x26    // Inactivity Time
#define ADXL343_ACT_INACT_CTL 0x27    // Axis Enable Control for Activity and Inactivity Detection
#define ADXL343_THRESH_FF   0x28      // Free-Fall Threshold.
#define ADXL343_TIME_FF     0x29      // Free-Fall Time.
#define ADXL343_TAP_AXES    0x2A      // Axis Control for Tap/Double Tap.
#define ADXL343_ACT_TAP_STATUS  0x2B  // Source of Tap/Double Tap
#define ADXL343_BW_RATE     0x2C      // Data Rate and Power mode Control
#define ADXL343_POWER_CTL   0x2D      // Power-Saving Features Control
#define ADXL343_INT_ENABLE    0x2E    // Interrupt Enable Control
#define ADXL343_INT_MAP     0x2F      // Interrupt Mapping Control
#define ADXL343_INT_SOURCE    0x30    // Source of Interrupts
#define ADXL343_DATA_FORMAT   0x31    // Data Format Control
#define ADXL343_DATAX0      0x32      // X-Axis Data 0
#define ADXL343_DATAX1      0x33      // X-Axis Data 1
#define ADXL343_DATAY0      0x34      // Y-Axis Data 0
#define ADXL343_DATAY1      0x35      // Y-Axis Data 1
#define ADXL343_DATAZ0      0x36      // Z-Axis Data 0
#define ADXL343_DATAZ1      0x37      // Z-Axis Data 1
#define ADXL343_FIFO_CTL    0x38      // FIFO Control
#define ADXL343_FIFO_STATUS   0x39    // FIFO Status

#define ADXL343_BW_1600     0xF       // 1111   IDD = 40uA
#define ADXL343_BW_800      0xE       // 1110   IDD = 90uA
#define ADXL343_BW_400      0xD       // 1101   IDD = 140uA
#define ADXL343_BW_200      0xC       // 1100   IDD = 140uA
#define ADXL343_BW_100      0xB       // 1011   IDD = 140uA 
#define ADXL343_BW_50       0xA       // 1010   IDD = 140uA
#define ADXL343_BW_25       0x9       // 1001   IDD = 90uA
#define ADXL343_BW_12_5     0x8       // 1000   IDD = 60uA 
#define ADXL343_BW_6_25     0x7       // 0111   IDD = 50uA
#define ADXL343_BW_3_13     0x6       // 0110   IDD = 45uA
#define ADXL343_BW_1_56     0x5       // 0101   IDD = 40uA
#define ADXL343_BW_0_78     0x4       // 0100   IDD = 34uA
#define ADXL343_BW_0_39     0x3       // 0011   IDD = 23uA
#define ADXL343_BW_0_20     0x2       // 0010   IDD = 23uA
#define ADXL343_BW_0_10     0x1       // 0001   IDD = 23uA
#define ADXL343_BW_0_05     0x0       // 0000   IDD = 23uA

#define ADXL343_BW_1600     0xF       // 1111 
#define ADXL343_BW_800      0xE       // 1110 
#define ADXL343_BW_400      0xD       // 1101 
#define ADXL343_BW_200      0xC       // 1100   
#define ADXL343_BW_100      0xB       // 1011 

#define DFRMT_RANGE_MASK         0xFC
#define PWRCTRL_OPMODE_MASK      0xF7
#define DFRMT_FULLRESBIT_MASK    0xF7
#define BW_RATE_MASK             0xF0

#define ADXL343_DEVICE_ID      0xE5     

#define SCALE_MULTIPLIER 0.0039

struct ADXL343_Accel {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct ADXL343_AccelG {
  double x;
  double y;
  double z;
};

typedef enum {

  STAND_BY = 0,
  FULL_BW_MEASUREMENT
} ADXL343_OP_MODE;

typedef enum {

  RANGE_2G = 0,
  RANGE_4G,
  RANGE_8G,
  RANGE_16G
} ADXL343_RANGE;

uint8_t adxl_init_device();
uint8_t adxl_get_id();
uint8_t adxl_update_reg(uint8_t reg_addr, uint8_t mask, uint8_t shift, uint8_t data);
bool adxl_operation_mode(ADXL343_OP_MODE mode);
bool adxl_range_setting(ADXL343_RANGE range);
bool adxl_full_res_bit(bool res_bit);
bool adxl_set_bw(uint8_t bw);
void read_all();
struct ADXL343_AccelG adxl_convert_to_g(const struct ADXL343_Accel *triplet);
void adxl_read_raw_data(struct ADXL343_Accel *triplet);
uint8_t get_data_ready();

#endif