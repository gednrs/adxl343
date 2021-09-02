/**/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres                                              */
/*    @file    main.c                                                            */
/*    @version v1                                                                */
/*    @date    01-September-2020                                                 */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the ADXL343 accelerometer                    */
/*             driver using ESP32 SPI                                            */
/*********************************************************************************/
/**/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "adxl343.h"

#define TAG adxl343

struct ADXL343_Accel accel;
struct ADXL343_AccelG accelG;

void app_main(void){

  ESP_LOGI(TAG, "*********** ADL343 TEST *********** \n");
  
  accel_device_id = adxl_init_device();

  if(accel_device_id == ADXL343_DEVICE_ID) { 
    ESP_LOGI(TAG, "ADXL343 ready!");
  }
  else 
    ESP_LOGI(TAG, "ADXL343 is not working!");


  for(;;) {

    adxl_read_raw_data(&accel);
    accelG = adxl_convert_to_g(&accel);
    printf("%f  %f  %f\n\r"accelG.x, accelG.y, accelG.z);
  }
}