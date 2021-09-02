/**/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres                                              */
/*    @file    adxl343.c                                                         */
/*    @version v1                                                                */
/*    @date    01-September-2020                                                 */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the ADXL343 accelerometer                    */
/*             driver using ESP32 SPI                                            */
/*********************************************************************************/
/**/

#include "adxl343.h"
#include <math.h>

static const char *TAG = "ADXL343";
static esp_err_t ret;

uint8_t adxl_init_device(){

    ESP_LOGI(TAG, "Initializing SPI");

    ret = SPI_init(ADXL343);
    ESP_ERROR_CHECK(ret);

    uint8_t adxl_device_id = adxl_get_id();
    if(adxl_device_id == ADXL343_DEVICE_ID)  
        ESP_LOGI(TAG, "ADXL343 online!");
    else{
        ESP_LOGE(TAG, "ADXL343 not found!");
        return (false);
    }

    adxl_full_res_bit(true);
    adxl_range_setting(RANGE_16G);
    adxl_set_bw(ADXL343_BW_1600);
    uint8_t dat = 0x0B;
    spi_reg_write(ADXL343_DATA_FORMAT, &dat, 1);
    dat = 0x0F;
    spi_reg_write(0x2C, &dat, 1);    
    adxl_operation_mode(FULL_BW_MEASUREMENT);

    //read_all();
    return (adxl_device_id);
}

uint8_t adxl_get_id(){

    uint8_t chip_id;
    if(!(spi_reg_read(ADXL343_DEVID, &chip_id, 1)))
        return (false);

    return (chip_id);
}

uint8_t adxl_update_reg(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t data){

    uint8_t reg_data;
    bool err = spi_reg_read(reg, &reg_data, 1);

    if (!err)
        return false;    

    reg_data &= ~mask;
    reg_data |= (data << shift) & ~mask;
    err = spi_reg_write(reg, &reg_data, 1);

    if (!err)
      return false;

    return reg_data;   
}

bool adxl_operation_mode(ADXL343_OP_MODE mode){

    adxl_update_reg(ADXL343_POWER_CTL, PWRCTRL_OPMODE_MASK,
                     3, mode);

    return true;
}

bool adxl_range_setting(ADXL343_RANGE range){

    adxl_update_reg(ADXL343_DATA_FORMAT, DFRMT_RANGE_MASK,
                     0, range);

    return true;
}

bool adxl_full_res_bit(bool res_bit){
    uint8_t dat = 0x0F;
    adxl_update_reg(ADXL343_DATA_FORMAT, DFRMT_FULLRESBIT_MASK,
                     3, res_bit);
    spi_reg_write(ADXL343_DATA_FORMAT, &dat, 1);
    return true;    
}

bool adxl_set_bw(uint8_t bw){

    adxl_update_reg(ADXL343_BW_RATE, BW_RATE_MASK,
                     0, bw);

    return true;      
}

void read_all(){

    uint8_t data;

    for(int i = 0; i < 0x40; i++){
        spi_reg_read((uint8_t)i, &data, 1);
        printf("%02X: %02X\n",i, data);
    }
}

void adxl_read_raw_data(struct ADXL343_Accel *triplet) {

    uint8_t data_ready = 0;
    uint8_t buff[6];

    do {
        data_ready = get_data_ready();
    } while (!data_ready);

    spi_reg_read((ADXL343_DATAX0 | ADXL_SPI_MULTB_FLAG), buff, 6);

    triplet->x = ((int16_t)buff[1] << 8) + buff[0];
    triplet->y = ((int16_t)buff[3] << 8) + buff[2];
    triplet->z = ((int16_t)buff[5] << 8) + buff[4];
}

struct ADXL343_AccelG adxl_convert_to_g(const struct ADXL343_Accel *triplet) {

    struct ADXL343_AccelG triplet_g = {0, 0, 0};

    triplet_g.x = triplet->x * SCALE_MULTIPLIER;
    triplet_g.y = triplet->y * SCALE_MULTIPLIER;
    triplet_g.z = triplet->z * SCALE_MULTIPLIER;

    return (triplet_g);
}

uint8_t get_data_ready(){

  uint8_t readyData;
  spi_reg_read(ADXL343_INT_SOURCE, &readyData, 1);
  uint8_t value = ((readyData & 0x80) >> 7);

  return (value);
}