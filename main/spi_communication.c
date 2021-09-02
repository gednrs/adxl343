/**/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres                                              */
/*    @file    spi_communication.c                                               */
/*    @version v1                                                                */
/*    @date    01-September-2020                                                 */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the spi of the ADXL343                       */
/*             accelerometer via hardware in ESP32                               */
/*********************************************************************************/
/**/

#include "spi_communication.h"

/**
 * @brief   starts and configures the spi
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @return   ret
 */
esp_err_t SPI_init(spi_accel_t accel){
	
	esp_err_t ret;
    
	spi_bus_config_t buscfg;
	memset(&buscfg, 0, sizeof(buscfg));
	buscfg.mosi_io_num = GPIO_MOSI;
	buscfg.miso_io_num = GPIO_MISO;
	buscfg.sclk_io_num = GPIO_SCLK;
    //buscfg.quadwp_io_num = -1;
	//buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 1;
	buscfg.flags = SPICOMMON_BUSFLAG_MASTER ; //| SPICOMMON_BUSFLAG_IOMUX_PINS ;
	buscfg.intr_flags = 0;

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));

    if(accel == IIS3DWB){ 
        devcfg.mode = 0;                   // SPI_MODE 0 to IIS3DWB
        devcfg.clock_speed_hz = 10000000;  // this is for 10MHz
        devcfg.input_delay_ns = 30;        // INPUT_DELAY_NS;
    }
    else{
        devcfg.mode = 3;                   // SPI_MODE 3 to ADXL343;
        devcfg.clock_speed_hz = 5000000;   // this is for 5MHz to ADXL343
        //devcfg.input_delay_ns = 30;      // INPUT_DELAY_NS;        
    }
	devcfg.spics_io_num = GPIO_CS;
	devcfg.queue_size = 6;

	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0); // No DMA 
	ESP_ERROR_CHECK(ret);
	ret =  spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);

	return ret;
}

/**
 * @brief   free spi bus
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @return   ret
 */
esp_err_t SPI_deinit()
{
    esp_err_t ret;
    ret = spi_bus_remove_device(spi);
	ESP_ERROR_CHECK(ret);    
    spi = NULL;
    ret = spi_bus_free(HSPI_HOST);
	ESP_ERROR_CHECK(ret);    

    return ret;
}
/**
 * @brief   Direct read from register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   reg      address of the first register to be read
 * @param   data     pointer to the data to be read from the register
 * @param   len      number of bytes to be read from the register
 * @return           true on success, false on error
 */
bool spi_reg_read(uint8_t reg, uint8_t *data, uint16_t len){

    uint8_t addr;

  //  if(len == 1)
        addr = reg | ADXL_SPI_READ_FLAG;
   // else
     //   addr = reg | ADXL_SPI_READ_FLAG | ADXL_SPI_MULTB_FLAG;
    
    static uint8_t mosi[ADXL_SPI_BUF_SIZE];
    static uint8_t miso[ADXL_SPI_BUF_SIZE];

    memset (mosi, 0xFF, ADXL_SPI_BUF_SIZE);
    memset (miso, 0xFF, ADXL_SPI_BUF_SIZE);

    mosi[0] = addr;
    
    if (!spi_transfer_data (mosi, miso, len + 1)){

        printf("Could not read data to SPI!");
        return false;
    }

    /* Shift data one by left, first byte received while sending register address is invalid */
    for (int i = 0; i < len; i++)
      data[i] = miso[i + 1];

    return true;
}

/**
 * @brief   Direct write to register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   reg      address of the first register to be changed
 * @param   data     pointer to the data to be written to the register
 * @param   len      number of bytes to be written to the register
 * @return           true on success, false on error
 */
bool spi_reg_write(uint8_t reg, uint8_t *data, uint16_t len){

    uint8_t addr = (reg & 0x3f) | ADXL_SPI_WRITE_FLAG;
    static uint8_t mosi[ADXL_SPI_BUF_SIZE];

    reg &= 0x7F;

    mosi[0] = addr;

    // shift data one byte right, first byte in output is the register address
    for (int i = 0; i < len; i++)
        mosi[i+1] = data[i];


    if (!spi_transfer_data (mosi, NULL, len+1)){
        printf("Could not write data to SPI!");
        return false;
    }

    return true;
}

/**
 * @brief  transfer by spi
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   mosi     pointer to the data for the mosi
 * @param   miso     pointer to the data for the miso
 * @param   len      number of bytes to be written to the register
 * @return           len on success, false on error
 */
size_t spi_transfer_data(const uint8_t *mosi, uint8_t *miso, uint16_t len){

    spi_transaction_t spi_trans;

    memset(&spi_trans, 0, sizeof(spi_trans)); // zero out spi_trans
    spi_trans.tx_buffer = mosi;
    spi_trans.rx_buffer = miso;
    spi_trans.length = len * 8; // len in bytes, spi_trans.length in bits
    
    if (spi_device_transmit(spi, &spi_trans) != ESP_OK){
        printf("Could not transfer data to SPI!");
        return false;
    }
    return len;
}
