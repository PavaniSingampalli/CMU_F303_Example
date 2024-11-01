/**
  ******************************************************************************
  * @file           BMS_SPI.c
  * @brief          SPI protocol handling source file
  ******************************************************************************
  */

#include "BQ76942SPI.h"

#define SPI_used       SPI3

SPI_Handle_t hSPIx;  //handler for SPI basic param structure
uint8_t Rx_data[] = {0x00,0x00,0x00};

/**
 * @brief  function to write BQ769x2 registers over SPI
 * @param  reg_addr address of register to be written
 * @param  reg_data pointer to data which is to be written
 * @param  count number of data bytes to be written
 * @retval None
 * @note   includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
 */
void SPI_WriteReg(uint8_t reg_addr,uint8_t* reg_data,uint8_t count){
    uint8_t addr = 0x80 | reg_addr;      //set R/W bit high for writing + 7bit address
    uint8_t TX_Buffer[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned int i;
	unsigned int match = 0;
	unsigned int retries = 10;
	hSPIx.Instance = SPI_used;

    for(i=0; i<count; i++) {
    	TX_Buffer[0] = addr;
		TX_Buffer[1] = reg_data[i];

		HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,0);
		HAL_SPI_TransmitReceive(&hspi3, TX_Buffer, Rx_data, 2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,1);

		while ((match == 0) & (retries > 0)) {
			HAL_Delay(5);
			HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,0);
			HAL_SPI_TransmitReceive(&hspi3, TX_Buffer, Rx_data, 2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,1);
			if ((Rx_data[0] == addr) & (Rx_data[1] == reg_data[i])){
				match = 1;
			}
			retries --;
		}

	    match = 0;
	    addr += 1;
	    HAL_Delay(5);
	  }

}

/**
 * @brief  function to read BQ769x2 registers over SPI
 * @param  reg_addr address of register to read from
 * @param  reg_data pointer to data to store read bytes
 * @param  count number of data bytes to be read
 * @retval None
 * @note   includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
 */
void SPI_ReadReg(uint8_t reg_addr,uint8_t* reg_data,uint8_t count){
    uint8_t addr;
    uint8_t TX_Buffer[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    unsigned int i;
    unsigned int match;
    unsigned int retries = 10;

    match = 0;
    addr = reg_addr;

    for(i=0; i<count; i++) {
		TX_Buffer[0] = addr;
		TX_Buffer[1] = 0xFF;

		HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,0);
		HAL_SPI_TransmitReceive(&hspi3, TX_Buffer, Rx_data, 2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,1);

		while ((match == 0) & (retries > 0)) {
			HAL_Delay(5);
			HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,0);
			HAL_SPI_TransmitReceive(&hspi3, TX_Buffer, Rx_data, 2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(CMU_CS_GPIO_Port,CMU_CS_Pin,1);
			if (Rx_data[0] == addr) {
				match = 1;
				reg_data[i] = Rx_data[1];
			}
			retries --;
		}
	match = 0;
	addr += 1;
	HAL_Delay(5);
  }
}
