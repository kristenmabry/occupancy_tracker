/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/


#include "platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint16_t status = 255;

        uint8_t data_write[2];

        data_write[0] = (RegisterAdress >> 8) & 0xff;
        data_write[1] = RegisterAdress & 0xff;

        status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);
	
	/* Need to be implemented by customer. This function returns 0 if OK */
        status |= nrf_drv_twi_rx(&(p_platform->m_twi), p_platform->address, p_value, sizeof(*p_value));  // recieve data from i2c bus

	return status;
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint16_t status = 255;
        uint8_t data_write[3];

        data_write[0] = (RegisterAdress >> 8) & 0xff;
        data_write[1] = RegisterAdress & 0xff;
        data_write[2] = value;

	/* Need to be implemented by customer. This function returns 0 if OK */
        status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 3, false);  // send data to i2c bus
        //status = WrMulti(p_platform, RegisterAdress, &value, 1);
	return status;
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;
	
		/* Need to be implemented by customer. This function returns 0 if OK */
        //uint32_t length = size;
        uint32_t i;
        uint8_t data_write[2];
        uint16_t temp_address = RegisterAdress;
        
         status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, true); // write register address
        status = 0;
        if(size >= 1){
          for(i = 0; i < size; i = i+1){
            status |= WrByte(p_platform, temp_address + i, p_values[i]);
            
          }
        }  

        //if(size < 255) {
        //  for(i = 0; i < size-255; i = i+255) {          
        //    //memset(data_write, 0, sizeof(data_write));    // clear array in case
        //    data_write[0] = (temp_address >> 8) & 0xff;   // Register address
        //    data_write[1] = temp_address & 0xff;          //
        //    status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, true);      // Address
        //    status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, &p_values[i], 255, false);  // I2C write data
        //    temp_address = temp_address + 255;  // update register address
        //    }
        //  data_write[0] = (temp_address >> 8) & 0xff;
        //  data_write[1] = temp_address & 0xff;
        //  status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);      // Address
        //  status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, &p_values[i], size-i, false);  // I2C write data
        //} else {
        //data_write[0] = (RegisterAdress >> 8) & 0xff;
        //data_write[1] = RegisterAdress & 0xff;
        //status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);
        //status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, p_values, size, false);  // I2C write data
        //}
	return status;
}

uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
        uint8_t data_write[2];
        uint16_t temp_address = RegisterAdress;
	uint32_t i = 0;

        status = 0;
        for(i = 0; i < size; i++){
          status |= RdByte(p_platform, temp_address + i, p_values + i);
        }


        //if(size >= 255){
        //  for(i = 0; i < size-255; i = i+255) {
        //    data_write[0] = (temp_address >> 8) & 0xff;   // Register address
        //    data_write[1] = temp_address & 0xff;          //            
        //    status |= nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);  // Register Address
        //    status |= nrf_drv_twi_rx(&(p_platform->m_twi), p_platform->address, &(p_values[i]), 255);   // I2C Read
        //    //temp_address = temp_address + 255;  // update register address
        //  }
        //    data_write[0] = (temp_address >> 8) & 0xff;   // Register address
        //    data_write[1] = temp_address & 0xff;          //            
        //    status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);  // Register Address
        //    status |= nrf_drv_twi_rx(&(p_platform->m_twi), p_platform->address, &(p_values[i]), 255);   // I2C Read
        //} else {
        //  data_write[0] = (temp_address >> 8) & 0xff;   // Register address
        //  data_write[1] = temp_address & 0xff;          //            
        //  status = nrf_drv_twi_tx(&(p_platform->m_twi), p_platform->address, data_write, 2, false);  // Register Address
        //  status |= nrf_drv_twi_rx(&(p_platform->m_twi), p_platform->address, p_values, size);   // I2C Read
        //  }
	return status;
}

uint8_t Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
        nrf_gpio_cfg_output(25);
        nrf_gpio_pin_clear(25);
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
        nrf_gpio_cfg_output(25);
        nrf_gpio_pin_set(25);
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);

	return status;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 0;

	/* Need to be implemented by customer. This function returns 0 if OK */
	nrf_delay_ms(TimeMs);

	return status;
}