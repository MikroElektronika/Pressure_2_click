/*
    __pressure2_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __pressure2_driver.h
@brief    Pressure2 Driver
@mainpage Pressure2 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   PRESSURE2
@brief      Pressure2 Click Driver
@{

| Global Library Prefix | **PRESSURE2** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **avg 2018.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _PRESSURE2_H_
#define _PRESSURE2_H_

/** 
 * @macro T_PRESSURE2_P
 * @brief Driver Abstract type 
 */
#define T_PRESSURE2_P    const uint8_t*

/** @defgroup PRESSURE2_COMPILE Compilation Config */              /** @{ */

   #define   __PRESSURE2_DRV_SPI__                            /**<     @macro __PRESSURE2_DRV_SPI__  @brief SPI driver selector */
//  #define   __PRESSURE2_DRV_I2C__                            /**<     @macro __PRESSURE2_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __PRESSURE2_DRV_UART__                           /**<     @macro __PRESSURE2_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup PRESSURE2_VAR Variables */                           /** @{ */

// Sensor constants:
extern const uint8_t _PRESSURE2_CMD_RESET;          // ADC reset command
extern const uint8_t _PRESSURE2_CMD_ADC_READ;       // ADC read command
extern const uint8_t _PRESSURE2_CMD_ADC_CONV;       // ADC conversion command
extern const uint8_t _PRESSURE2_CMD_ADC_D1;         // ADC D1 conversion
extern const uint8_t _PRESSURE2_CMD_ADC_D2;         // ADC D2 conversion
extern const uint8_t _PRESSURE2_CMD_ADC_256;        // ADC OSR=256
extern const uint8_t _PRESSURE2_CMD_ADC_512;        // ADC OSR=512
extern const uint8_t _PRESSURE2_CMD_ADC_1024;       // ADC OSR=1024
extern const uint8_t _PRESSURE2_CMD_ADC_2048;       // ADC OSR=2056
extern const uint8_t _PRESSURE2_CMD_ADC_4096;       // ADC OSR=4096
extern const uint8_t _PRESSURE2_CMD_PROM_RD;        // Prom read command

// true or false
extern const uint8_t _PRESSURE2_TRUE;
extern const uint8_t _PRESSURE2_FALSE;

                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup PRESSURE2_INIT Driver Initialization */              /** @{ */

#ifdef   __PRESSURE2_DRV_SPI__
void pressure2_spiDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P spiObj);
#endif
#ifdef   __PRESSURE2_DRV_I2C__
void pressure2_i2cDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P i2cObj, uint8_t slave);
#endif
#ifdef   __PRESSURE2_DRV_UART__
void pressure2_uartDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void pressure2_gpioDriverInit(T_PRESSURE2_P gpioObj);
                                                                       /** @} */
/** @defgroup PRESSURE2_FUNC Driver Functions */                   /** @{ */

/** Functions for reset chip */
void pressure2_reset();

/** Reads multiple registries depending on the specified number of bytes */
void pressure2_readData( uint8_t rAddr, uint8_t *buffer, uint8_t nBytes );

/** Sends command */
void pressure2_writeBytes( uint8_t cmd );

/** Read calibration coefficients and return coefficient */
uint16_t pressure2_readCoefficient( uint8_t index );

/** Preform ADC conversion and return 24bit result */
uint32_t pressure2_sendCmdADC( uint8_t cmd );

/** Initialize sensor */
uint8_t pressure2_init();

/** Read sensor */
void pressure2_readSensor( float *P, float *T );








                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Pressure2_STM.c
    @example Click_Pressure2_TIVA.c
    @example Click_Pressure2_CEC.c
    @example Click_Pressure2_KINETIS.c
    @example Click_Pressure2_MSP.c
    @example Click_Pressure2_PIC.c
    @example Click_Pressure2_PIC32.c
    @example Click_Pressure2_DSPIC.c
    @example Click_Pressure2_AVR.c
    @example Click_Pressure2_FT90x.c
    @example Click_Pressure2_STM.mbas
    @example Click_Pressure2_TIVA.mbas
    @example Click_Pressure2_CEC.mbas
    @example Click_Pressure2_KINETIS.mbas
    @example Click_Pressure2_MSP.mbas
    @example Click_Pressure2_PIC.mbas
    @example Click_Pressure2_PIC32.mbas
    @example Click_Pressure2_DSPIC.mbas
    @example Click_Pressure2_AVR.mbas
    @example Click_Pressure2_FT90x.mbas
    @example Click_Pressure2_STM.mpas
    @example Click_Pressure2_TIVA.mpas
    @example Click_Pressure2_CEC.mpas
    @example Click_Pressure2_KINETIS.mpas
    @example Click_Pressure2_MSP.mpas
    @example Click_Pressure2_PIC.mpas
    @example Click_Pressure2_PIC32.mpas
    @example Click_Pressure2_DSPIC.mpas
    @example Click_Pressure2_AVR.mpas
    @example Click_Pressure2_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __pressure2_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */