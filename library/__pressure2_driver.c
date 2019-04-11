/*
    __pressure2_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__pressure2_driver.h"
#include "__pressure2_hal.c"

/* ------------------------------------------------------------------- MACROS */

// Sensor constants:
const uint8_t _PRESSURE2_CMD_RESET       =  0x1E;     // ADC reset command
const uint8_t _PRESSURE2_CMD_ADC_READ    =  0x00;     // ADC read command
const uint8_t _PRESSURE2_CMD_ADC_CONV    =  0x40;     // ADC conversion command
const uint8_t _PRESSURE2_CMD_ADC_D1      =  0x00;     // ADC D1 conversion
const uint8_t _PRESSURE2_CMD_ADC_D2      =  0x10;     // ADC D2 conversion
const uint8_t _PRESSURE2_CMD_ADC_256     =  0x00;     // ADC OSR=256
const uint8_t _PRESSURE2_CMD_ADC_512     =  0x02;     // ADC OSR=512
const uint8_t _PRESSURE2_CMD_ADC_1024    =  0x04;     // ADC OSR=1024
const uint8_t _PRESSURE2_CMD_ADC_2048    =  0x06;     // ADC OSR=2056
const uint8_t _PRESSURE2_CMD_ADC_4096    =  0x08;     // ADC OSR=4096
const uint8_t _PRESSURE2_CMD_PROM_RD     =  0xA0;     // Prom read command

const uint8_t _PRESSURE2_TRUE            =  1;
const uint8_t _PRESSURE2_FALSE           =  0;

/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __PRESSURE2_DRV_I2C__
static uint8_t _slaveAddress;
#endif

static uint16_t _sensorCoefficients[8];
static uint32_t _pressure = 0;
static uint32_t _temperature = 0;
static uint32_t _deltaTemp = 0;
static float _sensorOffset = 0;
static float _sensitivity  = 0;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

static double _pow( double x, uint8_t y );
static uint8_t _crcMS5803();

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

static double _pow( double x, uint8_t y )
{
    double res = 1; // Initialize result

    while (y > 0)
    {
        // If y is odd, multiply x with result
        if (y & 1)
        {
            res = res * x;
        }
        // n must be even now
        y = y >> 1; // y = y/2
        x = x * x; // Change x to x^2
    }
    return res;
}

static uint8_t _crcMS5803()
{
    uint16_t cnt;                  // Simple counter
    uint16_t n_rem;                // Crc reminder
    uint16_t crc_read;             // Original value of the crc
    uint8_t  n_bit;

    n_rem = 0x00;
    crc_read = _sensorCoefficients[7];       // Save read CRC
    _sensorCoefficients[7] = ( 0xFF00 & ( _sensorCoefficients[7] ) );   // CRC byte is replaced by 0
  
    for ( cnt = 0; cnt < 16; cnt++ )        // Operation is performed on bytes
    {
        // choose LSB or MSB
        if ( cnt % 2 == 1 )
        {
            n_rem ^= ( uint8_t ) ( ( _sensorCoefficients[cnt>>1] ) & 0x00FF );
        }
        else
        {
            n_rem ^= ( uint8_t ) ( _sensorCoefficients[ cnt >> 1 ] >> 8 );
        }
        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else
            {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    n_rem = ( 0x000F & ( n_rem >> 12 ) );    // Final 4-bit reminder is CRC code
    _sensorCoefficients[7] = crc_read;        // Restore the crc_read to its original place

    return ( n_rem ^ 0x00 );                 // The calculated CRC should match what the device initally returned.
}


/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __PRESSURE2_DRV_SPI__

void pressure2_spiDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    hal_gpio_csSet( 1 );
}

#endif
#ifdef   __PRESSURE2_DRV_I2C__

void pressure2_i2cDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
}

#endif
#ifdef   __PRESSURE2_DRV_UART__

void pressure2_uartDriverInit(T_PRESSURE2_P gpioObj, T_PRESSURE2_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

void pressure2_reset()
{
    pressure2_writeBytes( _PRESSURE2_CMD_RESET );
    Delay_1ms();
}

void pressure2_readData( uint8_t rAddr, uint8_t *buffer, uint8_t nBytes )
{
    uint8_t regAddr[256] = {0};
    regAddr[0] = rAddr;

    hal_gpio_csSet( 0 );
    hal_spiTransfer( regAddr, buffer, nBytes + 1 );
    hal_gpio_csSet( 1 );
}

void pressure2_writeBytes( uint8_t cmd )
{
    uint8_t wData;
    wData = cmd;

    hal_gpio_csSet( 0 );
    hal_spiWrite( &wData, 1 );
    hal_gpio_csSet( 1 );
}

uint16_t pressure2_readCoefficient( uint8_t index )
{
    uint16_t result = 0;
    uint8_t resultFirst[3] = {0};
    pressure2_readData( _PRESSURE2_CMD_PROM_RD + ( index * 2 ), resultFirst, 2 );
    result = resultFirst[1] << 8;
    result |= resultFirst[2];

    return result;
}

uint32_t pressure2_sendCmdADC( uint8_t cmd )
{
    uint32_t result = 0;
    uint8_t adc_read[4] = {0};
    pressure2_writeBytes( _PRESSURE2_CMD_ADC_CONV + cmd ); // Send conversion command
    Delay_1ms();
    switch ( cmd & 0x0F )
    {
        case _PRESSURE2_CMD_ADC_256 :
        { 
            Delay_500us();
            Delay_80us();
            Delay_80us();
            Delay_80us();
            Delay_80us();
            Delay_80us();
            break;
        }
        case _PRESSURE2_CMD_ADC_512 :
        { 
            Delay_1ms();
            Delay_1ms();
            Delay_1ms();
            break;
        }
        case _PRESSURE2_CMD_ADC_1024 :
        {
            Delay_1ms();
            Delay_1ms();
            Delay_1ms();
            Delay_1ms();
            break;
        }
        case _PRESSURE2_CMD_ADC_2048 : 
        {
            Delay_8ms();
            break;
        }
        case _PRESSURE2_CMD_ADC_4096 :
        {
            Delay_10ms();
            Delay_1ms();
            Delay_1ms();
            Delay_1ms();
            Delay_1ms();
            break;
        }
    }
    pressure2_readData( _PRESSURE2_CMD_ADC_READ, adc_read, 3 );
    result = ( (uint32_t) adc_read[1] << 16 );
    result |= ( (uint16_t) adc_read[2] << 8 );
    result |= adc_read[3];

    return result;
}

uint8_t pressure2_init()
{
    uint8_t p_crc, n_crc;
    uint8_t i = 0;
    pressure2_reset();
    Delay_1ms();
  
    for ( i = 0; i < 8; i++ )
    {
        _sensorCoefficients[ i ] = pressure2_readCoefficient( i );
    }
    p_crc = _sensorCoefficients[ 7 ];
    n_crc = _crcMS5803();

    if( p_crc != n_crc )
    {
        return _PRESSURE2_FALSE;
    }
    else
    {
        return _PRESSURE2_TRUE;
    }
}


/** Read sensor */
void pressure2_readSensor( float *P, float *T )
{
    _pressure = pressure2_sendCmdADC( _PRESSURE2_CMD_ADC_D1 + _PRESSURE2_CMD_ADC_4096 );
    _temperature = pressure2_sendCmdADC( _PRESSURE2_CMD_ADC_D2 + _PRESSURE2_CMD_ADC_4096 );

    _deltaTemp = _temperature - _sensorCoefficients[5] * _pow( 2, 8 );
    _sensorOffset = _sensorCoefficients[2] * _pow( 2, 16 ) + ( _deltaTemp / _pow( 2, 7 ) ) * ( _sensorCoefficients[4] / _pow( 2, 7 ) );
    _sensitivity = _sensorCoefficients[1] * _pow( 2, 15 ) + ( _deltaTemp / _pow( 2, 8 ) ) * ( _sensorCoefficients[3] / _pow( 2, 8 ) );

    *P = ( ( _pressure / _pow( 2, 15 ) ) * ( _sensitivity / _pow( 2, 21 ) ) - ( _sensorOffset / _pow( 2, 15 ) ) ) / 10;
    *T = ( 2000 + ( ( _deltaTemp / _pow( 2, 13 ) ) * ( _sensorCoefficients[6] / _pow( 2, 10 ) ) )  ) / 100;
}





/* -------------------------------------------------------------------------- */
/*
  __pressure2_driver.c

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