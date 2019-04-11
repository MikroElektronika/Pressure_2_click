/*
Example for Pressure2 Click

    Date          : avg 2018.
    Author        : MikroE Team

Test configuration FT90x :
    
    MCU                : FT900
    Dev. Board         : EasyFT90x v7 
    FT90x Compiler ver : v2.3.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes SPI module and sets CS pin as OUTPUT.
- Application Initialization - Initializes driver init and chip init.
- Application Task - (code snippet) - Reads sensor and logs to USBUART pressure and temperature every second.

*/

#include "Click_Pressure2_types.h"
#include "Click_Pressure2_config.h"

float pressure_P;
float pressure_T;
char demoText[ 50 ];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_spiInit( _MIKROBUS1, &_PRESSURE2_SPI_CFG[0] );
    mikrobus_logInit( _LOG_USBUART, 9600 );
    Delay_ms( 100 );
}

void applicationInit()
{
    pressure2_spiDriverInit( (T_PRESSURE2_P)&_MIKROBUS1_GPIO, (T_PRESSURE2_P)&_MIKROBUS1_SPI );
    pressure2_init();
    Delay_ms( 300 );
}

void applicationTask()
{
    pressure2_readSensor( &pressure_P, &pressure_T );
    
    mikrobus_logWrite( "Pressure: ", _LOG_TEXT );
    FloatToStr( pressure_P, demoText );
    demoText[6] = 0;
    mikrobus_logWrite( demoText, _LOG_LINE );
    
    mikrobus_logWrite( "Temperature: ", _LOG_TEXT );
    FloatToStr( pressure_T, demoText );
    demoText[4] = 0;
    mikrobus_logWrite( demoText, _LOG_LINE );
    mikrobus_logWrite( "-----------", _LOG_LINE );
    Delay_ms( 1000 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}