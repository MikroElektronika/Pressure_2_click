#line 1 "C:/Users/katarina.perendic/Desktop/Pressure_2_click/example/c/ARM/STM/Click_Pressure2_STM.c"
#line 1 "c:/users/katarina.perendic/desktop/pressure_2_click/example/c/arm/stm/click_pressure2_types.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for arm/include/stdint.h"





typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int int32_t;
typedef signed long long int64_t;


typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
typedef unsigned long long uint64_t;


typedef signed char int_least8_t;
typedef signed int int_least16_t;
typedef signed long int int_least32_t;
typedef signed long long int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
typedef unsigned long long uint_least64_t;



typedef signed long int int_fast8_t;
typedef signed long int int_fast16_t;
typedef signed long int int_fast32_t;
typedef signed long long int_fast64_t;


typedef unsigned long int uint_fast8_t;
typedef unsigned long int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
typedef unsigned long long uint_fast64_t;


typedef signed long int intptr_t;
typedef unsigned long int uintptr_t;


typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
#line 1 "c:/users/katarina.perendic/desktop/pressure_2_click/example/c/arm/stm/click_pressure2_config.h"
#line 1 "c:/users/katarina.perendic/desktop/pressure_2_click/example/c/arm/stm/click_pressure2_types.h"
#line 4 "c:/users/katarina.perendic/desktop/pressure_2_click/example/c/arm/stm/click_pressure2_config.h"
const uint32_t _PRESSURE2_SPI_CFG[ 2 ] =
{
 _SPI_FPCLK_DIV256,
 _SPI_FIRST_CLK_EDGE_TRANSITION |
 _SPI_CLK_IDLE_LOW |
 _SPI_MASTER |
 _SPI_MSB_FIRST |
 _SPI_8_BIT |
 _SPI_SSM_ENABLE |
 _SPI_SS_DISABLE |
 _SPI_SSI_1
};
#line 1 "c:/users/katarina.perendic/desktop/pressure_2_click/library/__pressure2_driver.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for arm/include/stdint.h"
#line 58 "c:/users/katarina.perendic/desktop/pressure_2_click/library/__pressure2_driver.h"
extern const uint8_t _PRESSURE2_CMD_RESET;
extern const uint8_t _PRESSURE2_CMD_ADC_READ;
extern const uint8_t _PRESSURE2_CMD_ADC_CONV;
extern const uint8_t _PRESSURE2_CMD_ADC_D1;
extern const uint8_t _PRESSURE2_CMD_ADC_D2;
extern const uint8_t _PRESSURE2_CMD_ADC_256;
extern const uint8_t _PRESSURE2_CMD_ADC_512;
extern const uint8_t _PRESSURE2_CMD_ADC_1024;
extern const uint8_t _PRESSURE2_CMD_ADC_2048;
extern const uint8_t _PRESSURE2_CMD_ADC_4096;
extern const uint8_t _PRESSURE2_CMD_PROM_RD;


extern const uint8_t _PRESSURE2_TRUE;
extern const uint8_t _PRESSURE2_FALSE;
#line 82 "c:/users/katarina.perendic/desktop/pressure_2_click/library/__pressure2_driver.h"
void pressure2_spiDriverInit( const uint8_t*  gpioObj,  const uint8_t*  spiObj);
#line 92 "c:/users/katarina.perendic/desktop/pressure_2_click/library/__pressure2_driver.h"
void pressure2_gpioDriverInit( const uint8_t*  gpioObj);




void pressure2_reset();


void pressure2_readData( uint8_t rAddr, uint8_t *buffer, uint8_t nBytes );


void pressure2_writeBytes( uint8_t cmd );


uint16_t pressure2_readCoefficient( uint8_t index );


uint32_t pressure2_sendCmdADC( uint8_t cmd );


uint8_t pressure2_init();


void pressure2_readSensor( float *P, float *T );
#line 30 "C:/Users/katarina.perendic/Desktop/Pressure_2_click/example/c/ARM/STM/Click_Pressure2_STM.c"
float pressure_P;
float pressure_T;
char demoText[ 50 ];

void systemInit()
{
 mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
 mikrobus_spiInit( _MIKROBUS1, &_PRESSURE2_SPI_CFG[0] );
 mikrobus_logInit( _LOG_USBUART_A, 9600 );
 Delay_ms( 100 );
}

void applicationInit()
{
 pressure2_spiDriverInit( ( const uint8_t* )&_MIKROBUS1_GPIO, ( const uint8_t* )&_MIKROBUS1_SPI );
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
