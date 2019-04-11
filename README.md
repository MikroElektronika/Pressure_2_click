![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Pressure2 Click

- **CIC Prefix**  : PRESSURE2
- **Author**      : MikroE Team
- **Verison**     : 1.0.0
- **Date**        : avg 2018.

---


### Software Support

We provide a library for the Pressure2 Click on our [LibStock](https://libstock.mikroe.com/projects/view/1420/pressure-2-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library initializes and defines the SPI bus driver and drivers that offer a choice for writing data in address and reads data from address.
The library includes function for read Temperature data and Pressure data.
The user also has the function for initializes and reset chip, sends ADC command, and function for read Coefficient.

Key functions :

- ``` uint8_t pressure2_init() ``` - Function for initialization chip
- ``` void pressure2_readSensor( float *P, float *T ) ``` - Function for reads Pressure and Temperature data

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes SPI module and sets CS pin as OUTPUT.
- Application Initialization - Initializes driver init and chip init.
- Application Task - (code snippet) - Reads sensor and logs to USBUART pressure and temperature every second.

```.c
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
```

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/1420/pressure-2-click) page.

Other mikroE Libraries used in the example:

- SPI

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
