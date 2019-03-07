![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Expand Click

---

- **CIC Prefix**  : EXPAND
- **Author**      : Nenad Filipovic
- **Verison**     : 1.0.0
- **Date**        : Nov 2018.

---

### Software Support

We provide a library for the Expand Click on our [LibStock](https://libstock.mikroe.com/projects/view/212/expand-click-example) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library covers all the necessary functions to control Expand Click board.
Expand click communicates with the target board via SPI protocol. 
This library contains drivers for write and read data from MCP23S17 chip,
set PORTA/B direction, get PORTA/B direction, 
set PORTA/B status, get PORTA/B status, etc.

Key functions :

- ``` void expand_writeByte( uint8_t modCmd, uint8_t regAddr, uint8_t writeData ) ``` - Generic write one bayt to register function
- ``` void expand_writePortA( uint8_t modCmd, uint8_t writeData ) ``` - Write one byte of data to register for PORTA function
- ``` uint8_t expand_readPortB( uint8_t modCmd ) ``` - Read one byte of data from PORTB function

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes GPIO and LOG structures, 
     set CS and RST pins as output, INT pin as input.
- Application Initialization - Initialization driver enable's - GPIO,
     reset MCP23S17 chip, set PORTA to be output and PORTB to be input,
     set default configuration and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of Expand Click board.
     Expand Click communicates with register via SPI protocol by write and read from register,
     set configuration and state and read configuration and state.
     Results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 3 sec.


```.c

void applicationTask()
{
    pinPosition = 1;

    for ( position = 0; position < 8; position++ )
    {
        expand_writePortA( _EXPAND_SPI_MODULE_POSITION_0, pinPosition );
        Delay_100ms();

        portStatus = expand_readPortB( _EXPAND_SPI_MODULE_POSITION_0 );

        IntToStr( position , logText );
        mikrobus_logWrite( "       RA", _LOG_TEXT );
        ltrim( logText );
        mikrobus_logWrite( logText, _LOG_LINE );
        
        IntToStr( portStatus , logText );
        mikrobus_logWrite( "  PORTB: ", _LOG_TEXT );
        mikrobus_logWrite( logText, _LOG_LINE );
        mikrobus_logWrite( "----------------", _LOG_LINE );
        
        pinPosition <<= 1;

        Delay_ms( 3000 );
    }
}

```


The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/212/expand-click-example) page.

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
