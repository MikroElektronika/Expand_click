/*
Example for Expand Click

    Date          : Nov 2018.
    Author        : Nenad Filipovic

Test configuration PIC32 :
    
    MCU                : P32MX795F512L
    Dev. Board         : EasyPIC Fusion v7
    PIC32 Compiler ver : v4.0.0.0

---

Description :

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

*/

#include "Click_Expand_types.h"
#include "Click_Expand_config.h"


uint8_t portStatus;
uint8_t position;
uint16_t pinPosition;
char logText[50];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT );

    mikrobus_spiInit( _MIKROBUS1, &_EXPAND_SPI_CFG[0] );

    mikrobus_logInit( _LOG_USBUART_A, 9600 );

    Delay_ms( 100 );
}

void applicationInit()
{
    expand_spiDriverInit( (T_EXPAND_P)&_MIKROBUS1_GPIO, (T_EXPAND_P)&_MIKROBUS1_SPI );

    expand_reset();
    Delay_ms( 1000 );
    
    mikrobus_logWrite( "----------------", _LOG_LINE );
    mikrobus_logWrite( "  Expand Click  ", _LOG_LINE );
    mikrobus_logWrite( "----------------", _LOG_LINE );

    expand_setDirectionPortA( _EXPAND_SPI_MODULE_POSITION_0, _EXPAND_PORT_DIRECTION_OUTPUT );
    
    expand_setDirectionPortB( _EXPAND_SPI_MODULE_POSITION_0, _EXPAND_PORT_DIRECTION_INPUT );
    
    mikrobus_logWrite( " Configuring... ", _LOG_LINE );
    mikrobus_logWrite( "----------------", _LOG_LINE );
    
    expand_defaultConfiguration( _EXPAND_SPI_MODULE_POSITION_0 );
}

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

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}