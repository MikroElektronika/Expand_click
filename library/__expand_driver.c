/*
    __expand_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__expand_driver.h"
#include "__expand_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __EXPAND_DRV_I2C__
static uint8_t _slaveAddress;
#endif

const uint8_t _EXPAND_SPI_DEVICE_OPCODE                                 = 0x40;

const uint8_t _EXPAND_OPCODE_WRITE                                      = 0x00;
const uint8_t _EXPAND_OPCODE_READ                                       = 0x01;

const uint8_t _EXPAND_SPI_MODULE_POSITION_0                             = 0x00;
const uint8_t _EXPAND_SPI_MODULE_POSITION_1                             = 0x02;
const uint8_t _EXPAND_SPI_MODULE_POSITION_2                             = 0x04;
const uint8_t _EXPAND_SPI_MODULE_POSITION_3                             = 0x06;
const uint8_t _EXPAND_SPI_MODULE_POSITION_4                             = 0x08;
const uint8_t _EXPAND_SPI_MODULE_POSITION_5                             = 0x0A;
const uint8_t _EXPAND_SPI_MODULE_POSITION_6                             = 0x0C;
const uint8_t _EXPAND_SPI_MODULE_POSITION_7                             = 0x0E;

// Port Direction
const uint8_t _EXPAND_PORT_DIRECTION_OUTPUT                             = 0x00;
const uint8_t _EXPAND_PORT_DIRECTION_INPUT                              = 0xFF;

// BANK 1 register configuration
const uint8_t _EXPAND_IODIRA_BANK1                                      = 0x00;
const uint8_t _EXPAND_IPOLA_BANK1                                       = 0x01;
const uint8_t _EXPAND_GPINTENA_BANK1                                    = 0x02;
const uint8_t _EXPAND_DEFVALA_BANK1                                     = 0x03;
const uint8_t _EXPAND_INTCONA_BANK1                                     = 0x04;
const uint8_t _EXPAND_IOCON_BANK1                                       = 0x05;
const uint8_t _EXPAND_GPPUA_BANK1                                       = 0x06;
const uint8_t _EXPAND_INTFA_BANK1                                       = 0x07;
const uint8_t _EXPAND_INTCAPA_BANK1                                     = 0x08;
const uint8_t _EXPAND_GPIOA_BANK1                                       = 0x09;
const uint8_t _EXPAND_OLATA_BANK1                                       = 0x0A;
const uint8_t _EXPAND_IODIRB_BANK1                                      = 0x10;
const uint8_t _EXPAND_IPOLB_BANK1                                       = 0x11;
const uint8_t _EXPAND_GPINTENB_BANK1                                    = 0x12;
const uint8_t _EXPAND_DEFVALB_BANK1                                     = 0x13;
const uint8_t _EXPAND_INTCONB_BANK1                                     = 0x14;
const uint8_t _EXPAND_IOCONO_BANK1                                      = 0x15;
const uint8_t _EXPAND_GPPUB_BANK1                                       = 0x16;
const uint8_t _EXPAND_INTFB_BANK1                                       = 0x17;
const uint8_t _EXPAND_INTCAPB_BANK1                                     = 0x18;
const uint8_t _EXPAND_GPIOB_BANK1                                       = 0x19;
const uint8_t _EXPAND_OLATB_BANK1                                       = 0x1A;

// BANK 0 register configuration
const uint8_t _EXPAND_IODIRA_BANK0                                      = 0x00;
const uint8_t _EXPAND_IODIRB_BANK0                                      = 0x01;
const uint8_t _EXPAND_IPOLA_BANK0                                       = 0x02;
const uint8_t _EXPAND_IPOLB_BANK0                                       = 0x03;
const uint8_t _EXPAND_GPINTENA_BANK0                                    = 0x04;
const uint8_t _EXPAND_GPINTENB_BANK0                                    = 0x05;
const uint8_t _EXPAND_DEFVALA_BANK0                                     = 0x06;
const uint8_t _EXPAND_DEFVALB_BANK0                                     = 0x07;
const uint8_t _EXPAND_INTCONA_BANK0                                     = 0x08;
const uint8_t _EXPAND_INTCONB_BANK0                                     = 0x09;
const uint8_t _EXPAND_IOCON_BANK0                                       = 0x0A;
const uint8_t _EXPAND_GPPUA_BANK0                                       = 0x0C;
const uint8_t _EXPAND_GPPUB_BANK0                                       = 0x0D;
const uint8_t _EXPAND_INTFA_BANK0                                       = 0x0E;
const uint8_t _EXPAND_INTFB_BANK0                                       = 0x0F;
const uint8_t _EXPAND_INTCAPA_BANK0                                     = 0x10;
const uint8_t _EXPAND_INTCAPB_BANK0                                     = 0x11;
const uint8_t _EXPAND_GPIOA_BANK0                                       = 0x12;
const uint8_t _EXPAND_GPIOB_BANK0                                       = 0x13;
const uint8_t _EXPAND_OLATA_BANK0                                       = 0x14;
const uint8_t _EXPAND_OLATB_BANK0                                       = 0x15;

const uint8_t _EXPAND_IOCON_BYTE_MODE                                   = 0x20;
const uint8_t _EXPAND_IOCON_HAEN                                        = 0x08;

const uint8_t _EXPAND_CN4_PA0                                           = 0x01;
const uint8_t _EXPAND_CN4_PA1                                           = 0x02;
const uint8_t _EXPAND_CN4_PA2                                           = 0x04;
const uint8_t _EXPAND_CN4_PA3                                           = 0x08;
const uint8_t _EXPAND_CN4_PA4                                           = 0x10;
const uint8_t _EXPAND_CN4_PA5                                           = 0x20;
const uint8_t _EXPAND_CN4_PA6                                           = 0x40;
const uint8_t _EXPAND_CN4_PA7                                           = 0x80;

const uint8_t _EXPAND_CN5_PB0                                           = 0x01;
const uint8_t _EXPAND_CN5_PB1                                           = 0x02;
const uint8_t _EXPAND_CN5_PB2                                           = 0x04;
const uint8_t _EXPAND_CN5_PB3                                           = 0x08;
const uint8_t _EXPAND_CN5_PB4                                           = 0x10;
const uint8_t _EXPAND_CN5_PB5                                           = 0x20;
const uint8_t _EXPAND_CN5_PB6                                           = 0x40;
const uint8_t _EXPAND_CN5_PB7                                           = 0x80;

const uint8_t _EXPAND_CN_START_POSITION                                 = 0x01;

const uint8_t _EXPAND_INT_ERR                                           = 0xFF;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __EXPAND_DRV_SPI__

void expand_spiDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    hal_gpio_csSet( 1 );
}

#endif
#ifdef   __EXPAND_DRV_I2C__

void expand_i2cDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __EXPAND_DRV_UART__

void expand_uartDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif



/* ----------------------------------------------------------- IMPLEMENTATION */



/* Generic read one bayt from register function */
uint8_t expand_readByte( uint8_t modCmd, uint8_t regAddr )
{
    uint8_t bufferRead[ 2 ];
    uint8_t bufferWrite[ 2 ];

    bufferWrite[ 0 ] = _EXPAND_SPI_DEVICE_OPCODE | modCmd | _EXPAND_OPCODE_READ;
    bufferWrite[ 1 ] = regAddr;

    hal_gpio_csSet( 0 );
    hal_spiWrite( bufferWrite, 2 );
    hal_spiRead( bufferRead, 2 );
    hal_gpio_csSet( 1 );

    return bufferRead[ 1 ];
}

/* Generic write one bayt to register function */
void expand_writeByte( uint8_t modCmd, uint8_t regAddr, uint8_t writeData )
{
    uint8_t bufferWrite[ 4 ];

    bufferWrite[ 0 ] = _EXPAND_SPI_DEVICE_OPCODE | modCmd | _EXPAND_OPCODE_WRITE;
    bufferWrite[ 1 ] = regAddr;
    bufferWrite[ 2 ] = writeData;

    hal_gpio_csSet( 0 );
    hal_spiWrite( bufferWrite, 4 );
    hal_gpio_csSet( 1 );
}

/* Default configuration function */
void expand_defaultConfiguration( uint8_t modCmd )
{
    uint8_t check;
    
    check = 0;
    
    expand_writeByte( modCmd, _EXPAND_IOCON_BANK0, ( _EXPAND_IOCON_BYTE_MODE | _EXPAND_IOCON_HAEN ) );

    while ( !check )
    {
        expand_writePortA( 0x00, 0x01 );
        check = expand_readPortB( 0x00 );
    }
    
}

/* Set register bits function */
void expand_setBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand_readByte( modCmd, regAddr );

    temp |= bitMask;

    expand_writeByte( modCmd, regAddr, temp );
}

/*  Clear register beats function */
void expand_clearBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand_readByte( modCmd, regAddr );

    temp &= ~bitMask;

    expand_writeByte( modCmd, regAddr, temp );
}

/*  Toggle register beats function */
void expand_toggleBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand_readByte( modCmd, regAddr );

    temp ^= bitMask;

    expand_writeByte( modCmd, regAddr, temp );
}

/* Read one byte of data from PORTA function */
uint8_t expand_readPortA( uint8_t modCmd )
{
    return expand_readByte( modCmd, _EXPAND_GPIOA_BANK0 );
}

/* Read one byte of data from PORTB function */
uint8_t expand_readPortB( uint8_t modCmd )
{
    return expand_readByte( modCmd, _EXPAND_GPIOB_BANK0);
}

/* Read two byte of data from PORTA & PORTB function */
uint16_t expand_readBothPorta( uint8_t modCmd )
{
    uint16_t result;
    uint8_t buffer[ 2 ];

    buffer[ 0 ] = expand_readPortA( modCmd );
    buffer[ 1 ] = expand_readPortB( modCmd );

    result = buffer[ 0 ];
    result <<= 8;
    result |= buffer[ 1 ];

    return result;
}

/* Write one byte of data to register for PORTA function */
void expand_writePortA( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_OLATA_BANK0, writeData );
}

/* Clear bit to register for PORTA function */
void expand_clearBitPortA( uint8_t modCmd, uint8_t bitMask )
{
    expand_clearBits( modCmd, _EXPAND_OLATA_BANK0, bitMask );
}

/* Set bit to register for PORTA function */
void expand_setBitPortA( uint8_t modCmd, uint8_t bitMask )
{
    expand_setBits( modCmd, _EXPAND_OLATA_BANK0, bitMask );
}

/* Toggle bit to register for PORTA function */
void expand_toggleBitPortA( uint8_t modCmd, uint8_t bitMask )
{
    expand_toggleBits( modCmd, _EXPAND_OLATA_BANK0, bitMask );
}

/* Write one byte of data to register for PORTB function */
void expand_writePortB( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_OLATB_BANK0, writeData );
}

/* Clear bit to register for PORTB function */
void expand_clearBitPortB( uint8_t modCmd, uint8_t bitMask )
{
    expand_clearBits( modCmd, _EXPAND_OLATB_BANK0, bitMask );
}

/* Set bit to register for PORTB function */
void expand_setBitPortB( uint8_t modCmd, uint8_t bitMask )
{
    expand_setBits( modCmd, _EXPAND_OLATB_BANK0, bitMask );
}

/* Toggle bit to register for PORTB function */
void expand_toggleBitPortB( uint8_t modCmd, uint8_t bitMask )
{
    expand_toggleBits( modCmd, _EXPAND_OLATB_BANK0, bitMask );
}

/* Set expander PORTA direction function */
void expand_setDirectionPortA( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_INTCONA_BANK0, writeData );
}

/* Set expander PORTA input direction function */
void expand_setInputDirPortA( uint8_t modCmd, uint8_t bitMask )
{
    expand_setBits( modCmd, _EXPAND_IODIRA_BANK0, bitMask );
}

/* Set expander PORTA output direction function */
void expand_setOutputDirPortA( uint8_t modCmd, uint8_t bitMask )
{
    expand_clearBits( modCmd, _EXPAND_IODIRA_BANK0, bitMask );
}

/* Set expander PORTB direction function */
void expand_setDirectionPortB( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_INTCONB_BANK0, writeData );
}

/* Set expander PORTB input direction function */
void expand_setInputDirPortB( uint8_t modCmd, uint8_t bitMask )
{
    expand_setBits( modCmd, _EXPAND_IODIRB_BANK0, bitMask );
}

/* Set expander PORTB output direction function */
void expand_setOutputDirPortB( uint8_t modCmd, uint8_t bitMask )
{
    expand_clearBits( modCmd, _EXPAND_IODIRB_BANK0, bitMask );
}

/*  Set pull-ups of the expander for PORTA pins function */
void expand_setPullUpsPortA( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_GPPUA_BANK0, writeData );
}

/*  Set pull-ups of the expander for PORTB pins function */
void expand_setPullUpsPortB( uint8_t modCmd, uint8_t writeData )
{
    expand_writeByte( modCmd, _EXPAND_GPPUB_BANK0, writeData );
}

/*  Active pin by position on PORTA function */
void expand_setPotrA( uint8_t position )
{
    uint8_t writeData;

    position %= 8;

    writeData = 0x01 << position;

    expand_writePortA( _EXPAND_SPI_MODULE_POSITION_0, writeData );
}

/*  Active pin by position on PORTB function */
void expand_setPotrB( uint8_t position )
{
    uint8_t writeData;

    position %= 8;

    writeData = 0x01 << position;

    expand_writePortB( _EXPAND_SPI_MODULE_POSITION_0, writeData );
}

/* Reset function */
void expand_reset()
{
    hal_gpio_rstSet( 1 );
    Delay_100ms();
    hal_gpio_rstSet( 0 );
    Delay_100ms();
    hal_gpio_rstSet( 1 );
    Delay_100ms();
}

/* Get state of interrupt pin function */
uint8_t expand_getInterrupt()
{
    return hal_gpio_intGet();
}



/* -------------------------------------------------------------------------- */
/*
  __expand_driver.c

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