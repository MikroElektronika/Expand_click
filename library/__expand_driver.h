/*
    __expand_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __expand_driver.h
@brief    Expand Driver
@mainpage Expand Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   EXPAND
@brief      Expand Click Driver
@{

| Global Library Prefix | **EXPAND** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Nov 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _EXPAND_H_
#define _EXPAND_H_

/** 
 * @macro T_EXPAND_P
 * @brief Driver Abstract type 
 */
#define T_EXPAND_P    const uint8_t*

/** @defgroup EXPAND_COMPILE Compilation Config */              /** @{ */

   #define   __EXPAND_DRV_SPI__                            /**<     @macro __EXPAND_DRV_SPI__  @brief SPI driver selector */
//  #define   __EXPAND_DRV_I2C__                            /**<     @macro __EXPAND_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __EXPAND_DRV_UART__                           /**<     @macro __EXPAND_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup EXPAND_VAR Variables */                           /** @{ */


/** SPI hardware address */
extern const uint8_t _EXPAND_SPI_COMMAND;

extern const uint8_t _EXPAND_OPCODE_WRITE;
extern const uint8_t _EXPAND_OPCODE_READ;

/**
 * SPI module 0 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_0;
/**
 * SPI module 1 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_1;
/**
 * SPI module 2 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_2;
/**
 * SPI module 3 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_3;
/**
 * SPI module 4 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_4;
/**
 * SPI module 5 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_5;
/**
 * SPI module 6 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_6;
/**
 * SPI module 7 hardware address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND_SPI_MODULE_POSITION_7;

extern const uint8_t _EXPAND_PORT_DIRECTION_OUTPUT;
extern const uint8_t _EXPAND_PORT_DIRECTION_INPUT;
extern const uint8_t _EXPAND_IODIRA_BANK1;
extern const uint8_t _EXPAND_IPOLA_BANK1;
extern const uint8_t _EXPAND_GPINTENA_BANK1;
extern const uint8_t _EXPAND_DEFVALA_BANK1;
extern const uint8_t _EXPAND_INTCONA_BANK1;
extern const uint8_t _EXPAND_IOCON_BANK1;
extern const uint8_t _EXPAND_GPPUA_BANK1;
extern const uint8_t _EXPAND_INTFA_BANK1;
extern const uint8_t _EXPAND_INTCAPA_BANK1;
extern const uint8_t _EXPAND_GPIOA_BANK1;
extern const uint8_t _EXPAND_OLATA_BANK1;
extern const uint8_t _EXPAND_IODIRB_BANK1;
extern const uint8_t _EXPAND_IPOLB_BANK1;
extern const uint8_t _EXPAND_GPINTENB_BANK1;
extern const uint8_t _EXPAND_DEFVALB_BANK1;
extern const uint8_t _EXPAND_INTCONB_BANK1;
extern const uint8_t _EXPAND_IOCONO_BANK1;
extern const uint8_t _EXPAND_GPPUB_BANK1;
extern const uint8_t _EXPAND_INTFB_BANK1;
extern const uint8_t _EXPAND_INTCAPB_BANK1;
extern const uint8_t _EXPAND_GPIOB_BANK1;
extern const uint8_t _EXPAND_OLATB_BANK1;
extern const uint8_t _EXPAND_IODIRA_BANK0;
extern const uint8_t _EXPAND_IODIRB_BANK0;
extern const uint8_t _EXPAND_IPOLA_BANK0;
extern const uint8_t _EXPAND_IPOLB_BANK0;
extern const uint8_t _EXPAND_GPINTENA_BANK0;
extern const uint8_t _EXPAND_GPINTENB_BANK0;
extern const uint8_t _EXPAND_DEFVALA_BANK0;
extern const uint8_t _EXPAND_DEFVALB_BANK0;
extern const uint8_t _EXPAND_INTCONA_BANK0;
extern const uint8_t _EXPAND_INTCONB_BANK0;
extern const uint8_t _EXPAND_IOCON_BANK0;
extern const uint8_t _EXPAND_GPPUA_BANK0;
extern const uint8_t _EXPAND_GPPUB_BANK0;
extern const uint8_t _EXPAND_INTFA_BANK0;
extern const uint8_t _EXPAND_INTFB_BANK0;
extern const uint8_t _EXPAND_INTCAPA_BANK0;
extern const uint8_t _EXPAND_INTCAPB_BANK0;
extern const uint8_t _EXPAND_GPIOA_BANK0;
extern const uint8_t _EXPAND_GPIOB_BANK0;
extern const uint8_t _EXPAND_OLATA_BANK0;
extern const uint8_t _EXPAND_OLATB_BANK0;
extern const uint8_t _EXPAND_CN4_PA0;
extern const uint8_t _EXPAND_CN4_PA1;
extern const uint8_t _EXPAND_CN4_PA2;
extern const uint8_t _EXPAND_CN4_PA3;
extern const uint8_t _EXPAND_CN4_PA4;
extern const uint8_t _EXPAND_CN4_PA5;
extern const uint8_t _EXPAND_CN4_PA6;
extern const uint8_t _EXPAND_CN4_PA7;
extern const uint8_t _EXPAND_CN5_PB0;
extern const uint8_t _EXPAND_CN5_PB1;
extern const uint8_t _EXPAND_CN5_PB2;
extern const uint8_t _EXPAND_CN5_PB3;
extern const uint8_t _EXPAND_CN5_PB4;
extern const uint8_t _EXPAND_CN5_PB5;
extern const uint8_t _EXPAND_CN5_PB6;
extern const uint8_t _EXPAND_CN5_PB7;
extern const uint8_t _EXPAND_CN_START_POSITION;
extern uint8_t _EXPAND_INT_ERR;
                                                                       /** @} */
/** @defgroup EXPAND_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup EXPAND_INIT Driver Initialization */              /** @{ */

#ifdef   __EXPAND_DRV_SPI__
void expand_spiDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P spiObj);
#endif
#ifdef   __EXPAND_DRV_I2C__
void expand_i2cDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P i2cObj, uint8_t slave);
#endif
#ifdef   __EXPAND_DRV_UART__
void expand_uartDriverInit(T_EXPAND_P gpioObj, T_EXPAND_P uartObj);
#endif


/** @defgroup EXPAND_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Generic read one bayt from register function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] regAddr                    register address
 *
 * Function read 8-bit of data from 8-bit register address of MCP23S17 chip.
 */
uint8_t expand_readByte( uint8_t modCmd, uint8_t regAddr );

/**
 * @brief Generic write one bayt to register function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] regAddr                        register address
 *
 * @param[in] writeData                         data to write to register
 *
 * Function write 8-bit of data to 8-bit register address of MCP23S17 chip.
 */
void expand_writeByte( uint8_t modCmd, uint8_t regAddr, uint8_t writeData );

/**
 * @brief Generic write one bayt to register function
 *
 * @param[in] modCmd                     module command
 *
 * Function set default configuration to MCP23S17 chip.
 */
void expand_defaultConfiguration( uint8_t modCmd );

/**
 * @brief Set register bits function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] regAddr                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bits to 8-bit register address of MCP23S17 chip.
 */
void expand_setBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask );

/**
 * @brief Clear register bits function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] regAddr                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bits from 8-bit register address of MCP23S17 chip.
 */
void expand_clearBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask );

/**
 * @brief Toggle register bits function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] regAddr                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bits from 8-bit register address of MCP23S17 chip.
 */
void expand_toggleBits( uint8_t modCmd, uint8_t regAddr, uint8_t bitMask );

/**
 * @brief Read one byte of data from PORTA function
 *
 * @param[in] modCmd                     module command
 *
 * @return result                               read data ( PORTA )
 *
 * Function read 8-bit of data from PORTA from 8-bit register address of MCP23S17 chip.
 */
uint8_t expand_readPortA( uint8_t modCmd );

/**
 * @brief Read one byte of data from PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @return result                               read data ( PORTB )
 *
 * Function read 8-bit of data from PORTB from 8-bit register address of MCP23S17 chip.
 */
uint8_t expand_readPortB( uint8_t modCmd );

/**
 * @brief Read two byte of data from PORTA & PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @return result                               read data ( PORTA & PORTB )
 *
 * Function read 16-bit of data from PORTA & PORTB from 8-bit register address of MCP23S17 chip.
 */
uint16_t expand_readBothPorta( uint8_t modCmd );

/**
 * @brief Write one byte of data to register for PORTA function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         data to write
 *
 * Function write 8-bit of data to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_writePortA( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Clear bit from register for PORTA function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bit from 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_clearBitPortA( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set bit to register for PORTA function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bit to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_setBitPortA( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Toggle bit to register for PORTA function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bit from 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_toggleBitPortA( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Write one byte of data to register for PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         data to write
 *
 * Function write 8-bit of data
 * to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_writePortB( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Clear bit from register for PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bit from 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_clearBitPortB( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set bit to register for PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bit to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_setBitPortB( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Toggle bit to register for PORTB function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bit from 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_toggleBitPortB( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set expander PORTA direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         data to write
 *
 * Function set expander direction by write 8-bit data
 * to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_setDirectionPortA( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Set expander PORTA input direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTA set as input,
 * to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_setInputDirPortA( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set expander PORTA output direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTA set as output,
 * to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_setOutputDirPortA( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set expander PORTB direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         data to write
 *
 * Function set expander direction by write 8-bit data
 * to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_setDirectionPortB( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Set expander PORTB input direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTB set as input,
 * to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_setInputDirPortB( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set expander PORTB output direction function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTB set as output,
 * to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_setOutputDirPortB( uint8_t modCmd, uint8_t bitMask );

/**
 * @brief Set pull-ups of the expander for PORTA pins function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         pull up value
 *
 * Function set pull-ups of the expander for PORTA pins
 * by write 8-bit pull up value data
 * to 8-bit register address from PORTA of MCP23S17 chip.
 */
void expand_setPullUpsPortA( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Set pull-ups of the expander for PORTB pins function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         pull up value
 *
 * Function set pull-ups of the expander for PORTB pins
 * by write 8-bit pull up value data
 * to 8-bit register address from PORTB of MCP23S17 chip.
 */
void expand_setPullUpsPortB( uint8_t modCmd, uint8_t writeData );

/**
 * @brief Active pin by position on PORTA function
 *
 * @param[in] position                          pin position
 *
 * Function activate pin on PORTA by position, from PA0 to PA7.
 */
void expand_setPotrA( uint8_t position );

/**
 * @brief Active pin by position on PORTB function
 *
 * @param[in] position                          pin position
 *
 * Function activate pin on PORTB by position, from PB0 to PB7.
 */
void expand_setPotrB( uint8_t position );

/**
 * @brief Reset function
 *
 * @param[in] modCmd                     module command
 *
 * @param[in] writeData                         pull up value
 *
 * Function reset Expand 2 click by set RST pin from low to high.
 *
 * @note
 * delay is 11ms
 */
void expand_reset();

/**
 * @brief Get state of interrupt pin function
 *
 * @return state
 * 0 - No Active, 1 - Active
 *
 * Function get state of interrupt ( INT ) pin.
 */
uint8_t expand_getInterrupt();





                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Expand_STM.c
    @example Click_Expand_TIVA.c
    @example Click_Expand_CEC.c
    @example Click_Expand_KINETIS.c
    @example Click_Expand_MSP.c
    @example Click_Expand_PIC.c
    @example Click_Expand_PIC32.c
    @example Click_Expand_DSPIC.c
    @example Click_Expand_AVR.c
    @example Click_Expand_FT90x.c
    @example Click_Expand_STM.mbas
    @example Click_Expand_TIVA.mbas
    @example Click_Expand_CEC.mbas
    @example Click_Expand_KINETIS.mbas
    @example Click_Expand_MSP.mbas
    @example Click_Expand_PIC.mbas
    @example Click_Expand_PIC32.mbas
    @example Click_Expand_DSPIC.mbas
    @example Click_Expand_AVR.mbas
    @example Click_Expand_FT90x.mbas
    @example Click_Expand_STM.mpas
    @example Click_Expand_TIVA.mpas
    @example Click_Expand_CEC.mpas
    @example Click_Expand_KINETIS.mpas
    @example Click_Expand_MSP.mpas
    @example Click_Expand_PIC.mpas
    @example Click_Expand_PIC32.mpas
    @example Click_Expand_DSPIC.mpas
    @example Click_Expand_AVR.mpas
    @example Click_Expand_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __expand_driver.h

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