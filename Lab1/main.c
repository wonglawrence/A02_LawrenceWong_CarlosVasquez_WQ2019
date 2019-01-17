//
//
//
// LAWRENCE WONG, CARLOS VASQUEZ
//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the 
//                        GPIO control using Driverlib api calls. The LEDs 
//                        connected to the GPIOs on the LP are used to indicate 
//                        the GPIO output. The GPIOs are driven high-low 
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
//UART libraries
#include "uart.h"
#include "uart_if.h"

// Common interface includes
#include "gpio_if.h"

#include "pinmux.h"

#define APPLICATION_VERSION     "1.1.1"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();

    // Initialize the serial terminal
    InitTerm();
    ClearTerm();

    // Displays the opening message when the program is executed
    Message("\t\t****************************************************\n\r");
    Message("\t\t\t        CC3200 GPIO Application        \n\r");
    Message("\t\t Push SW3 to start LED binary counting  \n\r");
    Message("\t\t Push SW2 to blink LEDs on and off \n\r") ;
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");

    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    
    // counter: used to keep track of the count when SW3 is pressed
    // sw2Pressed and sw3Pressed is used to keep track of which button has been pressed
    int sw3Pressed = 0;
    int sw2Pressed = 0;
    int counter = 0;

    while(1){
    	//cotinuously polls the sw3 and sw2 buttons
        int sw3 = GPIOPinRead(GPIOA1_BASE,0x20);
        int sw2 = GPIOPinRead(GPIOA2_BASE,0x40);

        // check if sw3 has been pressed and it was not the most recent button pressed
        if (sw3 && !sw3Pressed) {
            Message("SW3 Pressed\n\r");
            //change the state so that sw3 is the most recent button pressed
            //this makes it so that the "SW3 Pressed" message is only displayed once
            sw2Pressed = 0;
            sw3Pressed = 1;
            //drive pin18
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0);

            //loops through the counter from 000-111
            while(1) {
                switch (counter) {
                    case 1:
                        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 2:
                        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 3:
                        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 4:
                        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 5:
                        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 6:
                        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        break;
                    case 7:
                        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                        MAP_UtilsDelay(8000000);
                        // reset the counter when it reaches 111
                        // -1 because it'll be incremented to 0 at the end of the loop
                        counter = -1;
                        break;
                }
                // polls sw2 at the end of each count
                sw2 = GPIOPinRead(GPIOA2_BASE,0x40);
                // break out of the infinite loop if sw2 has been pressed
                if (sw2) {
                    break;
                }
                counter++;
            }


        }

        // check if sw2 has been pressed and it was not the most recent button pressed
        if (sw2 && !sw2Pressed) {
            Message("SW2 Pressed\n\r");
            //change the state so that sw2 is the most recent button pressed
            //this makes it so that the "SW2 Pressed" message is only displayed once
            sw3Pressed = 0;
            sw2Pressed = 1;
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);

            GPIO_IF_LedOff(MCU_ALL_LED_IND);
            while(1) {
                //
                // Alternately toggle hi-low each of the GPIOs
                // to switch the corresponding LED on/off.
                //
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                MAP_UtilsDelay(8000000);
                GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                MAP_UtilsDelay(8000000);
                sw3 = GPIOPinRead(GPIOA1_BASE,0x20);
                if (sw3) {
                    break;
                }
            }
        }
    }

    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
