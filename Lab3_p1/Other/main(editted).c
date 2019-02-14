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
// Application Name     - UART Demo
// Application Overview - The objective of this application is to showcase the
//                        use of UART. The use case includes getting input from
//                        the user and display information on the terminal. This
//                        example take a string as input and display the same
//                        when enter is received.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_UART_Demo_Application
// or
// docs\examples\CC32xx_UART_Demo_Application.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_demo
//! @{
//
//*****************************************************************************

// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "interrupt.h"
#include "pinmux.h"
#include "utils.h"
#include "prcm.h"

// Common interface include
#include "uart.h"
#include "uart_if.h"
#include "gpio.h"
#include "timer.h"
#include "spi.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

//*****************************************************************************
//                          MACROS                                  
//*****************************************************************************
extern int cursor_x;
extern int cursor_y;

extern int cursor_x2 = 0;
extern int cursor_y2 = 64;

float p = 3.1415926;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define ONE 79396991
#define TWO 79380671
#define THREE 79413311
#define FOUR 79405151
#define FIVE 79388831
#define SIX 79421471
#define SEVEN 79401071
#define EIGHT 79384751
#define NINE 79417391
#define ZERO 79370471
#define MUTE 79373021
#define LAST 79401581

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100



//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;
volatile int oldTime;
volatile unsigned long oldCharTime;
volatile int currentState = 0;
volatile int bufferCount = 0;
volatile int buffer[32];
volatile int previousButton = 0;
volatile unsigned char currentChar = (unsigned char)'0';
volatile int charCounter = 0;
volatile unsigned char message[256];
volatile unsigned char receivedMessage[256];
volatile int rmessageIndex = 0;
volatile int messageLength = 0;
volatile int messageIndex = -1;
volatile int j = 0;


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
//                      LOCAL DEFINITION                                   
//*****************************************************************************
int determineState(int difference, int currentState) {
    if (currentState == 0) {
        if ((difference>700000)&&(difference<800000)) {
            return 1; //button has just been pressed
        }
        else
            return 0;
    }

    else if (currentState == 1) {
        if ((difference>300000)&&(difference<400000)) {
            return 2; //button has just been pressed
        }
        else
            return 0;
    }

    else if (currentState == 2) {
        return 3;
    }

    else
        return 2;
}

int decode(int* buffer) {
    int num = 0;
    int mask;
    int i;

    for(i=0;i<32;i++){
        if (buffer[i]>100000) {
            mask = 1<<(31-i);
            num = num | mask;
        }
    }

    switch (num) {
        case TWO :
            break;
        case THREE :
            break;
        case FOUR :
            break;
        case FIVE :
            break;
        case SIX :
            break;
        case SEVEN :
            break;
        case EIGHT :
            break;
        case NINE :
            break;
        case ZERO :
            break;
        case MUTE :
            break;
        case LAST :
            break;
        default:
            return -1;
    }
    return num;
}


void sendMessage(int num) {
    unsigned long time;
    int difference = 0;
    int i;
    int numbers[8] = {50, 51, 52, 53, 54, 55, 56, 57};
    int letters[8] = {97, 100, 103, 106, 109, 112, 116, 119};
    //Report("%d\n\r", time);


    if (num==MUTE){
        for(i=0;i<=messageIndex;i++) {
            UARTCharPut(UARTA1_BASE, message[i]);
            message[i] = ' ';
        }
        messageIndex = -1;
    }
    else if(num==LAST) {
        if(messageIndex>=0) {
            message[messageIndex--]=' ';
            previousButton = num;
        }
    }
    else if (num==ZERO){
        messageIndex++;
        message[messageIndex] = ' ';
        previousButton = num;
    }
    else if (num!=previousButton) {
        oldCharTime = TimerValueGet(TIMERA0_BASE,TIMER_A);
        charCounter = 0;
        previousButton = num;
        messageIndex++;

        switch (num) {
            case TWO :
                currentChar = (unsigned char) 'a';
                break;
            case THREE :
                currentChar = (unsigned char)'d';
                break;
            case FOUR :
                currentChar = (unsigned char)'g';
                break;
            case FIVE :
                currentChar = (unsigned char)'j';
                break;
            case SIX :
                currentChar = (unsigned char)'m';
                break;
            case SEVEN :
                currentChar = (unsigned char)'p';
                break;
            case EIGHT :
                currentChar = (unsigned char)'t';
                break;
            case NINE :
                currentChar = (unsigned char)'w';
                break;
        }
        message[messageIndex] = currentChar;
    }
    else {
        time = TimerValueGet(TIMERA0_BASE,TIMER_A);
        difference = oldCharTime - time;
        oldCharTime = time;

        //Report("%d\n\r", difference);
        if(num == -1) {
            return;
        }


        if (difference < 100000000) {
            if((num==SEVEN)||(num==NINE)) {
                if (charCounter >= 4) {
                    currentChar = letters[j];
                    charCounter = 0;
                }
                else if (charCounter >= 3) {
                    j = (currentChar - 99)/3;
                    currentChar = numbers[j];
                    charCounter++;
                }
                else {
                    currentChar = currentChar + 1;
                    charCounter++;
                }
            }
            else {
                if (charCounter >= 3) {
                    currentChar = letters[j];
                    charCounter = 0;
                }
                else if (charCounter >= 2) {
                    j = (currentChar - 99)/3;
                    currentChar = numbers[j];
                    charCounter++;
                }
                else {
                    currentChar = currentChar + 1;
                    charCounter++;
                }
            }
            message[messageIndex] = currentChar;
        }
        else {
            previousButton = num;
           // charCounter = 0;
            messageIndex++;
//            j = (currentChar - 99)/3;
            currentChar = letters[j];
//            currentChar = currentChar - charCounter;
            message[messageIndex] = currentChar;

            charCounter = 0;
        }
    }

    //Report("%c\n\r", currentChar);

    for(i=0;i<=messageIndex;i++){
        Report("%c",message[i]);
    }
    Report("\n\r");
  //  fillScreen(BLACK);
    cursor_x = 0;
    cursor_y = 0;
    Outstr(message);
}




void InterruptHandler(void)
{
    unsigned long ulStatus;
    unsigned long time;
    int difference;
    int decodedNum;

    ulStatus = GPIOIntStatus(GPIOA3_BASE, true);
    GPIOIntClear(GPIOA3_BASE, ulStatus);

    time = TimerValueGet(TIMERA0_BASE,TIMER_A);
    difference = oldTime - time;
    oldTime = time;

    if (bufferCount < 32) {
        if (currentState == 3) {
            buffer[bufferCount++] = difference;
            //Report("KEEP %d\n\r", difference);
        }
        currentState = determineState(difference, currentState);
    }
    else if (bufferCount == 32) {
        decodedNum = decode(buffer);
        sendMessage(decodedNum);
        bufferCount = 0;
        currentState = 0;
    }
}


void UARTHandler() {
    unsigned char character;
    cursor_x2 = 0;
    cursor_y2 = 64;

    fillRect(0,64,128,64, 0x000);
    UARTIntClear(UARTA1_BASE, UART_INT_RX);

    while (UARTCharsAvail(UARTA1_BASE)) {
        character = UARTCharGet(UARTA1_BASE);
        receivedMessage[rmessageIndex++] = character;

        fillRect(cursor_x2, cursor_y2, 6,8, 0x000);
        drawChar(cursor_x2, cursor_y2, character, WHITE, BLACK, 1);

        cursor_x2 += 6;

        if (cursor_x2 > 121) {
            cursor_x2 = 0;
            cursor_y2 += 8;
        }
    }

}

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

//*****************************************************************************
//
//! Main function handling the uart echo. It takes the input string from the
//! terminal while displaying each character of string. whenever enter command 
//! is received it will echo the string(display). if the input the maximum input
//! can be of 80 characters, after that the characters will be treated as a part
//! of next string.
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************
void main()
 {
    //
    // Initailizing the board
    //
    BoardInit();
    //
    // Muxing for Enabling UART_TX and UART_RX.
    //
    PinMuxConfig();
    //
    // Enable the SPI module clock
    //
    PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    //
    // Reset the peripheral
    //
    PRCMPeripheralReset(PRCM_GSPI);
    //
    // Reset SPI
    //
    SPIReset(GSPI_BASE);
    //
    // Configure SPI interface
    //
    SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    SPIEnable(GSPI_BASE);
    //
    // Initialize Adafruit
    //
    Adafruit_Init();
    //
    // Initialising the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Configure Timer
    //
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    TimerConfigure(TIMERA0_BASE,TIMER_CFG_PERIODIC);
    TimerPrescaleSet(TIMERA0_BASE,TIMER_A,0);
    TimerEnable(TIMERA0_BASE,TIMER_A);
    oldTime = TimerValueGet(TIMERA0_BASE,TIMER_A);


    UARTIntRegister(UARTA1_BASE, UARTHandler);
    UARTIntClear(UARTA1_BASE, UART_INT_RX);
    UARTIntEnable(UARTA1_BASE, UART_INT_RX);
    UARTFIFOEnable(UARTA1_BASE);
    UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                                UART_BAUD_RATE,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));



    unsigned long ulStatus;


    Report("***************LAWRENCE WONG*****************\n\r");
    Report("**************PROGRAM STARTING***************\n\r");
    //
    // Configure PIN_05 for interrupt
    //
    GPIOIntRegister(GPIOA3_BASE, InterruptHandler);
    GPIOIntTypeSet(GPIOA3_BASE, 0x10, GPIO_BOTH_EDGES);
    ulStatus = GPIOIntStatus(GPIOA3_BASE, false);
    GPIOIntClear(GPIOA3_BASE, ulStatus);
    GPIOIntEnable(GPIOA3_BASE, 0x10);

    fillScreen(BLACK);

    while(1);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************



