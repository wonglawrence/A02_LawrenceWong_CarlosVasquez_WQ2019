//Lawrence Wong
//Carlos Vasquez
//***************************************************************************
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
volatile unsigned char currentChar = (unsigned char) '0';
volatile int charCounter = 0;
volatile unsigned char message[256];
volatile unsigned char receivedMessage[256];
volatile int rmessageIndex = 0;
volatile int messageLength = 0;
volatile int messageIndex = -1;

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
//
// this function acts as a finite state machine
// the state is determined in this function
// state 0: waiting for signal
// state 1: extra long low pulse received
// state 1 > state 2: long pulse received
// state 2: discard
// state 3: save time
//
//
int determineState(int difference, int currentState) {
    //
    // go to state 1 from state 0 if extra long pulse received
    // else stay in state 0
    //
    if (currentState == 0) {
        if ((difference>700000)&&(difference<800000)) {
            return 1; //button has just been pressed
        }
        else
            return 0;
    }
    //
    // go to state 2 from state 1 if long pulse received
    // else go back to state 0
    //
    else if (currentState == 1) {
        if ((difference>300000)&&(difference<400000)) {
            return 2; //button has just been pressed
        }
        else
            return 0;
    }

    //
    // go to state 3 from state 2
    //
    else if (currentState == 2) {
        return 3;
    }

    //
    // go to state 2 from state 3
    //
    else
        return 2;
}

//
// this function takes the buffer of size 32 as an argument
// decodes the buffer and returns the decoded button
//
int decode(int* buffer) {
    int num = 0;
    int mask;
    int i;

    //
    // iterate throughe each value of in the buffer
    //
    for(i=0;i<32;i++){
        //if the value is greater than 100,000
        //then the value corresponds to a bit of 1
        //else it corresponds to 0
        if (buffer[i]>100000) {
            mask = 1<<(31-i); //the mask is a bit of 1 shifted to the designated bit location
            num = num | mask; //OR with the mask to save the bit
        }
    }

    // case statement to determine the button value
    // if the button is not recognized, -1 is returned
    // otherwise, the value for the button is returned
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


//
//this function determines the character and assembles the message
//
void sendMessage(int num) {
    unsigned long time;
    int difference = 0;
    int i;

    //
    // if the mute button is pressed
    // the message is sent through UART one character at a time
    //
    if (num==MUTE){
        for(i=0;i<=messageIndex;i++) {
            //this sends the character
            UARTCharPut(UARTA1_BASE, message[i]);
            message[i] = ' '; //the character is deleted once it is sent
        }
        //the messageIndex is reseted
        messageIndex = -1;
    }
    //
    // if the last button is pressed
    // we delete the current character
    //
    else if(num==LAST) {
        if(messageIndex>=0) {
            //the character is replaced with a white space
            //the current index of the character is decremented
            message[messageIndex--]=' ';
            // previousButton keeps track of the most recent button press
            previousButton = num;
        }
    }
    //
    // if 0 is pressed
    // we add a space to the message
    //
    else if (num==ZERO){
        messageIndex++;
        message[messageIndex] = ' ';
        previousButton = num;
    }
    //
    // there is a new button press
    // that is not the same as the previous button
    //
    else if (num!=previousButton) {
        oldCharTime = TimerValueGet(TIMERA0_BASE,TIMER_A);
        charCounter = 0;
        previousButton = num;
        messageIndex++;
        //
        // depending on the button pressed
        // the first character of that button is added to the message
        //
        switch (num) {
            case TWO :
                currentChar = (unsigned char) 'a';
                break;
            case THREE :
                currentChar = (unsigned char) 'd';
                break;
            case FOUR :
                currentChar = (unsigned char) 'g';
                break;
            case FIVE :
                currentChar = (unsigned char) 'j';
                break;
            case SIX :
                currentChar = (unsigned char) 'm';
                break;
            case SEVEN :
                currentChar = (unsigned char) 'p';
                break;
            case EIGHT :
                currentChar = (unsigned char) 't';
                break;
            case NINE :
                currentChar = (unsigned char) 'w';
                break;
        }
        message[messageIndex] = currentChar;
    }
    //
    // the same button has been pressed
    //
    else {
        // we get the time to check the time between button presses
        // a long wait time will result in a new character
        // short wait time results in a replacement of the current character
        time = TimerValueGet(TIMERA0_BASE,TIMER_A);
        difference = oldCharTime - time;
        oldCharTime = time;

        //end the function if an unrecognized button is pressed
        if(num == -1) {
            return;
        }

        //
        //if the time is short, we change the current character
        //to the next character of the corresponding button
        //
        if (difference < 100000000) {
            //
            // 7 and 9 are special cases because they have 4 characters
            // we use ASCII to increment the characters
            // charCounter keeps track of how many times a button
            // has been pressed sequentially
            //
            if((num==SEVEN)||(num==NINE)) {
                //
                //if we exceed the number of characters, we go back to the beginning
                //
                if (charCounter >= 3) {
                    currentChar = currentChar-3;
                    charCounter = 0;
                }
                //
                //else increment the character
                //
                else {
                    currentChar = currentChar + 1;
                    charCounter++;
                }
            }
            //
            // same but for the other buttons
            //
            else {
                if (charCounter >= 2) {
                    currentChar = currentChar-2;
                    charCounter = 0;
                }
                else {
                    currentChar = currentChar + 1;
                    charCounter++;
                }
            }
            message[messageIndex] = currentChar;
        }
        //
        // if the time between button presses is long
        // it acts as a brand new button press
        //
        else {
            previousButton = num;
            messageIndex++;
            currentChar = currentChar - charCounter;
            message[messageIndex] = currentChar;
            charCounter = 0;
        }
    }


    //
    // This rewrites the message on the screen with the new message
    // we rewrite each time the message changes
    // this helps the user see the message in real time
    //
    cursor_x = 0;
    cursor_y = 0;
    Outstr(message);
}


//
//GPIO Interrupt Handler
//
void InterruptHandler(void)
{
    unsigned long ulStatus;
    unsigned long time;
    int difference;
    int decodedNum;

    //Clear the interrupt flag
    //this tells the processor that the interrupt has been handled
    ulStatus = GPIOIntStatus(GPIOA3_BASE, true);
    GPIOIntClear(GPIOA3_BASE, ulStatus);

    //calculate the difference between current time and
    //the time of the previous interrupt
    //the difference equals the time of the pulse
    time = TimerValueGet(TIMERA0_BASE,TIMER_A);
    difference = oldTime - time;
    oldTime = time;

    //if the buffer is not filled
    if (bufferCount < 32) {
        //if we are in state 3, we save the time of the pulse
        //then we determine the state of the finite state machine
        if (currentState == 3) {
            buffer[bufferCount++] = difference;
            //Report("KEEP %d\n\r", difference);
        }
        currentState = determineState(difference, currentState);
    }
    //if the buffer is full
    // this means we have received the full transmission signal
    //This sends the buffer to be decoded to a specific button
    // then decoded to a specific character
    // next, the buffer is cleared and is ready for a new signal
    else if (bufferCount == 32) {
        decodedNum = decode(buffer);
        sendMessage(decodedNum);
        bufferCount = 0;
        currentState = 0;
    }
}



//
// UART interrupt handler
//
void UARTHandler() {
    unsigned char character;
    int i;
    //sets the cursors to the bottom half of the OLED
    cursor_x2 = 0;
    cursor_y2 = 64;

    //fillRect clears the messages on the bottom half of the screen
    fillRect(0, 64, 128,64, 0x000);
    //clear the interrupt flag
    UARTIntClear(UARTA1_BASE, UART_INT_RX);
    //receivedMessage[rmessageIndex++] = UARTCharGet(UARTA1_BASE);

    //while a character is available in the FIFO
    while (UARTCharsAvail(UARTA1_BASE)) {
        character = UARTCharGet(UARTA1_BASE);
        receivedMessage[rmessageIndex++] = character;

        //draw the received character on the screen and increment the cursors
        fillRect(cursor_x2, cursor_y2, 6,8, 0x000);
        drawChar(cursor_x2, cursor_y2, character, WHITE, BLACK, 1);

        cursor_x2 += 6;

        if(cursor_x2 > 121) {
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

    //
    // Configure UART Interrupt
    //
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

    Report("**************PROGRAM STARTING***************\n\r");
    //
    // Configure PIN_05 for interrupt
    //
    GPIOIntRegister(GPIOA3_BASE, InterruptHandler);
    GPIOIntTypeSet(GPIOA3_BASE, 0x10, GPIO_BOTH_EDGES);
    ulStatus = GPIOIntStatus(GPIOA3_BASE, false);
    GPIOIntClear(GPIOA3_BASE, ulStatus);
    GPIOIntEnable(GPIOA3_BASE, 0x10);

    //
    // start program with a black screen on OLED
    //
    fillScreen(BLACK);

    while(1);
}

