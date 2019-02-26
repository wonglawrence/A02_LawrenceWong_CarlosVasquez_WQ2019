//Lawrence Wong
//Carlos Vasquez
//LAB 4

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
#include "timer_if.h"

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
#define ONE '1'
#define TWO '2'
#define THREE '3'
#define FOUR '4'
#define FIVE '5'
#define SIX '6'
#define SEVEN '7'
#define EIGHT '8'
#define NINE '9'
#define ZERO '0'
#define MUTE '#'
#define LAST '*'

#define SPI_IF_BIT_RATE  400000
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
volatile char previousButton = '%';
volatile unsigned char currentChar = (unsigned char) '0';
volatile int charCounter = 0;
volatile unsigned char message[100];
volatile unsigned char receivedMessage[100];
volatile int rmessageIndex = 0;
volatile int messageLength = 0;
volatile int messageIndex = -1;

//DTMF globals
//int f_tone[8]={697, 770, 852, 941, 1209, 1336, 1477, 1633};
int N = 410;
int coeff[8] = {31548, 31281, 30951, 30556, 29144, 28361, 27409, 26258};
int power_all[8];
int samples[410];
volatile int sampleCount = 0;
volatile int flag = 0;
volatile int new_dig;
char buttonBuffer[3];
int buttonIndex = 0;
volatile int buttonState = 1;
volatile int oldToneTime =0;


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
//this function determines the character and assembles the message
//
void sendMessage(char num) {
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
    GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0);
    Outstr(message);
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
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

        GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
        GPIOPinWrite(GPIOA1_BASE, 0x10, 0);

        //draw the received character on the screen and increment the cursors
        fillRect(cursor_x2, cursor_y2, 6,8, 0x000);
        drawChar(cursor_x2, cursor_y2, character, WHITE, BLACK, 1);


        cursor_x2 += 6;

        if(cursor_x2 > 121) {
            cursor_x2 = 0;
            cursor_y2 += 8;
        }
        GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
    }
}


void TimerISR() {
    unsigned long sample1, sample2, sample;
    unsigned char inputBuffer[2];

    Timer_IF_InterruptClear(TIMERA1_BASE);

//   printf("%d\n",TimerValueGet(TIMERA1_BASE,TIMER_A));
//   printf("%d\n", sampleCount);
    if (sampleCount<N) {
        GPIOPinWrite(GPIOA3_BASE, 0x10, 0);
        SPICSEnable(GSPI_BASE);
        SPITransfer(GSPI_BASE, 0, inputBuffer, 2, SPI_CS_ENABLE|SPI_CS_DISABLE);

        sample1 = inputBuffer[0]&0x001F;
        sample1 = sample1<<8;
        sample2 = inputBuffer[1];
        sample = (sample1 | sample2) >>3;
        sample = sample - 380;
        samples[sampleCount++] = sample;
      //  printf("%d\n", sample);
        GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
        SPICSDisable(GSPI_BASE);
    }
    else if(sampleCount==N) {
        //TimerDisable(TIMERA1_BASE, TIMER_A);
        //TimerIntDisable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
 //       printf("stopped\n");
        Timer_IF_Stop(TIMERA1_BASE, TIMER_A);
        flag=1;
    }

    //TimerIntClear(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
    //TimerLoadSet(TIMERA1_BASE, TIMER_A, 5000);
}



long int goertzel(int sample[], long int coeff, int N)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
int Q, Q_prev, Q_prev2,i;
long prod1,prod2,prod3,power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i=0; i<N; i++) // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
        {
            Q = (sample[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
            Q_prev2 = Q_prev;                                    // shuffle delay elements
            Q_prev = Q;
        }

        //calculate the three products used to calculate power
        prod1=( (long) Q_prev*Q_prev);
        prod2=( (long) Q_prev2*Q_prev2);
        prod3=( (long) Q_prev *coeff)>>14;
        prod3=( prod3 * Q_prev2);

        power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down
//        printf("%d\n", power);
        return power;
}


//-------Post-test function---------------------------------------//
void post_test(void) {
    //initialize variables to be used in the function
//    printf("post_test\n");
    int i,row,col,max_power;
    char button;
    int time, difference;

//    for(i=0;i<8;i++) {
//        printf("%d | ",power_all[i]);
//    }
//    printf("\n");



    char row_col[4][4] =       // array with the order of the digits in the DTMF system
        {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
        };

    // find the maximum power in the row frequencies and the row number

    max_power=0;            //initialize max_power=0

    for(i=0;i<4;i++) {       //loop 4 times from 0>3 (the indecies of the rows)
        if (power_all[i] > max_power) {   //if power of the current row frequency > max_power
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
        }
    }
    // find the maximum power in the column frequencies and the column number
    max_power=0;            //initialize max_power=0

    for(i=4;i<8;i++) {    //loop 4 times from 4>7 (the indecies of the columns)
        if (power_all[i] > max_power) { //if power of the current column frequency > max_power
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
        }
    }

//    if(power_all[col]==0 && power_all[row]==0) {//if the maximum powers equal zero > this means no signal or inter-digit pause
//        new_dig=1; //set new_dig to 1 to display the next decoded digit
//    }

 //   printf("%c\n", row_col[row][col-4]);
    if((power_all[col]>2000 && power_all[row]>2000)) {
        button = row_col[row][col-4];
        buttonBuffer[buttonIndex++] = button;
        if (buttonIndex>2){
            buttonIndex = 0;
        }

        if (buttonBuffer[0]==buttonBuffer[1] && buttonBuffer[0]==buttonBuffer[2]){
            time = TimerValueGet(TIMERA0_BASE,TIMER_A);
            difference = oldToneTime - time;
            oldToneTime = time;
            if(difference > 20000000)
                //printf("%c\n\r", button);
                sendMessage(button);
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
    fillScreen(BLACK);
    //
    // Initialising the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Configure Timer A0
    //
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    TimerConfigure(TIMERA0_BASE,TIMER_CFG_PERIODIC);
    TimerPrescaleSet(TIMERA0_BASE,TIMER_A,0);
    //
    // Configure Timer A1
    //
    PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA1);
    TimerConfigure(TIMERA1_BASE,TIMER_CFG_PERIODIC);
    TimerPrescaleSet(TIMERA1_BASE,TIMER_A,0);
    //
    // Configure Timer Interrupt for Timer A1
    //
//    TimerIntRegister(TIMERA1_BASE,TIMER_A,TimerISR);
//    TimerIntClear(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
//    TimerIntEnable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
//    TimerLoadSet(TIMERA1_BASE, TIMER_A, 5000);

    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
    GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);


    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, TimerISR);

    // Turn on the timers feeding values in mSec
    Timer_IF_Start(TIMERA1_BASE, TIMER_A, 63);


    //
    // Enable Timers
    //
    TimerEnable(TIMERA0_BASE,TIMER_A);
    //TimerEnable(TIMERA1_BASE,TIMER_A);
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
    int i;

    //Report("**************PROGRAM STARTING***************\n\r");


    while(1) {

        if(flag == 1) { // wait till N samples are read in the buffer and the flag set by the ADC ISR
            //TimerIntDisable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
            for (i=0;i<8;i++) {
                power_all[i]=goertzel(samples, coeff[i], N); // call goertzel to calculate the power at each frequency and store it in the power_all array
            }
            post_test(); // call post test function to validate the data and display the pressed digit if applicable
            flag = 0;
            sampleCount = 0;
            Timer_IF_Start(TIMERA1_BASE, TIMER_A, 63);
        }
    }
}

