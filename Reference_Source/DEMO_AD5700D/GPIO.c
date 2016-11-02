/*
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES INC. ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES INC. BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

YOU ASSUME ANY AND ALL RISK FROM THE USE OF THIS CODE OR SUPPORT FILE.

IT IS THE RESPONSIBILITY OF THE PERSON INTEGRATING THIS CODE INTO AN APPLICATION
TO ENSURE THAT THE RESULTING APPLICATION PERFORMS AS REQUIRED AND IS SAFE.
*/

/******************************************************************************
 Author       : Analog Devices
                Industrial Automation System Applications (M.B.)

 Date         : February 2013

 File         : GPIO.c

 Hardware     : ADuCM360

 Description  : General purpose (digital) input/output related functions
                Code specific for DEMO-AD5700D2Z

 Status       : Funtionality checked
******************************************************************************/

#include "GPIO.h"

#include "ADuCM360.h"


//*****************************************************************************
// ADuCM360 GPIO initialisation
//*****************************************************************************
void GPIO_Init(void)
{
  // With respect to Hardware digital I/O allocation:
  // Enable relevant outputs, pull-ups and open collectors, set output state

  // --------------------------------------------------------------------------
  // Port 0
  // --------------------------------------------------------------------------
  // Pin         mode  Dir.  Drive    Signal            Comment
  // --------------------------------------------------------------------------
  // P0.7/SOUT  (UART) out   1        AD5700 HART TXD
  // P0.6/SIN   (UART) in    -        AD5700 HART RXD
  // P0.5        GPIO  in    -        AD5700 HART CD    Test point T1
  // P0.4        GPIO  out   1        AD5700 HART RTS   Test point T2
  // P0.3        GPIO  out   0        AD5700 CLK_CFG    Low = No CLK_OUT signal 
  // P0.2/SOUT   UART  out   1        J3-4   UART TXD   Connector J3 pin 4
  // P0.1/SIN    UART  in    pull-up  J3-6   UART RXD   Connector J3 pin 6
  // P0.0        GPIO  in    pull-up         Test       Test point T4
  //                                  (Board DGND,      Test point T3)

  // Set pin function
  pADI_GP0->GPCON = 0x00
    | GP0CON_CON7_GPIO            //  AD5700 HART TXD
    | GP0CON_CON6_GPIOIRQ2        //  AD5700 HART RXD
    | GP0CON_CON5_GPIOIRQ1        //  AD5700 HART CD    Test point T1
    | GP0CON_CON4_GPIO            //  AD5700 HART RTS   Test point T2
    | GP0CON_CON3_GPIOIRQ0        //  AD5700 CLK_CFG    Low = No CLK_OUT signal
    | GP0CON_CON2_UARTTXD         //  J3-4   UART TXD   Connector J3 pin 4
    | GP0CON_CON1_UARTRXD         //  J3-6   UART RXD   Connector J3 pin 6
    | GP0CON_CON0_GPIO            //         Test       Test point T4
  ;

  // Enable outputs
  // Note that pin direction may be overdriven by setting the pin function
  pADI_GP0->GPOEN = 0x00
    | GPOEN_OEN7_IN               //  AD5700 HART TXD
    | GPOEN_OEN6_IN               //  AD5700 HART RXD
    | GPOEN_OEN5_IN               //  AD5700 HART CD    Test point T1
    | GPOEN_OEN4_OUT              //  AD5700 HART RTS   Test point T2
    | GPOEN_OEN3_OUT              //  AD5700 CLK_CFG    Low = No CLK_OUT signal
    | GPOEN_OEN2_IN               //  J3-4   UART TXD   Connector J3 pin 4
    | GPOEN_OEN1_IN               //  J3-6   UART RXD   Connector J3 pin 6
    | GPOEN_OEN0_IN               //         Test       Test point T4
  ;

// Set output state
  pADI_GP0->GPOUT = 0x00
    | GPOUT_OUT7_HIGH             //  AD5700 HART TXD
    | GPOUT_OUT6_HIGH             //  AD5700 HART RXD
    | GPOUT_OUT5_LOW              //  AD5700 HART CD    Test point T1
    | GPOUT_OUT4_HIGH             //  AD5700 HART RTS   Test point T2
    | GPOUT_OUT3_LOW              //  AD5700 CLK_CFG    Low = No CLK_OUT signal
    | GPOUT_OUT2_HIGH             //  J3-4   UART TXD   Connector J3 pin 4
    | GPOUT_OUT1_HIGH             //  J3-6   UART RXD   Connector J3 pin 6
    | GPOUT_OUT0_HIGH             //         Test       Test point T4
  ;

  // Enable pull up resistors
  pADI_GP0->GPPUL = 0x00
    | GP0PUL_PUL7_EN              //  AD5700 HART TXD
    | GP0PUL_PUL6_DIS             //  AD5700 HART RXD
    | GP0PUL_PUL5_DIS             //  AD5700 HART CD    Test point T1
    | GP0PUL_PUL4_DIS             //  AD5700 HART RTS   Test point T2
    | GP0PUL_PUL3_DIS             //  AD5700 CLK_CFG    Low = No CLK_OUT signal
    | GP0PUL_PUL2_EN              //  J3-4   UART TXD   Connector J3 pin 4
    | GP0PUL_PUL1_EN              //  J3-6   UART RXD   Connector J3 pin 6
    | GP0PUL_PUL0_EN              //         Test       Test point T4
  ;

  // -------------------------------------------------------------------------
  // Port 1
  // -------------------------------------------------------------------------
  // Pin         mode  Dir.  Drive    Signal            Comment
  // -------------------------------------------------------------------------
  // P1.7/SS0    SPI   (out) 1        AD5421 SYNC
  // P1.6/MOSI0  SPI   (out) 1        AD5421 SDIN
  // P1.5/SCLK0  SPI   (out) 1        AD5421 SCLK
  // P1.4/MISO0  SPI   (in ) -        AD5421 DOUT
  // P1.3        GPIO  in    pull-up  Not connected
  // P1.2        GPIO  in    pull-up  Not connected
  // P1.1/IRQ4   GPIO  in    -        AD5421 FAULT
  // P1.0/ECLKI  GPIO  in    -        AD5700 CLK_OUT
  
  // Set pin function
  pADI_GP1->GPCON = 0x00
    | GP1CON_CON7_SPI0CS          //  AD5421 SYNC
    | GP1CON_CON6_SPI0MOSI        //  AD5421 SDIN
    | GP1CON_CON5_SPI0SCLK        //  AD5421 SCLK
    | GP1CON_CON4_SPI0MISO        //  AD5421 DOUT
    | GP1CON_CON3_GPIO            //  Not connected
    | GP1CON_CON2_GPIO            //  Not connected
    | GP1CON_CON1_GPIOIRQ4        //  AD5421 FAULT
    | GP1CON_CON0_GPIOIRQ3        //  AD5700 CLK_OUT
  ;
  // Enable outputs
  // Note that pin direction may be overdriven by setting the pin function
  pADI_GP1->GPOEN = 0x00
    | GPOEN_OEN7_OUT              //  AD5421 SYNC
    | GPOEN_OEN6_OUT              //  AD5421 SDIN
    | GPOEN_OEN5_OUT              //  AD5421 SCLK
    | GPOEN_OEN4_IN               //  AD5421 DOUT
    | GPOEN_OEN3_IN               //  Not connected
    | GPOEN_OEN2_IN               //  Not connected
    | GPOEN_OEN1_IN               //  AD5421 FAULT
    | GPOEN_OEN0_IN               //  AD5700 CLK_OUT
  ;

  // Set output state
  pADI_GP1->GPOUT = 0x00
    | GPOUT_OUT7_HIGH             //  AD5421 SYNC
    | GPOUT_OUT6_HIGH             //  AD5421 SDIN
    | GPOUT_OUT5_HIGH             //  AD5421 SCLK
    | GPOUT_OUT4_HIGH             //  AD5421 DOUT
    | GPOUT_OUT3_LOW              //  Not connected
    | GPOUT_OUT2_LOW              //  Not connected
    | GPOUT_OUT1_LOW              //  AD5421 FAULT
    | GPOUT_OUT0_LOW              //  AD5700 CLK_OUT
  ;

// Enable pull up resistors
  pADI_GP1->GPPUL = 0x00
    | GP1PUL_PUL7_DIS             //  AD5421 SYNC
    | GP1PUL_PUL6_DIS             //  AD5421 SDIN
    | GP1PUL_PUL5_DIS             //  AD5421 SCLK
    | GP1PUL_PUL4_EN              //  AD5421 DOUT
    | GP1PUL_PUL3_EN              //  Not connected
    | GP1PUL_PUL2_EN              //  Not connected
    | GP1PUL_PUL1_EN              //  AD5421 FAULT
    | GP1PUL_PUL0_DIS              //  AD5700 CLK_OUT
  ;

  // -------------------------------------------------------------------------
  // Port 2
  // -------------------------------------------------------------------------
  // Pin         mode  Dir.  Drive    Signal            Comment
  // -------------------------------------------------------------------------
  // P2.2/Dwnld  GPIO  in    pull-up  Download          Pushbutton S2
  // P2.1        GPIO  out   0        LED
  // P2.0        GPIO  in    -        AD5700 CLK_OUT
  // -------------------------------------------------------------------------

  // Set pin function
  pADI_GP2->GPCON = 0x00
    | GP2CON_CON4_SWDATA 
    | GP2CON_CON3_SWCLK
    | GP2CON_CON2_GPIO            //  Download          Pushbutton S2
    | GP2CON_CON1_GPIO            //  LED
    | GP2CON_CON0_GPIO            //  AD5700 CLK_OUT
  ;

  // Enable outputs
  // Note that pin direction may be overdriven by setting the pin function
  pADI_GP2->GPOEN = 0x00
    | GPOEN_OEN2_IN               //  Download          Pushbutton S2
    | GPOEN_OEN1_OUT              //  LED
    | GPOEN_OEN0_IN               //  AD5700 CLK_OUT
  ;

  // Set output state
  pADI_GP2->GPOUT = 0x00
    | GPOUT_OUT2_LOW              //  Download          Pushbutton S2
    | GPOUT_OUT1_HIGH             //  LED
    | GPOUT_OUT0_LOW              //  AD5700 CLK_OUT
  ;

  // Enable pull up resistors
  pADI_GP2->GPPUL = 0x00
    | GP1PUL_PUL2_EN              //  Download          Pushbutton S2
    | GP1PUL_PUL1_DIS             //  LED
    | GP1PUL_PUL0_DIS             //  AD5700 CLK_OUT
  ;

} // IO_Init

//*****************************************************************************
// Miscelaneous GPIO functions
//*****************************************************************************

void GPIO_LedOn(void)
{
  // Switch on LED
  // P2.1 output low
  pADI_GP2->GPCLR = GPCLR_CLR1;
} // LED_On

void GPIO_LedOff(void)
{
  // Switch off LED
  // P2.1 output high
  pADI_GP2->GPSET = GPSET_SET1;
} // LED_off

unsigned char GPIO_TestState(void)
{
  // Reads status of the test input, test point T4
  // P0.0 input, active low
  return !(pADI_GP0->GPIN & GPIN_IN0_MSK);
} // Test

unsigned char GPIO_DownloadState(void)
{
  // Reads status of the "DOWNLOAD" Pushbutton
  // P2.2 input, active low
  return !(pADI_GP2->GPIN & GPIN_IN2_MSK);
} // Download_State

//*****************************************************************************
// HART support GPIO functions
//*****************************************************************************

void GPIO_RtsOn(void)
{
  // Switch on HART RTS
  // P0.4 output low
  pADI_GP0->GPCLR = GPCLR_CLR4;
} // RTS_On

void GPIO_RtsOff(void)
{
  // Switch off HART RTS
  // P0.4 output high
  pADI_GP0->GPSET = GPSET_SET4;
} // RTS_On

unsigned char GPIO_CdState(void)
{
  // Reads status of HART CD
  // P0.5 input, active high
  return (pADI_GP0->GPIN & GPIN_IN5_MSK);
} // CD_Stat

void GPIO_UartMuxToHart(void)
{
  // Switch UART interface to HART Modem

  // Note that the P0.7 default function is POR (Power-On Reset!!)
  // Therefore, it is safer to switch P0.7 function in one step, 
  // using a temporary storage
  
  unsigned int uiTemp;
  
  // First step: Switch all pins from UART to GPIO
  uiTemp = pADI_GP0->GPCON;
  uiTemp &= ~(GP0CON_CON7_MSK       
            | GP0CON_CON6_MSK
            | GP0CON_CON2_MSK
            | GP0CON_CON1_MSK)
  ;   
  uiTemp |=   GP0CON_CON7_GPIO
            | GP0CON_CON6_GPIOIRQ2
            | GP0CON_CON2_GPIO
            | GP0CON_CON1_GPIO
  ;
  pADI_GP0->GPCON = uiTemp; 

  // Second step: Switch UART TXD to pin P0.7, RXD to pin P0.6
  uiTemp &= ~(GP0CON_CON7_MSK
            | GP0CON_CON6_MSK
            | GP0CON_CON2_MSK
            | GP0CON_CON1_MSK)
  ;
  uiTemp |=   GP0CON_CON7_UARTTXD   //  AD5700 HART TXD
            | GP0CON_CON6_UARTRXD   //  AD5700 HART RXD
            | GP0CON_CON2_GPIO 
            | GP0CON_CON1_GPIO
  ;
  pADI_GP0->GPCON = uiTemp; 

} // UART_mux_to_HART

void GPIO_UartMuxToJ3(void)
{
  // Switch UART interface to connector J3

  // The P0.7 default function is POR (Power On Reset!!)
  // Therefore, it is safer to switch P0.7 function in one step, 
  // using a temporary storage
  
  unsigned int uiTemp;
  
  // First step: Switch all pins from UART to GPIO
  uiTemp = pADI_GP0->GPCON;
  uiTemp &= ~(GP0CON_CON7_MSK       
            | GP0CON_CON6_MSK
            | GP0CON_CON2_MSK
            | GP0CON_CON1_MSK)
  ;   
  uiTemp |=   GP0CON_CON7_GPIO
            | GP0CON_CON6_GPIOIRQ2
            | GP0CON_CON2_GPIO
            | GP0CON_CON1_GPIO
  ;
  pADI_GP0->GPCON = uiTemp; 

  // Second step: Switch UART TXD to pin P0.2, RXD to pin P0.1
  uiTemp &= ~(GP0CON_CON7_MSK
            | GP0CON_CON6_MSK
            | GP0CON_CON2_MSK
            | GP0CON_CON1_MSK)
  ;
  uiTemp |=   GP0CON_CON7_GPIO
            | GP0CON_CON6_GPIOIRQ2
            | GP0CON_CON2_UARTTXD  //  J3-4   UART TXD   Connector J3 pin 4
            | GP0CON_CON1_UARTRXD  //  J3-6   UART RXD   Connector J3 pin 6
  ;
  pADI_GP0->GPCON = uiTemp; 

} // UART_mux_to_J3
