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

 File         : Timer.c

 Hardware     : ADuCM360

 Description  : ADuCM360 Timer functions
                Code specific for DEMO-AD5700D2Z

 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "TIMER.h"

// ****************************************************************************
// Timer 0 INITIALISATION
// ****************************************************************************
void TIMER0_Init(void)
{
  pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DIST0CLK_MSK; // Enable Timer0 clock

  pADI_TM0->LD = 32;    // 1mS period @ 32kHz clock

  pADI_TM0->CON  = 0
    | TCON_EVENTEN_DIS  // Disable time capture of an event
    | T0CON_EVENT_T2    // Select event
    | TCON_RLD_DIS      // DIS..reload only on timeout,EN..on write to TxCLRI  
    | TCON_CLK_LFOSC    // Clock source select
    | TCON_ENABLE_EN    // Timer enable
    | TCON_MOD_PERIODIC // Timer mode
    | TCON_UP_DIS       // DIS..Count down, EN..Count up
    | TCON_PRE_DIV1     // Prescaler, clock source divided by..
  ;

  NVIC_EnableIRQ(TIMER0_IRQn);  // Enable Timer 0 interrupt
}

// ****************************************************************************
// Timer 1 INITIALISATION
// ****************************************************************************
void TIMER1_Init(void)
{
  pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DIST1CLK_MSK; // Enable Timer0 clock

  pADI_TM1->LD = 32;    // 1mS period @ 32kHz clock

  pADI_TM1->CON  = 0
    | TCON_EVENTEN_DIS  // Disable time capture of an event
    | T1CON_EVENT_FEE   // Select event
    | TCON_RLD_DIS      // DIS..reload only on timeout,EN..on write to TxCLRI  
    | TCON_CLK_LFOSC    // Clock source select
    | TCON_ENABLE_EN    // Timer enable
    | TCON_MOD_PERIODIC // Timer mode
    | TCON_UP_DIS       // DIS..Count down, EN..Count up
    | TCON_PRE_DIV1     // Prescaler, clock source divided by..
  ;

  NVIC_EnableIRQ(TIMER1_IRQn);  // Enable Timer 0 interrupt
}

// ****************************************************************************
// Timer 0 INTERRUPT
// ****************************************************************************

void GP_Tmr0_Int_Handler()
{
  extern volatile void AD5421_Step(void);
  extern volatile unsigned int uiHartTimer;
  extern volatile unsigned int uiUartGapTimer;

  pADI_TM0->CLRI = TCLRI_CAP | TCLRI_TMOUT;  // Clear events

  AD5421_Step();                        // AD5421 4-20mA output step filter 

  if (uiHartTimer) uiHartTimer--;       // Timing for HART interface
  if (uiUartGapTimer) uiUartGapTimer--; // Timing for UART gap detection
}

// ****************************************************************************
// Timer 1 INTERRUPT
// ****************************************************************************
void GP_Tmr1_Int_Handler()
{
  pADI_TM1->CLRI = TCLRI_CAP | TCLRI_TMOUT;  // Clear events
}

