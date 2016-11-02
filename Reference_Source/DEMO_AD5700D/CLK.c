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

 File         : CLK.c

 Hardware     : ADuCM360

 Description  : ADuCM360 clock related functions

 Status       : Funtionality checked
******************************************************************************/

#include "CLK.h"

#include "ADuCM360.h"

//*****************************************************************************
// ADuCM360 clock initialisation
//*****************************************************************************
void CLK_Init(void)
{
  // Disable Watchdog timer
  pADI_WDT->T3CON = 0; 

  // Enable system clock divider by 2 (to minimize power consumption)
  pADI_CLKCTL->CLKSYSDIV = CLKSYSDIV_DIV2EN_EN;

  // Set system clock and CPU clock 
  pADI_CLKCTL->CLKCON0 = 0
    | CLKCON0_CLKOUT_UCLKCG // CLKOUT pin = CPU clock (default option)
    | CLKCON0_CLKMUX_HFOSC  // HF osc.  16MHz
    | CLKCON0_CD_DIV8       // CPU clock divider                            
  ;                         // CPU: 16MHz /2(CLKSYSDIV) /8(CLKCON0_CD) = 1MHz

  // Set peripherals clock frequency
  // NOTE: Any peripheral clock must be >= uC core clock set in CLKCON0 !!
  pADI_CLKCTL->CLKCON1 = 0
    | CLKCON1_PWMCD_DIV8
    | CLKCON1_UARTCD_DIV8
    | CLKCON1_I2CCD_DIV8
    | CLKCON1_SPI1CD_DIV8
    | CLKCON1_SPI0CD_DIV8
  ;
  
//Commented out as clocks are enabled as part individual peripheral init. 

// Disable / Enable clock for peripherals
  pADI_CLKCTL->CLKDIS = 0
    | CLKDIS_DISADCCLK_DIS  // Disable-disable => Enable :-) ADC clock
    | CLKDIS_DISDMACLK 
    | CLKDIS_DISDACCLK 
    | CLKDIS_DIST1CLK_DIS   // Disable-disable (=> Enable :-) Timer 1 clock
    | CLKDIS_DIST0CLK_DIS   // Disable-disable (=> Enable :-) Timer 0 clock
    | CLKDIS_DISPWMCLK 
    | CLKDIS_DISUARTCLK_DIS // Disable-disable (=> Enable :-) UART clock
    | CLKDIS_DISI2CCLK 
    | CLKDIS_DISSPI1CLK
    | CLKDIS_DISSPI0CLK_DIS // Disable-disable (=> Enable :-) SPI0 clock
  ;
}
