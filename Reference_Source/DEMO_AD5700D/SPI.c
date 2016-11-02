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

 Date         : December 2012

 File         : SPI.c

 Hardware     : ADuCM360

 Description  : SPI master using interrupt

 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "SPI.h"

volatile unsigned char ucSPI0_TxActive; // SPI flag
volatile unsigned char ucSPI1_TxActive; // SPI flag

// ****************************************************************************
// SPI Init
// Configures either SPI0 or SPI1 as a master. Transfer initiated by SPI Write
// ****************************************************************************

void SPI_Init(ADI_SPI_TypeDef *pSPI)   // SPI Init
{
  // Enable SPI clock
  if (pSPI == pADI_SPI0)
    pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DISSPI0CLK_MSK; // Enable SPI0 clock
  if (pSPI == pADI_SPI1)
    pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DISSPI1CLK_MSK; // Enable SPI1 clock

  // Set SCLK frequency, 
  // SPI clock frequency is set by combination of CLKSYSDIV, CLKCON0, CLKCON1
  // SCLK frequency is then = SPI_clock / (2x(1+SPIxDIV))
  pSPI->SPIDIV = 0x04;    // SCLK divided by 10, SCLK 100kHz at SPI clock 1MHz

  // Configure SPI
  pSPI->SPICON = 0
    | SPICON_MOD_TX3RX3   // IRQ mode - after 3 bytes transmitted/received
    | SPICON_TFLUSH_EN    // TX FIFO flush - disable in next step 
    | SPICON_RFLUSH_EN    // RX FIFO flush - disable in next step
    | SPICON_CON_EN       // Continuous transfer
    | SPICON_LOOPBACK_DIS // Loopback Mode
    | SPICON_SOEN_DIS     // Slave MISO output
    | SPICON_RXOF_DIS     // RX oveflow overwrite
    | SPICON_ZEN_EN       // Transmit zeros when empty
    | SPICON_TIM_TXWR     // Transfer and interrupt mode: start by RXRD or TXWR
    | SPICON_LSB_DIS      // LSB first transfer mode (disabled => MSB first)
    | SPICON_WOM_DIS      // Wired OR mode
    | SPICON_CPOL_LOW     // Clock polarity mode
    | SPICON_CPHA_SAMPLETRAILING // High =SAMPLETRAILING, Low = SAMPLELEADING
    | SPICON_MASEN_EN     // Master mode
    | SPICON_ENABLE_EN    // SPI enable
  ;

  pSPI->SPICON &=~(SPICON_TFLUSH_MSK|SPICON_RFLUSH_MSK);  // Finish FIFOs flush

  if (pSPI == pADI_SPI0)
  {
    ucSPI0_TxActive = 0;        // Clear flag
    NVIC_EnableIRQ(SPI0_IRQn);  // Enable SPI0 interrupt
  }
  if (pSPI == pADI_SPI1)
  {
    ucSPI1_TxActive = 0;        // Clear flag
    NVIC_EnableIRQ(SPI1_IRQn);  // Enable SPI1 interrupt
  }
} // SPI_Init


// ****************************************************************************
// SPI Write
// Writes new data to SPI to be transmitted
// Returns data read from the previous SPI transfer
// ****************************************************************************

unsigned long SPI_Write(ADI_SPI_TypeDef *pSPI, unsigned long TxData)
{
  unsigned iNumber;              // Number of bytes per transfer
  int i;
  unsigned long RxData = 0;
 
  // Get number of bytes to read and write from SPICON register
  iNumber = (pSPI->SPICON & SPICON_MOD_MSK) >> 14; 

  // Wait for the previous TX to complete
  // XXXXXXXXX Note: THERE IS NO TIMEOUT!!!
  // XXXXXXXXX If SPI is not initiated properly, it will wait here forever!

  if (pSPI == pADI_SPI0)
  {
    while (ucSPI0_TxActive);  // Wait for flag to be cleared in SPI interrupt
    ucSPI0_TxActive = 1;      // Set the flag again
  }
  if (pSPI == pADI_SPI1)
  {
    while (ucSPI1_TxActive);  // Wait for flag to be cleared in SPI interrupt
    ucSPI1_TxActive = 1;      // Set the flag again
  }

  // Read the previous data
  for (i = iNumber; i>=0; i--)
  {
    RxData <<= 8;             // Shift
    RxData |= pSPI->SPIRX;    // Read from SPI register
  }

  // Write new data to be transmitted
  for (i = iNumber; i>=0; i--)
  {
    // Shift data and write to SPI register
    pSPI->SPITX = (unsigned char) (TxData >> (i<<3));
  }
  return (RxData);

} // SPI_Write

// ****************************************************************************
// SPI Read
// Writes new data to SPI to be transmitted
// Returns data read from the previous SPI transfer
// ****************************************************************************

unsigned long SPI_Read(ADI_SPI_TypeDef *pSPI)
{
  unsigned iNumber;              // Number of bytes per transfer
  int i;
  unsigned long RxData = 0;
 
  // Get number of bytes to read and write from SPICON register
  iNumber = (pSPI->SPICON & SPICON_MOD_MSK) >> 14; 

  // Wait for the previous TX to complete
  // XXXXXXXXX Note: THERE IS NO TIMEOUT!!!
  // XXXXXXXXX If SPI is not initiated properly, it will wait here forever!
  if (pSPI == pADI_SPI0)
  {
    while (ucSPI0_TxActive);  // Wait for flag to be cleared in SPI interrupt
  }
  if (pSPI == pADI_SPI1)
  {
    while (ucSPI1_TxActive);  // Wait for flag to be cleared in SPI interrupt
  }

  // Read the previous data
  for (i = iNumber; i>=0; i--)
  {
    RxData <<= 8;             // Shift
    RxData |= pSPI->SPIRX;    // Read from SPI register
  }

  return (RxData);

} // SPI_Read

// ****************************************************************************
// SPI INTERRUPT
// ****************************************************************************
void SPI0_Int_Handler ()
{
  unsigned short usSPISTA;
  usSPISTA = pADI_SPI0->SPISTA; // Read the SPI status - only once

  // Handle depending on the SPI status
  if ((usSPISTA & SPISTA_TX_MSK) == SPISTA_TX)  // Transmit done
  {
    ucSPI0_TxActive = 0;                        // Clear the flag
  }
}

void SPI1_Int_Handler ()
{
  unsigned short usSPISTA;
  usSPISTA = pADI_SPI1->SPISTA; // Read the SPI status - only once

  // Handle depending on the SPI status
  if ((usSPISTA & SPISTA_TX_MSK) == SPISTA_TX)  // Transmit done
  {
    ucSPI1_TxActive = 0;                        // Clear the flag
  }
}
 
