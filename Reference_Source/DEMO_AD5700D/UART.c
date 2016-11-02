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

 File         : UART.c

 Hardware     : ADuCM360

 Description  : UART using interrupt and circular Rx and Tx buffers
                HART code addition specific for DEMO-AD5700D2Z


 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "UART.h"
#include "HART.h"

#define EOL '\r'                  // End of line character
#define EOF (-1)                  // End of file

#define ERRCHAR 0xFF;             // Replace character in case of error
                                  // Comment out if not to replace

volatile unsigned char  ucRxBuf[UART_RX_BUFSIZE]; // Rx buffer
volatile unsigned int   uiRxWrIdx = 0;    // Rx buffer write index
volatile unsigned int   uiRxRdIdx = 0;    // Rx buffer read index

volatile unsigned int   uiRxCount = 0;    // Number of characters in Rx buffer
volatile unsigned int   uiRxLines = 0;    // Number of lines in Rx buffer

volatile unsigned char  ucTxBuf[UART_TX_BUFSIZE]; // Tx buffer
volatile unsigned int   uiTxWrIdx = 0;    // Rx buffer write index
volatile unsigned int   uiTxRdIdx = 0;    // Rx buffer read index

volatile unsigned int   uiTxCount = 0;    // Number of characters in Tx buffer
volatile unsigned char  ucTxActive = 0;   // Tx buffer active flag 

volatile unsigned char  ucErrorFlag = 0;  // Error flag - defined in UART.h

volatile unsigned int   uiUartGapTimer= 0;// Handle in 1ms timer interrupt

// ****************************************************************************
// UART Init
// ****************************************************************************

#define SYSCLK 16000000 // Should be taken from / moved to somewhere global?

void UART_Init(unsigned int uiBaud, char cData, char cParity, char cStop)
{
  // unsigned int uiBaud = 1200;
  unsigned long ulTmp1;
  unsigned long ulTmp2;
  
  pADI_UART->COMIEN = 0x00; // Disable all UART interrupts
  pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DISUARTCLK_MSK; // Enable UART clock

  // --------------------------------------------------------------------------
  // UART buffers
  // --------------------------------------------------------------------------

  uiRxRdIdx = uiRxWrIdx = 0;    // Clear buffer index pointers
  uiRxCount = 0;                // Clear number of characters
  uiRxLines = 0;                // Clear number of lines
  ucErrorFlag = 0;              // Clear error flag
  uiUartGapTimer = 0;           // Stop gap timer
  
  uiTxRdIdx = uiTxWrIdx = 0;    // Clear buffer index pointers
  uiTxCount = 0;                // Clear number of characters
  ucTxActive = 0;               // Clear TX buffer active flag

  // --------------------------------------------------------------------------
  // Baud rate
  // --------------------------------------------------------------------------
  // Optimized for minimum clock frequency => minimum power consumption
  // NOTE: UART clock set in CLKCON1 must be >= uC core clock set in CLKCON0 !!
  // --------------------------------------------------------------------------

  // Calculate Ratio of UART clock frequency to Baud Rate
  ulTmp1 = SYSCLK << 7;       // Multiply by 128, headroom for further rounding
  ulTmp1 >>= pADI_CLKCTL->CLKSYSDIV;                           // System Clock
  ulTmp1 >>= (pADI_CLKCTL->CLKCON1 & CLKCON1_UARTCD_MSK) >> 9; // UART Clock
  ulTmp1 /= uiBaud;           // Temp1 = 128 * (Ratio = UART Clock / Baud Rate)

  // Set UART COMDIV as round(Ratio / 128)
  // This maximizes UART fractional divider value for best Baud accuracy 
  ulTmp2 = ulTmp1 >>13;       // divided by 128, divided by 64
  ulTmp2++;                   // Correction for rounding
  ulTmp2 >>= 1;               // Final division, round(Ratio / 128)
  pADI_UART->COMDIV = ulTmp2; // Write to register
  
  // Set UART COMFBR as round(64 * Ratio / ComDiv)
  ulTmp1 /= ulTmp2;           // 128 * Ratio / ComDiv
  ulTmp1++;                   // Correction for rounding
  ulTmp1 >>= 1;               // Final division, round(64 * Ratio / ComDiv)
  ulTmp1 |=COMFBR_ENABLE;     // Enable fractional divider
  pADI_UART->COMFBR = ulTmp1; // Write to register

  // --------------------------------------------------------------------------
  // UART Format
  // --------------------------------------------------------------------------

   // Here fixed: 8 data bits, Odd parity, 1 stop bit

  pADI_UART->COMLCR = 0 // Enable relevant UART line control bits:
    | COMLCR_BRK_DIS    // Set Break
    | COMLCR_SP_DIS     // Stick Parity
    | COMLCR_EPS_DIS    // Even Parity (disable: Odd Parity)
    | COMLCR_PEN_EN     // Parity Enable 
    | COMLCR_STOP_DIS   // Stop (enable: 2 or 1.5 depending on Word Length)
    | COMLCR_WLS_8BITS  // Word Length Select
  ;

  pADI_UART->COMLCR = 0 // Enable relevant UART line control bits:
    | COMLCR_BRK_DIS    // Set Break
    | COMLCR_SP_DIS     // Stick Parity
  ;
  if (cParity & 2) pADI_UART->COMLCR |= COMLCR_EPS_EN;  // Parity Even 
  if (cParity)     pADI_UART->COMLCR |= COMLCR_PEN_EN;  // Parity Enable 
  if (cStop == 2)  pADI_UART->COMLCR |= COMLCR_STOP_EN; // Extra stop bit
  pADI_UART->COMLCR |= (cData - 5) & COMLCR_WLS_MSK;    // Word Length Select
  


  // -------------------------------------------------------------------------
  // UART interrupt
  // -------------------------------------------------------------------------

  pADI_UART->COMIEN = 0 // Enable relevant UART interrupt sources:
    | COMIEN_EDMAR_DIS  // DMA requests in Tx mode
    | COMIEN_EDMAT_DIS  // DMA requests in Rx mode
    | COMIEN_EDSSI_DIS  // Modem status 
    | COMIEN_ELSI_DIS   // Rx status (Break, Framing, Parity, Overrun error)
    | COMIEN_ETBEI_EN   // Tx buffer empty
    | COMIEN_ERBFI_EN   // Rx buffer full
  ;

   NVIC_EnableIRQ(UART_IRQn); // Enable UART interrupt

} // UART_Init

// ****************************************************************************
// UART buffer functions
// ****************************************************************************

// UART Rx buffer empty status
int UART_RxFull(void)
{
  return(uiRxCount>=UART_RX_BUFSIZE); 
}

// Number of characters in UART RX buffer
int UART_RxCount(void)
{
  return(uiRxCount); 
}

// Number of lines (ended by EOL) in UART RX buffer
int UART_RxLines(void)
{
  return(uiRxLines); 
}

// UART TX buffer full status
int UART_TxFull(void)
{
  return(uiTxCount>=UART_TX_BUFSIZE); 
}

// Number of characters in UART TX buffer
int UART_TxCount(void)
{
  return(uiTxCount); 
}

// Clear RX buffer
void UART_RxClear(void) 
{
  unsigned char ucCOMIEN;
  ucCOMIEN = pADI_UART->COMIEN;   // Local copy of UART interrupt enable
  pADI_UART->COMIEN = 0x00;       // Disable all UART interrupts
  uiRxRdIdx = uiRxWrIdx = 0;      // Clear buffer index pointers
  uiRxCount = 0;                  // Clear number of characters
  uiRxLines = 0;                  // Clear number of lines
  ucErrorFlag = 0;                // Clear error flag
  uiUartGapTimer = 0;             // Stop gap timer
  pADI_UART->COMIEN = ucCOMIEN;   // Restore UART interrupt enable
}

// Clear TX buffer
void UART_TxClear(void) 
{
  unsigned char txCOMIEN;
  txCOMIEN = pADI_UART->COMIEN;   // Local copy of UART interrupt enable
  pADI_UART->COMIEN = 0x00;       // Disable all UART interrupts
  uiTxRdIdx = uiTxWrIdx = 0;      // Clear buffer index pointers
  uiTxCount = 0;                  // Clear number of characters
  ucTxActive = 0;                 // Clear TX buffer active flag
  pADI_UART->COMIEN = txCOMIEN;   // Restore UART interrupt enable
}

// Check for errors
char UART_Error(void)
{
  char cTemp;
  cTemp = ucErrorFlag;            // Copy flag
  ucErrorFlag = 0;                // And clear it
  return(cTemp);
}

// ****************************************************************************
// Get a character from RX buffer
// ****************************************************************************
// Reads a character from the RX buffer if there is a character received

// If there is no character in the RX buffer, the function waits 
// for a character physically received by UART, without any time-out.
// It is recommended to check if there is a character in the RX buffer before 
// calling this function, e.g. using UART_uiRxCount(), for a robust code...

char UART_getkey(void) 
{
  unsigned char ucCOMIEN;
  unsigned char ucChar;

  while (uiRxCount == 0);         // Wait for something to be received  

  ucCOMIEN = pADI_UART->COMIEN;   // Local copy of UART interrupt enable
  pADI_UART->COMIEN = 0x00;       // Disable all UART interrupts
  ucChar = ucRxBuf[uiRxRdIdx++];  // Read character from buffer
  uiRxRdIdx %= UART_RX_BUFSIZE;   // Index roll over
  uiRxCount--;                    // Number of characters in buffer 
  if (ucChar == EOL) uiRxLines--; // Number of lines in buffer
  pADI_UART->COMIEN = ucCOMIEN;   // Restore UART interrupt enable
  return(ucChar);
}

// ****************************************************************************
// Put a character into TX buffer
// ****************************************************************************
// If the UART is not already transmitting, passes the char. directly to UART,
// otherwise writes a characters to the TX buffer 

// If there is no space in the TX buffer, the function waits for the first 
// character in the TX buffer queue to be passed to UART, without any time-out.
// It is recommended to check if there is space in the TX buffer before calling
// this function, e.g. using UART_txFull(), for a robust code...

// Alternative: 
// If there is no space in the TX buffer, the latest charater is lost and 
// the function returns EOF charater as error.

char UART_putchar(char ucChar) 
{
  unsigned char ucCOMIEN;

  while(uiTxCount==UART_TX_BUFSIZE); // Wait for space in TX buffer  

/*
  // If there is no space in the TX buffer, character is lost, returns EOF
  if (uiTxCount == UART_TX_BUFSIZE) // Check if there is space in TX buffer
  {
    return((char) EOF);             // Return with EOF character
  }    
*/
  ucCOMIEN = pADI_UART->COMIEN;     // Local copy of UART interrupt enable
  pADI_UART->COMIEN = 0x00;         // Disable all UART interrupts
  if (ucTxActive)                   // Check if TX active, already transmitting
  {
    ucTxBuf[uiTxWrIdx++] = ucChar;  // Write character to buffer
    uiTxWrIdx %= UART_TX_BUFSIZE;   // Index roll over
    uiTxCount++;                    // Number of characters in buffer 
  }
  else 
  {
    pADI_UART->COMTX = ucChar;      // Transmit first character directly  
    ucTxActive = 1;                 // TX active now, next char to go to buffer
  }
  pADI_UART->COMIEN = ucCOMIEN;     // Restore UART interrupt enable
  return(ucChar);
}

// ****************************************************************************
// UART INTERRUPT
// ****************************************************************************

void UART_Int_Handler ()
{
  volatile unsigned char ucCOMIIR;
  unsigned char ucCOMLSR;
  unsigned char ucChar;
  unsigned char ucErr;

  ucCOMIIR = pADI_UART->COMIIR;   // Read UART Interrupt ID - must be read
  ucCOMLSR = pADI_UART->COMLSR;   // Read Line Status - can be read only once

  // --------------------------------------------------------------------------
  // UART Rx 
  // --------------------------------------------------------------------------

  if (ucCOMLSR & COMLSR_DR_MSK)   // Character received (COMRX full)
  {
    ucChar = pADI_UART->COMRX;        // Read character from UART
    if (uiRxCount < UART_RX_BUFSIZE)  // Check if there is space in buffer
    {
      ucErr = 0;
      // Set relevant error flags
      if (ucCOMLSR & COMLSR_FE_MSK) ucErr |= UART_ERR_FRAMING; // Framing error
      if (ucCOMLSR & COMLSR_PE_MSK) ucErr |= UART_ERR_PARITY;  // Parity error

      #ifdef ERRCHAR
        // For above errors replace character (if ERRCHAR defined)
        if (ucErr) ucChar = ERRCHAR;
      #endif

      // Set more error flags
      if (ucCOMLSR & COMLSR_OE_MSK) ucErr |= UART_ERR_OVERRUN;// OverRun error
      if (uiUartGapTimer==0) ucErr |= UART_ERR_GAP;           // Gap error
      uiUartGapTimer = UART_GAP_TIME;
      
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // ADDED FOR HART 
      #ifdef HART
        if (cHartState == HART_STATE_RX)
          // HART command direct receive handling, not buffered here.
          Hart_Receive(ucChar, ucErr);
        else
          // Buffered UART below

      #endif // no HART
      // End of ADDED FOR HART
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      { // Bracket necessary for HART command option
        ucErrorFlag |= ucErr;           // Cumulative flag
        ucRxBuf[uiRxWrIdx++] = ucChar;  // Store received character into buffer
        uiRxWrIdx %= UART_RX_BUFSIZE;   // Index roll over 
        uiRxCount++;                    // Number of characters in buffer
        if (ucChar == EOL) uiRxLines++; // Number of lines in buffer
      } // Bracket necessary for HART command option
    }
    else 
    {
      // No space in buffer - buffer overrun error
      ucErrorFlag |= UART_ERR_OVERRUN;
    }
  } // Character received

  // --------------------------------------------------------------------------
  // UART Tx
  // --------------------------------------------------------------------------

  if (ucCOMLSR & COMLSR_THRE_MSK)  // Character to be sent (COMTX empty)
  {
    if (uiTxCount > 0)                // Check if there is something to send
    {
      pADI_UART->COMTX = ucTxBuf[uiTxRdIdx++]; // Pass character to UART
      uiTxRdIdx %= UART_TX_BUFSIZE;    // Index roll over
      uiTxCount--;                    // Number of characters in buffer
    }
    else 
    {
      ucTxActive = 0; // Tx passive now - next putchar call will write COMTX
    }
  } // Character to be sent

} // UART_Int_Handler


// ****************************************************************************
// Additional functions - hex numbers transmit (More efficient than printf)
// ****************************************************************************

unsigned char UART_puthex(unsigned char c) 
{
  unsigned char b;
  b = (c >> 4)   | 0x30;        // High nible to ASCII
  if (b > 0x39) b+=0x07;
  UART_putchar(b);              // Send to UART
  b = (c & 0x0F) | 0x30;        // Low nible to ASCII
  if (b > 0x39) b+=0x07;
  UART_putchar(b);              // Send to UART
  return(c);
}

unsigned int UART_puthex_int(unsigned int i) 
{
  // Special data type to be accessed both as bytes or as long
  union {unsigned char B[2]; unsigned int I;} d;
  d.I = i;
  UART_puthex(d.B[0]);
  UART_puthex(d.B[1]);
  return(i);
}

unsigned long UART_puthex_long(unsigned long l) 
{
  // Special data type to be accessed both as bytes or as long
  union {unsigned char B[4]; unsigned long I;} d;
  d.I = l;
  UART_puthex(d.B[0]);
  UART_puthex(d.B[1]);
  UART_puthex(d.B[2]);
  UART_puthex(d.B[3]);
  return(l);
}


