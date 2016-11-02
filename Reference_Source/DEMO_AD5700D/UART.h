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
//
// NOTE: UART clock set in CLKCON1 must be >= uC core clock set in CLKCON0 !!
// NOTE: UART clock must be enabled in CLKDIS register !!!
// NOTE:  If stdio functions are to use this buffered UART,
//        putc / getc and/or fputc / fgetc functions should be re-defined
//        to call UART_getkey and UART_putchar.
//        In this demo project this is done in "Retarget.c" file
//

#define UART_RX_BUFSIZE 128  // RX buffer size - must be 2^N
#define UART_TX_BUFSIZE 128  // TX buffer size - must be 2^N

#define UART_PARITY_NONE  0
#define UART_PARITY_ODD   1
#define UART_PARITY_EVEN  2

void UART_Init(unsigned int uiBaud, char cData, char cParity, char cStop);

extern char UART_getkey(void);    // Receive a character (from UART RX buffer)
extern char UART_putchar(char c); // Send a character (to UART TX buffer)

extern int UART_RxFull(void);     // UART Rx buffer full status
extern int UART_TxFull(void);     // UART TX buffer full status

extern int UART_RxCount(void);    // Number of characters in RX buffer
extern int UART_RxLines(void);    // Number of lines (ended by EOL) in RX buffer
extern int UART_TxCount(void);    // Number of characters in UART TX buffer

extern void UART_RxClear(void);   // Clear UART RX buffer
extern void UART_TxClear(void);   // Clear UART TX buffer

// Gap: No character received within specified time since start of previous ch.
#define UART_GAP_TIME     20      // Time to detect RX gap, in miliseconds

#define UART_ERR_GAP      0x80    // Gap (error)
#define UART_ERR_PARITY   0x40    // Parity error
#define UART_ERR_OVERRUN  0x20    // Overrun - both UART shift and SW buffer
#define UART_ERR_FRAMING  0x10    // Framing error

extern char UART_Error(void);     // Reads + clers UART error(s)

// Additional functions - hex numbers transmit (More efficient than printf)
extern unsigned char UART_puthex(unsigned char c);
extern unsigned int  UART_puthex_int(unsigned int i);
extern unsigned long UART_puthex_long(unsigned long l);
