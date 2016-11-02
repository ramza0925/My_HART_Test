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

 File         : HART.h (DEMO-AD5700D2Z_HART.lib)

 Hardware     : DEMO-AD5700D2Z

 Description  : Simplified demo HART interface functions provided "as-is"
                Provided as a compiled library to be linked with the other code
                The C source is proprietary property of Analog Devices

 Status       : Basic functionality checked
******************************************************************************/

#define HART 1

// Device ID
// Should be read from uC flash during the demo intialisation
// Should be different for each individual board
// Referred as pointer, can be assigned to a long variable in the code
extern unsigned long *pulHartDevID;

// Flag for writing changes back to flash
// It is set by HartCmdResponse after receiving command for changing Device ID
// and the main program should write the Device ID back to the flash memory.
extern char cHartStoreIDFlag;

// Device Variables
// Referred as pointers, can be assigned to a long variable in the code
// Updating the assign variables makes it then available to HART as well
extern float *pfHartVarP;
extern float *pfHartVarC;
extern float *pfHartVar1;
extern float *pfHartVar2;
extern float *pfHartVar3;
extern float *pfHartVar4;

// Reset command receive state machine
// Call when Carrier Detect becomes active, prior first HartCmdReceive
void Hart_Reset(void);

// Error codes for cRxErr detected by UART
#define HART_ERR_GAP      0x80
#define HART_ERR_PARITY   0x40
#define HART_ERR_OVERRUN  0x20
#define HART_ERR_FRAMING  0x10
       
// Command receive state machine, should be called with each received character
// cRxChar .. character received by UART
// cRxErr  .. Error(s) associated with the received character
// ONLY LIMITED DEMO FUNCTIONALITY !!!
void Hart_Receive(char cRxChar, char cRxErr);

// Hart command response - generates response message
// ONLY LIMITED DEMO FUNCTIONALITY !!!
//
// The Hart_Response function implements only response to Command 0
// and to ADI Non-Public test commands

int Hart_Response(char* pcBuffer);

// HART slave state machine
// Definitions and references are here to make them available to UART module
// However, allocation and implementation is in the main program
#define HART_STATE_IDLE   0
#define HART_STATE_RX     1
#define HART_STATE_ACK    2
#define HART_STATE_RTS    3
#define HART_STATE_TX     4
#define HART_STATE_OFF    5
extern char cHartState;
extern volatile unsigned int uiHartTimer;
