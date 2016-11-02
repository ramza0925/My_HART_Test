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

 File         : DEMO-AD5700D2Z.c

 Hardware     : DEMO-AD5700D2Z (ADuCM360 + AD5421 + AD5700)

 Description  : Demo Example Code

 Status       : Functionality checked
******************************************************************************/

#define MSG_INI "Analog Devices DEMO-AD5700D2Z"
#define MSG_VER "Demo firmware v.2 12.Feb 2013"
#define MSG_ENT "Press <enter> for commands..."

#include "ADuCM360.h" // ADuCM360 definitions. Usually supplied with compiler.

#include <stdio.h>    // some standard i/o related definitions, e.g. printf()
#include <ctype.h>    // some standard c definitions, e.g. puts()

#include "GPIO.h"
#include "CLK.h"
#include "TIMER.h"
#include "AFE.h"
#include "AD5421.h"
#include "FLASH.h"
#include "UART.h"
#include "HART.h"

// ----------------------------------------------------------------------------
// Demo parameters & variables
// ----------------------------------------------------------------------------

// Note: For an easy start with the demo, it takes ADC0 Vref directly from AVdd
// ignoring the REF+ and REF- connections. Also ADC0 gain is set to 1. 
// It avoids the ADC0 value and 4-20mA current jumping too much. 
// However, for best performance with your sensor, choose the correct reference
// source and optimum gain. See more ADC0_Init notes in this file & in "AFE.c".
                                
#define ADC0_FS_CODE  268435456 // Full scale code for voltage calculation
#define ADC0_VREF     3.3       // Assumed Vref = AVdd = 3.3V
float   fADC0toVolts;           // ADC0 Code -> Volts
float   fADC0Voltage;           // ADC0 result in Volts

#define ADC0_PC_COEF  625.0     // 100(%) / 0.16(V) .. 0.16V max inp.voltage
#define ADC0_PC_OFFS  0         // 0(%) ~ 0(V)      .. 0.00V min inp.voltage
float   fPercent;               // Analog output current in %

#define ADC1_FS_CODE  268435456 // Full scale code for voltage calculation
#define ADC1_VREF     1.2       // Internal Vref = 1.2V
float   fADC1toVolts;           // ADC1 Code -> Volts
char    cADC1switch;            // switching ADC1 input: 0..Rref, 1..RTD
float   fADC1VoltageRTD;        // ADC1 RTD result in Volts 
float   fADC1VoltageRef;        // ADC1 Ref result in Volts 

#define RREF          5620.0    // Reference resistor value
// RTD temperature calculated only as a linear approximation, 
// using average temperature coefficient 0.00388 /°C
#define RTD_COEF 2.575          // 1(°C) / 0.00388(/°C) / 100(ohm)
#define RTD_OFFS -100.0         // 0(°C) ~ 100(ohm)
float   fRTDtoOhms;             // RTD sensor calculation
float   fRTDResistance;         // RTD sensor result in ohms
float   fRTDTemperature;        // RTD sensor result in °C       

#define LOOP_MIN        4       // Min current
#define LOOP_MAX        20      // Max current
#define LOOP_COEF       0.16    // 16(mA) / 100(%)
#define LOOP_OFFS       4       // 4(mA) ~ 0(%)
float   fLoopCurrent;           // Analog output current in mA

#define DAC_MIN         0x0000  // Max DAC code
#define DAC_MAX         0xFFFF  // Max DAC code
#define DAC_COEF        655.36  // 65536(FS code) / 100(%)
#define DAC_OFFS        0       // 0(code) ~ 0(%)
long    lDacData;               // Analog output code (16-bit data for DAC)

char    cLoopNewData;           // New analog output data flag
char    cLoopSwitch;            // Switching analog output between 4 and 20mA

// Loop voltage: Connected via 20Mohm:1Mohm divider to 8-bit ADC with 2.5V Vref
#define LOOP_V_COEF     0.206   // 2.5(V) /256(FS code) /1(Mohm) *21(Mohm)
#define LOOP_V_OFFS     1.5     // 1.5V..2 diodes in series with loop terminal
#define LOOP_R_SENS     0.052   // (kohm) AD5421 internal resistor

unsigned long ulAD5421Fault;    // AD5421 fault register
float         fAD5421Regin;     // AD5421 Regin input voltage in V
float         fLoopVoltage;     // AD5421 measured (loop) input voltage in V

unsigned long ulDeviceID;       // Demo unique device ID, stored in uC flash
char          ucDeviceStatus;   // Demo status

// ----------------------------------------------------------------------------
// Demo UART interface
// ----------------------------------------------------------------------------

// An extra buffer for message here. It results in a simple and readable code.
// However, this approach increases  amount of SRAM used for UART.
int iMsgLen = 0;                  // Actual UART message length
char cMsg[UART_RX_BUFSIZE];       // Working buffer for UART message

#define CMD_PSW 0xAD      // Password for selected commands
int  iCmdPsw;             // Received password
char cUartCmdEnable = 1;  // Flag for enabling command over UART 
char cUartContData = 0;   // Flag for continuous data transmit to UART
char cUartNewData  = 0;   // Flag for new data to be sent over UART0
unsigned int  uiMBatch;   // Demo manufacturing batch number
unsigned int  uiMUnit;    // Demo manufacturing unit number

// ----------------------------------------------------------------------------
// Demo HART interface
// ----------------------------------------------------------------------------

char cHartState = HART_STATE_IDLE;      // State machine switch
volatile unsigned int uiHartTimer = 0;  // Handle (--) in 1ms timer interrupt
                                        // must be declared as volatile !!

unsigned int  i;                        // index for cycles

// ****************************************************************************
// Main
// ****************************************************************************

int main(void)
{  

// ****************************************************************************
// Initialisation
// ****************************************************************************

  pADI_WDT->T3CON=0;// ADuCM360 watchdog timer disabled
  
  GPIO_Init();      // ADuCM360 general purpose (digital) inputs/outputs
  GPIO_LedOn();     // LED on during init after power-up or reset

  CLK_Init();       // ADuCM360 clocks

  TIMER0_Init();    // ADuCM360 timer, 1ms interrupt


  // --------------------------------------------------------------------------
  // AFE
  // --------------------------------------------------------------------------
  // Analog front end
  // Excitation current sources 2x100uA for RTD (possible 3-wire extern.sensor)
  // Voltage reference configuration
  // Ground switch for primary sensor

  AFE_Init();       // ADuCM360 Analog Front-End 

  // --------------------------------------------------------------------------
  // ADC 0
  // --------------------------------------------------------------------------
  // Measures primary sensor input voltage
  // Input (+) buffered, AIN0, via RC filter from connector J5 pin 3
  // Input (-) buffered, AIN1, via RC filter from connector J5 pin 4
  //
  // For default demo:
  // Gain = 1
  // Vref = AVdd (unbuffered)
  //
  // For demo with pressure sensor (#define PressureSensor in "AFE.h")
  // Gain = 8
  // Vref ext. (unbuffered), VREF+ connector J5 pin 2, VREF- connector J5 pin 5
  //
  // Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch
  // Result every 60ms

  ADC0_Init();
  fADC0toVolts = ADC0_VREF / ADC0_FS_CODE; // Coeff. for voltage calculation

  // --------------------------------------------------------------------------
  // ADC 1
  // --------------------------------------------------------------------------
  // Alternates between RTD and Rref to measure temperature
  // Reference internal 1.2V
  // Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch
  // ADC result every 120ms for each, complete temperature result every 240ms

  ADC1_Init_Rref(); 
  cADC1switch = 0;                          // ADC1 input:0..Rref, 1..RTD
  fADC1toVolts = ADC1_VREF / ADC1_FS_CODE;  // Coeff. for voltage calculation
  
  // --------------------------------------------------------------------------
  // 4-20mA output
  // --------------------------------------------------------------------------
  // AD5421 interfaced via SPI0 - configured SCLK=100kHz, CPOL=low, CPHA=high
  // AD5421 reset, then on-chip ADC enabled to measure loop voltage

  AD5421_Init(AD5421_DEMO_VLOOP);

  // --------------------------------------------------------------------------
  // Device ID
  // --------------------------------------------------------------------------

  // Try to read Unique Device ID from ADuCM360 FLASH
  // The number is set by Analog Devices when the board is manufactured
  ReadFlash(&ulDeviceID);

  // --------------------------------------------------------------------------
  // UART
  // --------------------------------------------------------------------------

  UART_Init(9600, 8, UART_PARITY_NONE, 1); 
                    // Send initial message to UART


  putchar('\n');    // New line
  puts(MSG_INI);    // Initial Message
  puts(MSG_ENT);    // Enter Message
    
  while(UART_TxCount());// Wait for UART to transmit 
  // Not necessary to wait, but it keeps the LED on longer...

  // --------------------------------------------------------------------------
  // HART  
  // --------------------------------------------------------------------------
#ifdef HART

    // Map HART device ID
    pulHartDevID = &ulDeviceID; 

    // Map HART variables
    pfHartVarP = &fPercent; 
    pfHartVarC = &fLoopCurrent; 
    pfHartVar1 = &fADC0Voltage;
    pfHartVar2 = &fRTDTemperature;
    pfHartVar3 = (float*) &ulAD5421Fault; // Only, ONLY for ADI demo...
    pfHartVar4 = &fLoopVoltage;

#endif // HART

  GPIO_LedOff();

// ****************************************************************************
// MAIN LOOP
// ****************************************************************************

  while(1)
  {
    // ************************************************************************
    // ************************************************************************
    // Primary Sensor
    // ************************************************************************
    // ************************************************************************

    if (ucADC0NewData)    // Check new data flag, set in ADC0 interrupt
    {
      // ----------------------------------------------------------------------
      // ADC0 has new data 
      // ----------------------------------------------------------------------
      ucADC0NewData = 0;  // Clear the flag
      
      // ----------------------------------------------------------------------
      // Translate ADC data to Voltage
      // Note that ADuCM360 input is limited to max ±1V / Gain for Gain > 1
      // => ADuCM360 input limited to max ±125mV @ Gain=8
      // ----------------------------------------------------------------------

      fADC0Voltage = (lADC0Data * fADC0toVolts); // Volts
      
      // ----------------------------------------------------------------------
      // Translate ADC0 Voltage to Percent (of full scale)
      // ----------------------------------------------------------------------

      fPercent = fADC0Voltage * ADC0_PC_COEF + ADC0_PC_OFFS;
    
      // ----------------------------------------------------------------------
      // Flag for new analog output data
      // When a complete primary result available, every 60ms
      // ----------------------------------------------------------------------

      cLoopNewData = 1;     // Set the new analog output data flag 

    } // if (ucADC0NewData)


    // ************************************************************************
    // ************************************************************************
    // Secondary Sensor
    // ************************************************************************
    // ************************************************************************

    if (ucADC1NewData)      // Check new data flag, set in ADC1 interrupt
    {
      // ----------------------------------------------------------------------
      // ADC1 has new data 
      // ----------------------------------------------------------------------
      ucADC1NewData = 0;    // Clear the flag

      if (cADC1switch)      // switching ADC1 input: 0..Rref, 1..RTD
      {      
        // --------------------------------------------------------------------
        // ADC1 next conversion should be on Rref 
        // --------------------------------------------------------------------

        cADC1switch = 0;    // change switch 
        ADC1_Init_Rref();   // reconfigure ADC
        // Measure Rref (R9 5.62kohm 10ppm) voltage
        // Input(+) buffered, AIN7, via RC filter to R9 and connector J1 pin 1
        // Input(-) buffer bypassed, AIN8, AGND (and R9 and connector J1 pin 4)
        // Gain = 1
        // Reference internal 1.2V
        // Conversion contin., chopping, filter sin3 50Hz with extra 60Hz notch

        // --------------------------------------------------------------------
        // ADC1 previous data are measured on RTD
        // Translate ADC data to Voltage
        // --------------------------------------------------------------------

        fADC1VoltageRTD = (lADC1Data * fADC1toVolts); // Volts

        // --------------------------------------------------------------------
        // ADC1 previous data are measured on RTD
        // Translate data to Resistance
        // --------------------------------------------------------------------

        fRTDResistance = fRTDtoOhms * lADC1Data;

        // --------------------------------------------------------------------
        // Translate RTD Resistance to Temperature
        //
        // Linear approximation, good enough for demo:
        // Pt100, average 2.575'C / ohm, 100 ohm .. 0'C
        // +/-0.1'C   error in -10 to +50'C
        // +/-0.5'C   error in -40 to +85'C
        // +1.0'C     error at +105'C
        // +2.5°C     error at +150°C
        // Can be improved by implementing a non-linear approximation
        // --------------------------------------------------------------------

        fRTDTemperature = RTD_COEF * (fRTDResistance + RTD_OFFS);

        // ----------------------------------------------------------------------
        // Flag for new UART data (if UART used for sending data)
        // When a complete secondary result available, every 240ms
        // ----------------------------------------------------------------------

        cUartNewData = 1;  

      } // if (cADC1switch)
      else
      {            
        // --------------------------------------------------------------------
        // ADC1 next conversion should be on RTD
        // --------------------------------------------------------------------
        cADC1switch = 1;  // change switch 
        ADC1_Init_RTD();  // reconfigure ADC

        // Measures RTD (PT100) voltage
        // Input(+) buffered, AIN3, via RC filter to RTD and connector J1 pin 3
        // Input(-) buffered, AIN2, via RC filter to RTD and connector J1 pin 2
        // Gain = 16
        // Reference internal 1.2V
        // Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch

        // --------------------------------------------------------------------
        // ADC1 previous data are measured on Rref
        // Translate ADC data to Voltage
        // ----------------------------------------------------------------------

        fADC1VoltageRef = (lADC1Data * fADC1toVolts); // Volts

        // --------------------------------------------------------------------
        // ADC1 previous data are measured on Rref
        // Translate data to coefficient for next resistance calculation
        // The coefficient 2.0 comes from that both excitation currents flow
        // through the resistor for a "3-wire" RTD configuration
        // ----------------------------------------------------------------------

        fRTDtoOhms = 2.0 * RREF / lADC1Data;

      } // else (cADC1switch)
      
    } // if (ucADC1NewData)

    // ************************************************************************
    // ************************************************************************
    // 4-20mA loop output
    // ************************************************************************
    // ************************************************************************

    if (cLoopNewData)  // Check new analog output data flag
    {
      cLoopNewData = 0;  // Clear the flag 

      lDacData     = fPercent * DAC_COEF  + DAC_OFFS;
      fLoopCurrent = fPercent * LOOP_COEF + LOOP_OFFS;

      // ----------------------------------------------------------------------
      // Analog output limits - analog output clamped to 4 to 20mA range
      // ----------------------------------------------------------------------

      if (lDacData  < DAC_MIN)
      {
        lDacData    = DAC_MIN;      // Clamp to low limit
        fLoopCurrent= LOOP_MIN;
      }
      if (lDacData  > DAC_MAX) 
      {
        lDacData    = DAC_MAX;      // Clamp to high limit
        fLoopCurrent= LOOP_MAX;
      }

      // ----------------------------------------------------------------------
      // Test mode - periodic switching between 4 and 20mA
      // Active when pressed "Download" push-button 
      // or when test point T4 shorted with test point T3 (local(!) ground)  
      // ----------------------------------------------------------------------
      if (GPIO_DownloadState() || GPIO_TestState())
      {   
//        GPIO_LedOn();           // Turn on LED to indicate test mode
        if (cLoopSwitch)          // Periodically switch the output between
        {
          cLoopSwitch = 0;
          lDacData = DAC_MAX;     // Force Full scale (20mA)
          fLoopCurrent = -12;     // Fixed -12mA to indicate test mode
        }
        else
        {
          cLoopSwitch = 1;
          lDacData = DAC_MIN;     // Force Zero scale (4mA)
          fLoopCurrent = -12;     // Fixed -12mA to indicate test mode
        }
      } // if (Download)
      else
      {
//        GPIO_LedOff();          // Turn off LED
      } // else (Download)

      // ----------------------------------------------------------------------
      // AD5421 diagnostics
      // ----------------------------------------------------------------------

      // Utilizing the AD5421 fault register readback at every new data write
      // Get the last value
      ulAD5421Fault = AD5421_LastFlt();

      // AD5421 measured (loop) voltage, 8 LSB of fault register
      fAD5421Regin = (ulAD5421Fault & 0xFF) * LOOP_V_COEF;
      // Correction for diodes in series with the loop terminals
      fLoopVoltage = fAD5421Regin + LOOP_V_OFFS;
      // Correction for AD5421 on-chip Rsense * Output current voltage drop
      fLoopVoltage += fLoopCurrent * LOOP_R_SENS;
      // ----------------------------------------------------------------------
      // AD5421 output
      // ----------------------------------------------------------------------

      AD5421_Output(lDacData);    // 4-20mA output, 
      // AD5421_Output(0x8000);   // DEBUG: Fixed 12mA output


    } // if (cLoopNewData)


    // ************************************************************************
    // ************************************************************************
    // Simple UART, ASCII terminal
    // ************************************************************************
    // ************************************************************************

    // ------------------------------------------------------------------------
    // Send data to UART
    // ------------------------------------------------------------------------

    if (cUartContData && cUartNewData) // Should be data sent to UART ?
    {
      cUartNewData = 0;           // Clear the flag

      // Items delimited by TAB character, finished by new line
      printf("%+6.3fV\t",fADC0Voltage);
      printf("%+7.2f'C\t",fRTDTemperature);
      putchar('\n');
    } // if (data sent to UART)

    // ------------------------------------------------------------------------
    // Received new command (line)
    // ------------------------------------------------------------------------
    
    if (cUartCmdEnable)
    {  
      if (UART_RxLines())         // Is there a new line in Rx buffer?
      {
        cUartContData = 0;        // For any line (<ENTER> pressed) stop data
        gets((char*)cMsg);        // Read the line
        switch (toupper(cMsg[0])) // Decode the first character as a command
        {                         // (uppercase / lowercase insensitive)

          // ------------------------------------------------------------------
          // D .. Data
          // ------------------------------------------------------------------
          case 'D':
            putchar('\n');
            puts("D..Data");
            puts("Press <enter> to stop continuous data\n");
            puts("Prim(V)\t RTD('C)\n");
            cUartContData = 1;
          break;

          // ------------------------------------------------------------------
          // F .. AD5421 Fault (Diagnostics)
          // ------------------------------------------------------------------
          case 'F':
            putchar('\n');
            puts("F..Fault (Diagnostics)");  
            printf("AD5421 Fault Reg. 0x%04lX\t",ulAD5421Fault);   
            printf("REGIN Voltage %4.1fV\t",fAD5421Regin);   
            printf("Loop Voltage %4.1fV\n",fLoopVoltage);
          break;

          // ------------------------------------------------------------------
          // I .. Information
          // ------------------------------------------------------------------
          case 'I':
            putchar('\n');
            puts("I..Info");
            puts(MSG_INI);            // Initial Message
            puts(MSG_VER);            // Version Message
            printf("Batch No: %05d ",ulDeviceID>>8);          
            printf("Unit No:   %02d\n",(char)ulDeviceID);
            puts(MSG_ENT);            // Enter Message 
          break;

          // ------------------------------------------------------------------
          // N .. Manufacturing number - command not advertised
          // ------------------------------------------------------------------
          case 'N':
            putchar('\n');
            if(
              // Skip first character, decode the rest of the command line:
              (3==sscanf((char*)(cMsg+1),"%x%d%d",&iCmdPsw,&uiMBatch,&uiMUnit))
                                      // There must be 3 parameters
              && (iCmdPsw == CMD_PSW) // Password, must match defined value
              && (uiMBatch > 0)       // Batch, must be in the defined range
              && (uiMBatch < 0xFFFF)
              )
            {
            // Translate ADI manufacturing Batch and Unit number to device ID
            ulDeviceID  = uiMBatch << 8;  // Manufact.Batch No. MSB
            ulDeviceID |= uiMUnit & 0xFF; // Manufact.Unit No.
            WriteFlash(&ulDeviceID);      // Store the new device ID to FLASH

            printf("Batch No: %05d ",ulDeviceID>>8);          
            printf("Unit No:   %02d\n",(char)ulDeviceID);
            }
            else
              puts("N AD <Batch No> <Unit No> <enter>");
          break;

          // ------------------------------------------------------------------
          // O .. Output Current
          // ------------------------------------------------------------------
          case 'O':
            putchar('\n');
            puts("O..Output");
            if (fLoopCurrent<0)
              puts("Test Mode, Current Pulsing 4 <> 20mA");
            else
            {
              printf("(%+7.2f%%)\t",fPercent);
              printf("AD5421 DAC reg. 0x%04lX\t",lDacData);
              printf("Loop Current %6.3fmA\n",fLoopCurrent);
            }
          break;

          // ------------------------------------------------------------------
          // P .. Primary Input
          // ------------------------------------------------------------------
          case 'P':
            putchar('\n');
            puts("P..Primary Input");
            printf("Vref (=AVdd) %+3.1fV ",ADC0_VREF);
            printf("(Chosen) Full Scale %+5.3fV\t",100/ADC0_PC_COEF);            
            printf("Vin %+9.6fV\t",fADC0Voltage);
            printf("%+7.2f%%\n",fPercent);
          break;

          // ------------------------------------------------------------------
          // T .. Temperature
          // ------------------------------------------------------------------
          case 'T':
            putchar('\n');
            puts("T..Temperature");
            printf("Rref %+9.6fV\t",fADC1VoltageRef);
            printf("RTD %+9.6fV\t",fADC1VoltageRTD);
            printf("Rref %5.0fohm\t",RREF);
            printf("RTD %6.1fohm\t",fRTDResistance);
            printf("%+7.2f'C\n",fRTDTemperature);
          break;

          // ------------------------------------------------------------------
          // Anything else - undefined command
          // ------------------------------------------------------------------
          default:
            // Undifined command - show the list of implemented commands 
            putchar('\n');
            puts("UART Commands:");
            puts("D..Data");
            puts("F..Fault (Diagnostics)");  
            puts("I..Info");
            puts("O..Output");
            puts("P..Primary Input");
            puts("T..Temperature");
            puts("always finish with <enter>");
          break;

        } // switch UART first character as a command

      } // if new line
      else
      { 
        // --------------------------------------------------------------------
        // Prevent UART input from locking
        // When no new line received and input buffer full - clear it...
        // --------------------------------------------------------------------
        if (UART_RxFull())
          UART_RxClear();
      } 
    }
      
    // ************************************************************************
    // ************************************************************************
    // Demo HART, 
    // ************************************************************************
    // ************************************************************************

    // Note: 
    // The code is only a very simplified implementation of HART slave.
    // This is not, by any means, a code compliant with HART specifications.
    // The only purpose of this code is to enable evaluation of the hardware.
    // 
    // Only a basic state machine for receiving and sending via HART modem
    // is implemented in the code below.
    //
    // The Hart_Response function implements only response to Command 0
    // and to ADI Non-Public test commands

#ifdef HART
    // ------------------------------------------------------------------------
    // HART slave state machine 
    // Waits for carrier detect (CD) signal from HART modem to become active
    // When CD is active, the UART is rederected to HART modem and starts 
    // receiving the incoming HART message from master via modem and UART.
    // The receive function is called in UART interrupt after each character.
    // When CD signal goes back to idle state, a response message is generated
    // and send via UART to HART. HART modem RTS signal has to be set active
    // before transmitting and reset to passive after trasmnision is finished.
    // Note that transmsion is finished only after the last character leaves
    // the UART shift register. (And that does not generate any interrupt...)
    // ------------------------------------------------------------------------
    switch (cHartState)
    {
      // ----------------------------------------------------------------------
      // Wait for HART Carrier Detected. 
      // When active, switch UART from J3 to HART modem (and start receiving)
      // ----------------------------------------------------------------------
      case HART_STATE_IDLE:   // Wait for carrier detect 
        if (GPIO_CdState())   // Carrier detected ?
        {
//        GPIO_LedOn();
          cHartState = HART_STATE_RX;
          cUartCmdEnable = 0; // Disable UART (J3) command handling
          cUartContData  = 0; // Disable UART (J3) continuous data transmitting
          Hart_Reset();       // Reset HART state machine
          GPIO_UartMuxToHart(); // Switch UART from connector J3 to HART
          UART_Init(1200, 8, UART_PARITY_ODD, 1); 
          // If not changing UART settings, clearing buffers is enough
          // UART_RxClear();    // Clear UART buffers
          // UART_TxClear();
        } // if .. Carrier detected
      break;  // HART_STATE_IDLE

      // ----------------------------------------------------------------------
      // When Carrier Detect stops, generate response message
      // ----------------------------------------------------------------------
      case HART_STATE_RX:     // Wait for carrier stop
        if (!GPIO_CdState())  // Carrier stopped ? 
        {
//        GPIO_LedOff();
          uiHartTimer = 0;    // Set timer - min. time before sending response
                              // Change to non-zero if, for whatever reason,
                              // the response needs to be delayed
          
          iMsgLen = Hart_Response(&cMsg[0]); // Generate response
          if (iMsgLen)        // If succesfull, procede to sending response
            cHartState = HART_STATE_ACK;
          else                // Otherwise revert to idle
            cHartState = HART_STATE_IDLE;
          if (cHartStoreIDFlag)      // If a command to change ID was received
            WriteFlash(&ulDeviceID); // Store the (new) device ID to FLASH
        } //if .. Carrier stopped

      break;  // HART_STATE_RX

      // ----------------------------------------------------------------------
      // Wait defined minimum time before sending response, then switch RTS on
      // ----------------------------------------------------------------------
      case HART_STATE_ACK:
        if (!uiHartTimer)     // Timer elapsed?
        { 
          cHartState = HART_STATE_RTS;
          GPIO_RtsOn();       // Switch on HART RTS (Request to send)
          uiHartTimer = 3;    // Set timer - min time before 1st character
        }
      break;  // HART_STATE_ACK


      // ----------------------------------------------------------------------
      // Wait a defined period after RTS On, then transmit response message
      // ----------------------------------------------------------------------
      case HART_STATE_RTS:    // Wait after RTS On
        if (!uiHartTimer)     // Timer elapsed?
        {
          cHartState = HART_STATE_TX;   
          // Send prepared response
          // Copy message to UART buffer byte by byte - not really efficient
          // but simple use of existing UART code with the circular buffer...
          for (i = 0; i < iMsgLen; i++)
            UART_putchar(cMsg[i]);
        }
      break;  // HART_STATE_RTS  

      // ----------------------------------------------------------------------
      // Wait for response message to transmit
      // ----------------------------------------------------------------------
      case HART_STATE_TX:     // Wait for message to be sent
        if (!UART_TxCount())  // Transmit finished ?
        {
          // Transmit finished
          cHartState = HART_STATE_OFF;
          uiHartTimer = 20;   // Set timer - min time before RTS off

          // The ADuCM360 UART generates interrupt only when a character
          // is moved from the COMTX register to the transmit shift register
          // When the transmit from the shift register is complete, there is a
          // bit set (TEMT bit in COMLSR register), but there is no interrupt.
          // Therefore, the RTS switch off is derived from a timer rather than
          // from the UART interrupt.
          //
          // This code uses double buffered UART, and the "transmit finished"
          // based on software buffer empty (TxCount = 0) happens when the 2nd
          // last character starts to transmit. Therefore, the RTS needs to be
          // actually delayed by two characters time:
          // (2 char * 11 bits /1200Bd) = 18.3ms + a small extra delay => 20ms.

        }
      break; // HART_STATE_TX

      // ----------------------------------------------------------------------
      // Wait for response message transmit to finish
      // Switch UART back from HART modem to connector J3
      // ----------------------------------------------------------------------
      case HART_STATE_OFF:    // Wait after the last response character sent
        if (!uiHartTimer)     // Timer elapsed?
        {  
          cHartState = HART_STATE_IDLE;
          GPIO_RtsOff();      // Switch off HART RTS (Request to send)
          GPIO_UartMuxToJ3(); // Switch UART from HART to connector J3
          UART_Init(9600, 8, UART_PARITY_NONE, 1); 
          // If not changing UART settings, clearing buffers is enough
          // UART_RxClear();    // Clear UART buffers
          // UART_TxClear();     
          cUartCmdEnable = 1; // Enable UART (J3) command handling
        }
      break; // HART_STATE_OFF

    } // switch (cHartState)

#endif // HART

  } // while(1) .. main loop

} // main
