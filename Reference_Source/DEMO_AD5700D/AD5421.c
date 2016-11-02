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

 File         : AD5421.c

 Hardware     : ADuCM360 + AD5421

 Description  : AD5421 interfaced to ADuCM360 SPI

 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "AD5421.h"
#include "SPI.h"

// Last fault read from the AD5421, to utilize Auto fault readback
unsigned int uiAD5421_Flt;

// SPI port that was used at init - no need to set SPI port for each function
ADI_SPI_TypeDef *pAD5421_SPI;

// ****************************************************************************
// Initialise AD5421 (and relevant SPI)
// ****************************************************************************
void AD5421_Init(unsigned short usCtrl /* ,ADI_SPI_TypeDef *pSPI */)
{
  short i;
  pAD5421_SPI = pADI_SPI0;        // Fixed SPI interface - not a parameter
//pAD5421_SPI = pSPI;             // SPI port from parameter
  SPI_Init(pAD5421_SPI);          // Initialise relevant SPI interface
  AD5421_Write(AD5421_RESET,0);   // Reset AD5421
  SPI_Read(pAD5421_SPI);          // Wait for transfer to finish + dummy read
  for (i=0; i<20; i++);           // Wait for reset to complete, ~120uS @ 1MHz
  AD5421_Write(AD5421_WR_CTRL,usCtrl); // Write AD5421 Control Register
  //Note: If no data written within timeout, output will go to 3.2mA alarm cur.
}

// ****************************************************************************
// Write AD5421 register
// Returns AD5421 fault register read at the PREVIOUS(!) SPI transfer
// ****************************************************************************
unsigned short AD5421_Write(char cCommand, unsigned short usData)
{
  uiAD5421_Flt=(unsigned short)SPI_Write(pAD5421_SPI,(cCommand << 16) | usData); 
  return(uiAD5421_Flt);
} // AD5421_Write

// ****************************************************************************
// Read AD5421 register
// ****************************************************************************
unsigned short AD5421_Read(char cCommand)
{
  SPI_Write(pAD5421_SPI, cCommand << 16); // Write command for reading regist
  SPI_Read(pAD5421_SPI);          // Wait for transfer to finish + dummy read
  SPI_Write(pAD5421_SPI, 0);      // Write dummy command - to read back data
  return (SPI_Read(pAD5421_SPI)); // Wait for transfer to finish + read data
} // AD5421_Read

// ****************************************************************************
// Read AD5421 last fault (status) which was read during the last AD5421 write 
// ****************************************************************************
unsigned short AD5421_LastFlt(void)
{
  return(uiAD5421_Flt);
} // AD5421_LastFlt


// ****************************************************************************
// 4-20mA output slew rate control for HART analog rate of change compliance
// Industrial Automation System Applications (M.B.)
// 20 June 2011
// ****************************************************************************

unsigned short  usDacData=0;
unsigned short  usDacDiff; 
unsigned char   ucDacActive=0;
unsigned char   ucDacUp;

// ----------------------------------------------------------------------------
// Call the AD5421_Output function for updating the system 4-20mA output.
// Maximum update rate is limited by the number of steps in AD5421_SlewStepping
// and the time period of calling the AD5421_SlewStepping.
// For the the AD5421_Step called from 1ms timer and 19 steps,
// the AD5421_Output can be called, and the 4-20mA output updated, every 20ms.
// ----------------------------------------------------------------------------

void AD5421_Output(unsigned short DacNewData)   
{
  #define DacMaxSingleStep 0x090B         // 2nd last value from DacSlewTable 

  ucDacUp = (DacNewData >= usDacData)?1:0;// Find direction of transition 
  if (ucDacUp)                            // The output current is increasing
    usDacDiff = DacNewData - usDacData;

  else                                    // The output current is decreasing
    usDacDiff = usDacData - DacNewData;
  usDacData = DacNewData;                 // Copy and store data for next time

  if (usDacDiff > DacMaxSingleStep)       // Assess size of required change
    ucDacActive = 1;                      // Flag for changing output in steps
    // The 1ms timer interrupt must be enabled, and AD5421_Step called there...

  else                                    // Small change - direct ouput
    AD5421_Write(AD5421_WR_DAC,usDacData);// Write data to AD5421 DAC

} // AD5421_Output


// ----------------------------------------------------------------------------
// Call the the AD5421_Step in 1ms timer interrupt
// ----------------------------------------------------------------------------

volatile void AD5421_Step(void)
{
  #define DacNumberOfSteps 19      // Number of steps to shape the output slew

  // Note: This table needs to be calculated for the defined number of steps 
  const unsigned short DacSlewTable[DacNumberOfSteps] = {
    0xF6F3,0xECE9,0xE1CA,0xD586,0xC817,0xB986,0xA9EF,0x9985,0x8891,0x776D,
    0x6679,0x560F,0x4678,0x37E7,0x2A78,0x1E34,0x1315,0x090B,0x0000,
  };

  static short DacIndex = 0;        // Index 
  unsigned short DacStep;           // Data for DAC in this step

  if (ucDacActive == 1)             // Is the analog output stepping now?
  {
    // Calculate data to be sent to DAC in each step 
    // Using shift instead of division requires unsigned long integer format
    // That is the reason for code beeing different between 
    // increasing and decreasing the output current 

    if (ucDacUp)                    // The output current is increasing
      DacStep=usDacData-(((unsigned long)usDacDiff*DacSlewTable[DacIndex])>>16);
    else                            // The output current is increasing
      DacStep=usDacData+(((unsigned long)usDacDiff*DacSlewTable[DacIndex])>>16);

    AD5421_Write(AD5421_WR_DAC,DacStep);  // Write data to DAC

    if(++DacIndex == DacNumberOfSteps) // Check how many steps were performed
    {
      // All steps performed, transition is finished 
      ucDacActive = 0;              // Clear the flag
      DacIndex    = 0;              // Clear index (define as static!)
    }
  }  // if ucDacActive

} // AD5421_Step

