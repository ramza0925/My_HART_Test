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

 Date         : January 2013

 File         : FLASH.c

 Hardware     : ADuCM360

 Description  : FLASH functions - code specific for DEMO-AD5700D2Z

 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "FLASH.h"

// Fixed device ID address used for compatibility with other demo code examples
#define DEVICE_ID_ADDRESS 0xC864

// ****************************************************************************
// Write to FLASH
// ****************************************************************************

void WriteFlash(unsigned long *pulDeviceID)
{
  // Note that this is a very simple code.
  // It erases the whole page and re-programs only specific location
  // Therefore anything else that was on the specified page will be lost
  
  while (pADI_FEE->FEESTA & FEESTA_CMDBUSY);  // Wait for FLASH not busy 

  pADI_FEE->FEEKEY =  0xF456;                 // Flash user protection key 
  pADI_FEE->FEEKEY =  0xF123;

  pADI_FEE->FEEADR0L = DEVICE_ID_ADDRESS & 0xFF80; // Address lower part
  pADI_FEE->FEEADR0H = DEVICE_ID_ADDRESS >> 16;    // Address higher part

  pADI_FEE->FEECMD = FEECMD_CMD_ERASEPAGE;    // Page erase

  pADI_FEE->FEECON0 |= FEECON0_WREN;          // Enable FLASH write

  while (pADI_FEE->FEESTA & FEESTA_CMDBUSY);  // Wait for FLASH not busy 

  *(unsigned long*)DEVICE_ID_ADDRESS = *pulDeviceID;

  pADI_FEE->FEECON0 &= ~FEECON0_WREN_MSK;     // Disable FLASH write
}

// ****************************************************************************
// Read from FLASH
// ****************************************************************************

void ReadFlash(unsigned long *pulDeviceID)
{
  unsigned long ulTemp;
  ulTemp = *(unsigned long*)DEVICE_ID_ADDRESS; // Read HART ID from Flash
  // Check if the value has been already programmed to FLASH
  if (ulTemp != 0xFFFFFFFF)
    *pulDeviceID = ulTemp;
}
