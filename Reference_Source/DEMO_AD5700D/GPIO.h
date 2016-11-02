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

 File         : GPIO.h

 Hardware     : ADuCM360

 Description  : General purpose (digital) input/output related functions
                Code specific for DEMO-AD5700D2Z


 Status       : Funtionality checked
******************************************************************************/

extern void GPIO_Init(void);

extern void GPIO_LedOn(void);
extern void GPIO_LedOff(void);

extern unsigned char GPIO_TestState(void);
extern unsigned char GPIO_DownloadState(void);

extern void GPIO_RtsOn(void);
extern void GPIO_RtsOff(void);

extern unsigned char GPIO_CdState(void);

extern void GPIO_UartMuxToHart(void);
extern void GPIO_UartMuxToJ3(void);
