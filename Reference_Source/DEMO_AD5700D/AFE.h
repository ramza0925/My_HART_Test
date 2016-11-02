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
 Author       : Analog Devices Inc.
                I&I Segement System Applications (M.B.)

 Date         : February 2013

 File         : AFE.h

 Hardware     : ADuCM360

 Description  : ADuCM360 AFE (Analog Front End) handling
                Code specific for DEMO-AD5700D2Z

 Status       : Funtionality checked
******************************************************************************/

// #define PressureSensor

extern volatile long lADC0Data; // ADC0 Data, updated in ADC0 interrupt
extern volatile long lADC1Data; // ADC1 Data, updated in ADC1 interrupt

extern volatile unsigned char ucADC0Status; // ADC status, updated in ADC0 int.
extern volatile unsigned char ucADC1Status; // ADC status, updated in ADC1 int.

extern volatile unsigned char ucADC0NewData;// New data flag, set in ADC0 int.
extern volatile unsigned char ucADC1NewData;// New data flag, set in ADC1 int.


void AFE_Init(void);
void ADC0_Init(void);
void ADC1_Init_RTD(void);
void ADC1_Init_Rref(void);
