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

 File         : AFE.c

 Hardware     : ADuCM360

 Description  : ADuCM360 AFE (Analog Front End) handling
                Code specific for DEMO-AD5700D2Z

                

 Status       : Functionality checked
******************************************************************************/

#include "ADuCM360.h"
#include "AFE.h"

volatile long lADC0Data; // ADC0 Data, updated in ADC0 interrupt
volatile long lADC1Data; // ADC1 Data, updated in ADC1 interrupt

volatile unsigned char ucADC0Status;  // ADC status, updated in ADC0 interrupt
volatile unsigned char ucADC1Status;  // ADC status, updated in ADC1 interrupt

volatile unsigned char ucADC0NewData; // New data flag, set in ADC0 interrupt
volatile unsigned char ucADC1NewData; // New data flag, set in ADC1 interrupt

// ****************************************************************************
// AFE INITIALISATION
// ****************************************************************************

void AFE_Init(void)
{
  pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DISADCCLK_MSK; // Enable ADC clock

  pADI_ADC0->ADCCFG = 0
    | ADCCFG_BOOST30_DIS    // Boost the Vbias current source ability 30x
    | ADCCFG_BOOST15_DIS    // Boost the Vbias current source ability 15x
  // Commented out - there is no constant defined for disabling vbias generator 
  //| ADCCFG_PINSEL_AIN7    // Switch vbias generator to pin 7
  //| ADCCFG_PINSEL_AIN11   // Switch vbias generator to pin 11
    | ADCCFG_GNDSWON_EN     // Ground switch
    | ADCCFG_GNDSWRESEN_DIS // 20k resistor in series with ground switch
    | ADCCFG_EXTBUF_OFF     // Reference buffers off. 
  ;                         // Really pity there is no more choice for ref.buf.

  pADI_ANA->REFCTRL = 0
    | REFCTRL_REFPD_DIS     // Int.Vref power down (DIS => Use the Vref :-)
  ;
  
  pADI_ANA->IEXCCON = 0
    | IEXCCON_PD_off        // Exc.current power down (PD_off => Use them :-)
    | IEXCCON_REFSEL_Int    // Exc.current ref.(Int = on-chip, Ext = IREF pin)
    | IEXCCON_IPSEL1_AIN6   // Exc.current 1 output pin
    | IEXCCON_IPSEL0_AIN4   // Exc.current 0 output pin
  ;
  
  pADI_ANA->IEXCDAT = 0
    | IEXCDAT_IDAT_100uA    // Exc.current value
    | IEXCDAT_IDAT0_DIS     // Extra 10uA current
  ;
  
} // AFE_Init



// ****************************************************************************
// ADC 0 INITIALISATION
// ****************************************************************************
// Measures primary sensor input voltage
// Input (+) buffered, AIN0, via RC filter from connector J5 pin 3
// Input (-) buffered, AIN1, via RC filter from connector J5 pin 4
//
// For demo with pressure sensor (#define PressureSensor in "AFE.h")
// Gain = 8
// Vref ext. (unbuffered), VREF+ connector J5 pin 2, VREF- connector J5 pin 5
//
// For demo shipped:
// Gain = 1
// Vref = AVdd (unbuffered)
//
// Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch



void ADC0_Init(void)
                              
{
//pADI_CLKCTL->CLKDIS &= ~ CLKDIS_DISADCCLK_MSK; // Enable ADC clock

  pADI_ADC0->CON = 0
    | ADCCON_ADCEN          // Enable ADC
    | ADCCON_ADCCODE_INT    // Output code signed (bipolar)
    | ADCCON_BUFPOWN_DIS    // ADC- buf power down dis => Enable the buffer :-)
    | ADCCON_BUFPOWP_DIS    // ADC+ buf pwr down disable => Enable the buffer :-)
    | ADCCON_BUFBYPP_DIS    // ADC+ buf bypass disable => Use the buffer :-)
    | ADCCON_BUFBYPN_DIS    // ADC- buf bypass disable => Use the buffer :-)

// Choose the correct Vref option based on the type of sensor and its connection
#ifdef PressureSensor
    | ADCCON_ADCREF_EXTREF  // Voltage reference
#else  
    | ADCCON_ADCREF_AVDDREF // Voltage reference
#endif  
    | ADCCON_ADCDIAG_DIAG_OFF // Diagnostic currents
    | ADCCON_ADCCP_AIN0     // ADC+ pin
    | ADCCON_ADCCN_AIN1     // ADC- pin
  ;  

  pADI_ADC0->FLT  = 0
    | ADCFLT_CHOP_ON        // System chopping
    | ADCFLT_RAVG2_OFF      // Running average-by-2
    | ADCFLT_SINC4EN_DIS    // Sync4 filter (disabled => Sync3 filter)
    |(ADCFLT_AF_MSK & 0)    // Averiging filter
    | ADCFLT_NOTCH2_EN      // Extra notch for 60Hz rejection at 50Hz data rate
    |(ADCFLT_SF_MSK & 0x7D) // Sync filter oversampling selection
                            // The UG-367 descriptionis really not sufficient
                            // Value 0x7D should give 50Hz data rate...
  ; 

  pADI_ADC0->MDE = 0
// Choose the correct gain option based on the type of sensor and its connection
#ifdef PressureSensor
    | ADCMDE_PGA_G8         // PGA gain
#else  
    | ADCMDE_PGA_G1         // PGA gain
#endif  
    | ADCMDE_ADCMOD2_MOD2OFF// Modulator extra gain of 2
    | ADCMDE_ADCMD_CONT     // Conversion mode
  ;

  pADI_ADC0->MSKI = 0
    | ADCMSKI_ATHEX_DIS     // ADC accumulator comparator threshold interrupt
    | ADCMSKI_THEX_DIS      // ADC comparator threshold interrupt
    | ADCMSKI_OVR_DIS       // ADC overrange interrupt
    | ADCMSKI_RDY_EN        // ADC ready interrupt
  ;

  NVIC_EnableIRQ(ADC0_IRQn); // Enable ADC interrupt

} // ADC0_Init



// ****************************************************************************
// ADC 1 RTD INITIALISATION
// ****************************************************************************
// Measures RTD (PT100) voltage
// Input(+) buffered, AIN3, via RC filter to RTD and connector J1 pin 3
// Input(-) buffered, AIN2, via RC filter to RTD and connector J1 pin 2
// Gain = 16
// Reference internal 1.2V
// Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch

void ADC1_Init_RTD(void)
                              
{
  pADI_ADC1->CON = 0
    | ADCCON_ADCEN          // Enable ADC
    | ADCCON_ADCCODE_INT    // Output code signed (bipolar)
    | ADCCON_BUFPOWN_DIS    // ADC- buf power down dis => Enable the buffer :-)
    | ADCCON_BUFPOWP_DIS    // ADC+ buf pwr down disable => Enable the buffer :-)
    | ADCCON_BUFBYPP_DIS    // ADC+ buf bypass disable => Use the buffer :-)
    | ADCCON_BUFBYPN_DIS    // ADC- buf bypass disable => Use the buffer :-)
    | ADCCON_ADCREF_INTREF  // Voltage reference
    | ADCCON_ADCDIAG_DIAG_OFF // Diagnostic currents
    | ADCCON_ADCCP_AIN3     // ADC+ pin
    | ADCCON_ADCCN_AIN2     // ADC- pin
  ;  

  pADI_ADC1->FLT  = 0
    | ADCFLT_CHOP_ON        // System chopping
    | ADCFLT_RAVG2_OFF      // Running average-by-2
    | ADCFLT_SINC4EN_DIS    // Sync4 filter (disabled => Sync3 filter)
    |(ADCFLT_AF_MSK & 0)    // Averiging filter
    | ADCFLT_NOTCH2_EN      // Extra notch for 60Hz rejection at 50Hz data rate
    |(ADCFLT_SF_MSK & 0x7D) // Sync filter oversampling selection
                            // The UG-367 descriptionis really not sufficient
                            // Value 0x7D should give 50Hz data rate...
  ;
  
  pADI_ADC1->MDE = 0
    | ADCMDE_PGA_G16        // PGA gain
    | ADCMDE_ADCMOD2_MOD2OFF// Modulator extra gain of 2
    | ADCMDE_ADCMD_CONT     // Conversion mode
  ;

  pADI_ADC1->MSKI = 0
    | ADCMSKI_ATHEX_DIS     // ADC accumulator comparator threshold interrupt
    | ADCMSKI_THEX_DIS      // ADC comparator threshold interrupt
    | ADCMSKI_OVR_DIS       // ADC overrange interrupt
    | ADCMSKI_RDY_EN        // ADC ready interrupt
  ;

  NVIC_EnableIRQ(ADC1_IRQn); // Enable ADC interrupt

} // ADC1_Init_RTD



// ****************************************************************************
// ADC 1 Rref INITIALISATION
// ****************************************************************************
// Measure Rref (R9 5.62kohm 10ppm) voltage
// Input(+) buffered, AIN7, via RC filter to R9 and connector J1 pin 1
// Input(-) buffer bypassed, AIN8, AGND (and R9 and connector J1 pin 4)
// Gain = 1
// Reference internal 1.2V
// Conversion continuous, chopping, filter sin3 50Hz with extra 60Hz notch
  
void ADC1_Init_Rref(void)
                              
{
  pADI_ADC1->CON = 0
    | ADCCON_ADCEN          // Enable ADC
    | ADCCON_ADCCODE_INT    // Output code signed (bipolar)
    | ADCCON_BUFPOWN_DIS    // ADC- buf power down dis => Enable the buffer :-)
    | ADCCON_BUFPOWP_DIS    // ADC+ buf pwr down disable => Enable the buffer :-)
    | ADCCON_BUFBYPP_DIS    // ADC+ buf bypass disable => Use the buffer :-)
    | ADCCON_BUFBYPN_EN     // ADC- buf bypass disable => Use the buffer :-)
    | ADCCON_ADCREF_INTREF  // Voltage reference
    | ADCCON_ADCDIAG_DIAG_OFF // Diagnostic currents
    | ADCCON_ADCCP_AIN7     // ADC+ pin
    | ADCCON_ADCCN_AIN8     // ADC- pin
  ;  

  pADI_ADC1->FLT  = 0
    | ADCFLT_CHOP_ON        // System chopping
    | ADCFLT_RAVG2_OFF      // Running average-by-2
    | ADCFLT_SINC4EN_DIS    // Sync4 filter (disabled => Sync3 filter)
    |(ADCFLT_AF_MSK & 0)    // Averiging filter
    | ADCFLT_NOTCH2_EN      // Extra notch for 60Hz rejection at 50Hz data rate
    |(ADCFLT_SF_MSK & 0x7D) // Sync filter oversampling selection
                            // The UG-367 descriptionis really not sufficient
                            // Value 0x7D should give 50Hz data rate...
  ;
  
  pADI_ADC1->MDE = 0
    | ADCMDE_PGA_G1         // PGA gain
    | ADCMDE_ADCMOD2_MOD2OFF// Modulator extra gain of 2
    | ADCMDE_ADCMD_CONT     // Conversion mode
  ;

  pADI_ADC1->MSKI = 0
    | ADCMSKI_ATHEX_DIS     // ADC accumulator comparator threshold interrupt
    | ADCMSKI_THEX_DIS      // ADC comparator threshold interrupt
    | ADCMSKI_OVR_DIS       // ADC overrange interrupt
    | ADCMSKI_RDY_EN        // ADC ready interrupt
  ;

  NVIC_EnableIRQ(ADC1_IRQn); // Enable ADC interrupt

} // ADC1_Init_Rref



// ****************************************************************************
// ADC 0 INTERRUPT
// ****************************************************************************

void ADC0_Int_Handler()
{
  ucADC0Status = pADI_ADC0->STA;  // Read ADC status
  lADC0Data = pADI_ADC0->DAT;     // Read ADC conversion result
  ucADC0NewData++;                // Set ADC new data flag
}

// ****************************************************************************
// ADC 1 INTERRUPT
// ****************************************************************************

void ADC1_Int_Handler()
{
  ucADC1Status = pADI_ADC1->STA;  // Read ADC status
  lADC1Data = pADI_ADC1->DAT;     // Read ADC conversion result
  ucADC1NewData++;                // Set ADC new data flag
}
