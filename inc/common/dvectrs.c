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

    Module       : dvectrs.c
    Description  : Default exception vectors.
                   These are referenced in the supplied start-up code.
                   An application supplied version (where provided) will be
                   used in preference to these ones as these are defined
                   as weak functions.
    Date         : 10 June 2010
    Version      : v1.00
    Changelog    : v1.00 Initial
*/
#include <cportabl.h>
//*********************************************************************
//
// Default fault handlers.
//
//*****************************************************************************
WEAK_PROTO(void ResetISR                 (void)) ATTRIBUTE_INTERRUPT;
WEAK_PROTO(void SysTick_Handler          (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void NmiSR                    (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void FaultISR                 (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void MemManage_Handler        (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void BusFault_Handler         (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void UsageFault_Handler       (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void SVC_Handler              (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void DebugMon_Handler         (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void PendSV_Handler           (void)) ATTRIBUTE_INTERRUPT ;


WEAK_PROTO(void WakeUp_Int_Handler        (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int0_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int1_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int2_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int3_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int4_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int5_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int6_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Ext_Int7_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void WDog_Tmr_Int_Handler      (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void Test_OSC_Int_Handler      (void)) ATTRIBUTE_INTERRUPT ;

WEAK_PROTO(void GP_Tmr0_Int_Handler       (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void GP_Tmr1_Int_Handler       (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void ADC0_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void ADC1_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void SINC2_Int_Handler         (void)) ATTRIBUTE_INTERRUPT ; 

WEAK_PROTO(void Flsh_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void UART_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void SPI0_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void SPI1_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void I2C0_Slave_Int_Handler    (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void I2C0_Master_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ; 

WEAK_PROTO(void DMA_Err_Int_Handler       (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_SPI1_TX_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_SPI1_RX_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_UART_TX_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_UART_RX_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_I2C0_STX_Int_Handler  (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_I2C0_SRX_Int_Handler  (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_I2C0_MTX_Int_Handler  (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_I2C0_MRX_Int_Handler  (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void DMA_DAC_Out_Int_Handler   (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void DMA_ADC0_Int_Handler      (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void DMA_ADC1_Int_Handler      (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void DMA_SINC2_Int_Handler     (void)) ATTRIBUTE_INTERRUPT ;
WEAK_PROTO(void PWMTRIP_Int_Handler       (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void PWM0_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void PWM1_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void PWM2_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 
WEAK_PROTO(void PWM3_Int_Handler          (void)) ATTRIBUTE_INTERRUPT ; 


WEAK_PROTO(void UnUsed_Handler            (void)) ATTRIBUTE_INTERRUPT  ;

#ifdef __ARMCC_VERSION
// We want a warning if semi-hosting libraries are used.
#pragma import(__use_no_semihosting_swi)
#endif

//*****************************************************************************
//
// Application Entry point
//
//*****************************************************************************
extern int main(void);


//*****************************************************************************
// Symbols defined by the GNU linker.
//*****************************************************************************
#ifdef __GNUC__
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss ;
extern unsigned long _ebss;
#endif


//*****************************************************************************
// Function    : ResetISR
// Description : Reset event handler
//*****************************************************************************
WEAK_FUNC(void ResetISR(void)){

#ifdef __GNUC__
    unsigned long *pulSrc, *pulDest;

    // Copy initialised data from flash into RAM
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    // Clear the bss segment
    for(pulDest = &_bss; pulDest < &_ebss; )
    {
        *pulDest++ = 0;
    }
#endif
    // Call application main.
    main();

    // Stick here if main returns
    while(1);
}

//*****************************************************************************
// Function    : NmiSR
// Description : NMI Handler
//*****************************************************************************
WEAK_FUNC(void NmiSR(void)) {
    while(1)
    {
    }
}
//*****************************************************************************
// Function    : FaultISR
// Description : Hard fault handler
//*****************************************************************************
WEAK_FUNC(void FaultISR(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}

//*****************************************************************************
// Function    : MemManage_Handler
// Description : Mem Manage Handler
//*****************************************************************************
WEAK_FUNC(void MemManage_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : BusFault_Handler
// Description : BusFault Handler
//*****************************************************************************
WEAK_FUNC(void BusFault_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : UsageFault_Handler
// Description : Usage Fault Handler
//*****************************************************************************
WEAK_FUNC(void UsageFault_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : SVC_Handler
// Description : SVC handler
//*****************************************************************************
WEAK_FUNC(void SVC_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : DebugMon_Handler
// Description : DebugMon handler
//*****************************************************************************
WEAK_FUNC(void DebugMon_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : PendSV_Handler
// Description : PendSV handler
//*****************************************************************************
WEAK_FUNC(void PendSV_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
//*****************************************************************************
// Function    : SysTick_Handler
// Description : SysTick interrupt handler
//*****************************************************************************
WEAK_FUNC(void SysTick_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : WakeUp_Int_Handler        (void)
// Decription  :
// *****************************************************************************
WEAK_FUNC(void WakeUp_Int_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}

//******************************************************************************
// // Function    : Ext_Int0_Handler (void)
// Description  :
// *****************************************************************************
WEAK_FUNC(void Ext_Int0_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}

//******************************************************************************
// Function    : Ext_Int1_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int1_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int2_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int2_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int3_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int3_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int4_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int4_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int5_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int5_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int6_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int6_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Ext_Int7_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Ext_Int7_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : WDog_Tmr_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void WDog_Tmr_Int_Handler      (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : LF_Eng_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void LF_Eng_Int_Handler        (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : GP_Tmr0_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void GP_Tmr0_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : GP_Tmr1_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void GP_Tmr1_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : ADC0_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void ADC0_Int_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Flsh_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Flsh_Int_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : UART_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void UART_Int_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : SPI0_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void SPI0_Int_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : SPI1_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void SPI1_Int_Handler (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : I2C0_Slave_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void I2C0_Slave_Int_Handler    (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : I2C0_Master_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void I2C0_Master_Int_Handler   (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}




//******************************************************************************
// Function    : DMA_Err_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_Err_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_SPI1_TX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_SPI1_TX_Int_Handler   (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_SPI1_RX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_SPI1_RX_Int_Handler   (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_UART_TX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_UART_TX_Int_Handler   (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_UART_RX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_UART_RX_Int_Handler   (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_I2C0_STX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_I2C0_STX_Int_Handler  (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_I2C0_SRX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_I2C0_SRX_Int_Handler  (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_I2C0_MTX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_I2C0_MTX_Int_Handler  (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_I2C0_MRX_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_I2C0_MRX_Int_Handler  (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}



//******************************************************************************
// Function    : DMA_ADC_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_ADC_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}

//******************************************************************************
// Function    : UnUsed_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void UnUsed_Handler(void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : Test_OSC_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void Test_OSC_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
    
//******************************************************************************
// Function    : ADC1_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void ADC1_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
   
//******************************************************************************
// Function    : SINC2_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void SINC2_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
 
//******************************************************************************
// Function    : PWMTRIP_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void PWMTRIP_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
    
//******************************************************************************
// Function    : PWM0_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void PWM0_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
    
//******************************************************************************
// Function    : PWM1_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void PWM1_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
    
//******************************************************************************
// Function    : PWM2_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void PWM2_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
    
//******************************************************************************
// Function    : PWM3_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void PWM3_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}


//******************************************************************************
// Function    : DMA_DAC_Out_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_DAC_Out_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
   
//******************************************************************************
// Function    : DMA_ADC0_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_ADC0_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
   
//******************************************************************************
// Function    : DMA_ADC1_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_ADC1_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}
  
//******************************************************************************
// Function    : DMA_SINC2_Int_Handler
// Description :
// *****************************************************************************
WEAK_FUNC(void DMA_SINC2_Int_Handler       (void)){
   // Infinite loop. An implementation appropriate to the system
   // should be provided by the application.
   while(1)
      {
      }
}

