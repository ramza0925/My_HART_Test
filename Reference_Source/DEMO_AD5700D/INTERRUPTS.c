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

void RESET_EXCPT_HNDLR ()
{
  extern int main(void);
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

void SysTick_Handler ()
{}
void NmiSR ()
{}
void FaultISR ()
{} 
void MemManage_Handler ()
{}
void BusFault_Handler ()
{}
void UsageFault_Handler ()
{} 
void SVC_Handler ()
{}
void DebugMon_Handler ()
{}
void PendSV_Handler ()
{} 
void WakeUp_Int_Handler ()
{}
void Ext_Int0_Handler ()
{}
void Ext_Int1_Handler ()
{}
void Ext_Int2_Handler ()
{}
void Ext_Int3_Handler ()
{}
void Ext_Int4_Handler ()
{}
void Ext_Int5_Handler ()
{}
void Ext_Int6_Handler ()
{}
void Ext_Int7_Handler ()
{}
void WDog_Tmr_Int_Handler ()
{}
void Test_OSC_Int_Handler ()
{}
/*
// Timer handling moved to Timer.c
void GP_Tmr0_Int_Handler ()
{}
void GP_Tmr1_Int_Handler ()
{}
*/
/*
// ADC handling moved to ADC.c
void ADC0_Int_Handler ()
{}
void ADC1_Int_Handler ()
{}
*/
void SINC2_Int_Handler()
{} 
void Flsh_Int_Handler ()
{}   
/*
// UART handling moved to UART.c
void UART_Int_Handler ()
{}
*/
/*
// SPI handling moved to SPI.c
void SPI0_Int_Handler ()
{}
void SPI1_Int_Handler ()
{}
*/
void I2C0_Slave_Int_Handler ()
{}
void I2C0_Master_Int_Handler ()
{}
void DMA_Err_Int_Handler ()
{}
void DMA_SPI1_TX_Int_Handler ()
{}
void DMA_SPI1_RX_Int_Handler ()
{}
void DMA_UART_TX_Int_Handler ()
{}
void DMA_I2C0_STX_Int_Handler ()
{}
void DMA_I2C0_SRX_Int_Handler ()
{}
void DMA_I2C0_MTX_Int_Handler ()
{}
void DMA_UART_RX_Int_Handler ()
{}
void DMA_I2C0_MRX_Int_Handler ()
{}
void DMA_ADC0_Int_Handler ()
{}
void DMA_ADC1_Int_Handler ()
{}
void DMA_DAC_Out_Int_Handler ()
{}
void DMA_SINC2_Int_Handler ()
{}
void PWM0_Int_Handler ()
{}
void PWM1_Int_Handler ()
{}
void PWM2_Int_Handler ()
{}
void PWM3_Int_Handler ()
{}
void PWMTRIP_Int_Handler ()
{}
