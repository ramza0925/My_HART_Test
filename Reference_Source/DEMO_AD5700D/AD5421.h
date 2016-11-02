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

 File         : AD5421.h

 Hardware     : ADuCM360 + AD5421

 Description  : AD5421 interfaced to ADuCM360 SPI

 Status       : Functionality checked
******************************************************************************/

// AD5421 commands definition 
#define AD5421_WR_DAC     0x01
#define AD5421_WR_CTRL    0x02
#define AD5421_WR_OFFS    0x03
#define AD5421_WR_GAIN    0x04
#define AD5421_LD_DAC     0x05
#define AD5421_ALARM      0x06
#define AD5421_RESET      0x07
#define AD5421_INI_DAC    0x08
#define AD5421_RD_DAC     0x81
#define AD5421_RD_CTRL    0x82
#define AD5421_RD_OFFS    0x83
#define AD5421_RD_GAIN    0x84
#define AD5421_RD_FLT     0x85

// AD5421 Control Register Bits Definition 
#define AD5421_CTRL_WT_MSK        0xF000
#define AD5421_CTRL_WT_DIS        0x1000
#define AD5421_CTRL_WT_50MS       0x0000
#define AD5421_CTRL_WT_100MS      0x2000
#define AD5421_CTRL_WT_500MS      0x4000
#define AD5421_CTRL_WT_1000MS     0x6000
#define AD5421_CTRL_WT_2000MS     0x8000
#define AD5421_CTRL_WT_3000MS     0xA000
#define AD5421_CTRL_WT_4000MS     0xC000
#define AD5421_CTRL_WT_5000MS     0xE000

#define AD5421_CTRL_RDBACK_MSK    0x0800
#define AD5421_CTRL_RDBACK_DIS    0x0000
#define AD5421_CTRL_RDBACK_EN     0x0800

#define AD5421_CTRL_SPIALARM_MSK  0x0400
#define AD5421_CTRL_SPIALARM_DIS  0x0400
#define AD5421_CTRL_SPIALARM_EN   0x0000

#define AD5421_CTRL_MINCURR_MSK   0x0200
#define AD5421_CTRL_MINCURR_DIS   0x0000
#define AD5421_CTRL_MINCURR_EN    0x0200

#define AD5421_CTRL_ADCSEL_MSK    0x0100
#define AD5421_CTRL_ADCSEL_VLOOP  0x0000
#define AD5421_CTRL_ADCSEL_TEMP   0x0100

#define AD5421_CTRL_ADC_MSK       0x0080
#define AD5421_CTRL_ADC_DIS       0x0000
#define AD5421_CTRL_ADC_EN        0x0080

#define AD5421_CTRL_REFPD_MSK     0x0040
#define AD5421_CTRL_REFPD_DIS     0x0000
#define AD5421_CTRL_REFPD_EN      0x0040

#define AD5421_CTRL_FLTVLOOP_MSK  0x0020
#define AD5421_CTRL_FLTVLOOP_DIS  0x0000
#define AD5421_CTRL_FLTVLOOP_EN   0x0020

extern void AD5421_Init(unsigned short usCtrl /* ADI_SPI_TypeDef *pSPI */);
extern unsigned short AD5421_Write(char cCommand, unsigned short usData);
extern unsigned short AD5421_Read(char cCommand);
extern unsigned short AD5421_LastFlt(void);

extern void AD5421_Output(unsigned short DacNewData);
extern volatile void AD5421_Step(void);

// AD5421 control register value used specificaly for DEMO-AD5700D2Z:
#define AD5421_DEMO_VLOOP         0x6080 // WDT = 1sec, readback, ADC loop volt.
