#include "HART.h"

void Chip_Initailize(){
	//---------- Disable Watchdog timer resets ----------
   WdtCfg(T3CON_PRE_DIV1, T3CON_IRQ_EN,T3CON_PD_DIS);
   //---------- Disable clock to unused peripherals ----------
   ClkDis(CLKDIS_DISSPI0CLK|CLKDIS_DISSPI1CLK|CLKDIS_DISI2CCLK|CLKDIS_DISPWMCLK|CLKDIS_DISDACCLK|CLKDIS_DIST1CLK);
   // Select CD0 for CPU clock - 2Mhz clock
   ClkCfg(CLK_CD2,CLK_HF,CLKSYSDIV_DIV2EN_EN,CLK_UDIV);
   // Select CD0 for CPU clock - 16Mhz clock
   //ClkCfg(CLK_CD0,CLK_HF,CLKSYSDIV_DIV2EN_DIS,CLK_UDIV);
   // Select CD0 for UART System clock
   ClkSel(CLK_CD7,CLK_CD7,CLK_CD0,CLK_CD7);
   
   //---------- I/O Port Configuration ----------
   DioCfg(pADI_GP0, 0x9000);   	// GPIO0 Port Function Setting 7:TX(10), 6:RX(01), 5~0:GPIO(0000000000)
   DioOen(pADI_GP0, 0xA4); 		// Sets GPIO direction, in or out. (7,5)
   DioPul(pADI_GP0, 0x00);		// Sets pull up resistors of port pins.(No pullup)
   
   DioCfg(pADI_GP1, 0x0000);   // GPIO1 Port Function Setting
   DioOen(pADI_GP1, 0xEB);
   DioPul(pADI_GP1, 0x00);

   DioCfg(pADI_GP2, 0x0000);   // GPIO2 Port Function Setting
   DioOen(pADI_GP2, 0xFF);
   DioPul(pADI_GP2, 0x00);

   DioSet(pADI_GP0, BIT5);
}

int main(void){
	Chip_Initailize();
	HART_Init();
	Soft_Timer_Init();
	Init_Param();
	
	
	while(TRUE){
		HART_polling();
	}
}

