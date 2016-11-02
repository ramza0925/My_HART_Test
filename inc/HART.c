#include "HART.h"

#define MANUFACTURER_ID   		0x60DE
#define DISTRIBUTER_ID          0x60DE						//SAMHOI
#define DEVICE_TYPE0    		0xE3
#define DEVICE_TYPE1    		0x7D
#define UNIQUE_DEVICE_ID0 		0x70						//HART Revision
#define UNIQUE_DEVICE_ID1 		0x00						//Device Revision
#define UNIQUE_DEVICE_ID2 		0x00						//Hardware Revision, Software Revision

#define HART_REVISION           0x07						//HART Revision
#define DEVICE_REVISION         0x00						//Device Revision
#define SOFT_REVISION           0x00						//Software Revision
#define HARD_REVISION           0x00						//Hardware Revision

Tx_Frame_Type g_Tx;											//Transmitt Message Frame
Rx_Frame_Type g_Rx;											//Transmitt Message Frame
Rcv_Msg_Type g_RcvMsgType;									//Receive Msg Type
Xmt_Msg_Type g_XmtMsgType;									//Transmitt Msg Type

HART_State g_HartState = HART_WAIT;							//HART Status variable
RSM_State g_RcvState = RCV_WAIT_IDLE;						//Receive Status variable
TSM_State g_XmtState = XMT_INIT;							//Transmitt Status variable
Soft_Timer g_tmr[TMR_CNT];									//System Tick Timer

void (*command)(u8 *data);

struct parameter{
	float PV;
	float SV;
	float TV;
	float QV;
	float LoopCurrent; //unit : mA
	float PercnetOfRange; // unit : %
	float FixedCurrent;   // if = 0 , current value = actual current; else current value = fixed current; unit : mA
	float PVZero;
	float ExtMeasuredZeroCurr; //externally measured zero currrent
	float ActMeasuredZeroCurr; //actual measured zero current
	float ExtMeasuredGainCurr; //externallu measured gain current
	float ActMeasuredGainCurr; //actual measured gain current
	u8 PVUnit;
	u8 SVUnit;
	u8 TVUnit;
	u8 QVUnit;
	u8 PVCode;
	u8 SVCode;
	u8 TVCode;
	u8 QVCode;
	u8 ULRangeUnit;
	u8 PollingAddr;
	u8 LoopCurrentMode;
	u8 ResposePreambleNum;
	u8 BurstModeCmdNum;
	u8 BurstModeCode;
	u8 ConfigChangeFlag; //configuration change flag
	u8 ExtendedDeviceStatus;
	u8 DeviceOperatingMode;
	u8 Standardized_status_0;
	u8 Standardized_status_1;
	u8 AnalogChannelSaturation;
	u8 Standardized_status_2;
	u8 Standardized_status_3;
	u8 AnalogChannelFixed;
	u8 Msg[24];  //packed
	u8 Tag[6];   //packed
	u8 LongTag[32]; //Latin-1
	u8 Dscp[12];  //descriptor , packed
	u8 Date[3];  //date : day/month/year
	u8 Tsn[3];  //transducer serial number
	u8 Dss[6];  //device specific status
	float TransducerUpper;  //PV transducer unit code is the same as PV_UNIT
	float TransducerLower;
	float PVMinSpan;
	float PVUpperRange;
	float PVlowerRange;
	float PVDampingTime;
	enum alarm Alarm;
	enum transfer_func TransferFunc;
	enum protect ProtectStatus;
	enum analog_channel AnalogChlFlg;
	u8 Fan[3]; //final assembly number
}_para;

static volatile u8 *pXmtBufferCur;
static volatile u16 s_XmtBufferCnt;							//Received Message Length
static volatile u8 s_XmtPreambleNum;						//Received Preamble Count
static volatile u16 s_RcvBufferPos;

volatile u16 RcvFrameCnt = 0; 		// receive frame count
//volatile u8 FramePrcCompFlg = 0; 	// frame process completed flag
//volatile u8 TsmFrameCompFlg = 0; 	// transmit frame completed flag

u8 g_Burst = FALSE;
//u8 g_Bt = 0;
u8 g_Host = PRIMARY_MASTER;

Long_Addr_Type long_addr = {DEVICE_TYPE0,DEVICE_TYPE1,UNIQUE_DEVICE_ID0,UNIQUE_DEVICE_ID1,UNIQUE_DEVICE_ID2};

u16 HrtByteCnt = 0;
u8 HrtResposeCode = 0;
u8 HrtDeviceStatus = 0;
//u8 HrtResposePos = 0;

u16 configurationChangeCounter = 0;											//Configuration Change Counter(Must save/load in H/W Memory)
u8 fieldDeviceStatus = 0;													//Field Device Status(Must saved/load in H/W Memory)

u16 dly_cnt = 0;
u16 glcd_x, glcd_y;
char LCD[20];

void delay_us (u32 length){ 
  length /= 3;
  while (length-- > 0);
}

void delay_ms (u32 length){
   dly_cnt = 0;
   
   while (length > dly_cnt);
}

#include "..\inc\G_LCD\SPLC501C.c"
#include "..\inc\G_LCD\graphic.c"

/* DVs */
void  Set_Pv(float pv) {_para.PV = pv;}
float Get_Pv(void)     {return _para.PV;}
void  Set_Sv(float sv) { _para.SV = sv; 	}
float Get_Sv(void)     { return _para.SV; }
void  Set_Tv(float tv) { _para.TV = tv; 	}
float Get_Tv(void)     { return _para.TV; }
void  Set_Qv(float qv) { _para.QV = qv; 	}
float Get_Qv(void)     { return _para.QV; }
/* DV unit */
void Set_Pv_Unit(u8 pv_unit) { _para.PVUnit = pv_unit; }
u8 Get_Pv_Unit(void) { return _para.PVUnit; }
void Set_Sv_Unit(unsigned char sv_unit) { _para.SVUnit = sv_unit; }
u8 Get_Sv_Unit(void) { return _para.SVUnit; }
void Set_Tv_Unit(unsigned char tv_unit) { _para.TVUnit = tv_unit; }
u8 Get_Tv_Unit(void) { return _para.TVUnit; }
void Set_Qv_Unit(unsigned char qv_unit) { _para.QVUnit = qv_unit; }
u8 Get_Qv_Unit(void) { return _para.QVUnit; }
/* DV code */
void Set_Pv_Code(u8 pv_code) { _para.PVCode = pv_code; }
u8 Get_Pv_Code(void) { return _para.PVCode; }
void Set_Sv_Code(u8 sv_code) { _para.SVCode = sv_code; }
u8 Get_Sv_Code(void) { return _para.SVCode; }
void Set_Tv_Code(u8 tv_code) { _para.TVCode = tv_code; }
u8 Get_Tv_Code(void) { return _para.TVCode; }
void Set_Qv_Code(u8 qv_code) { _para.QVCode = qv_code; }
u8 Get_Qv_Code(void) { return _para.QVCode; }
/* loop current and percent of range */
void  Set_Loop_Current(float current) { _para.LoopCurrent = current; }
float Get_Loop_Current(void) { return _para.LoopCurrent; }
void  Set_Percent_Of_Range(float percent_of_range) { _para.LoopCurrent = percent_of_range; }
float Get_Percent_Of_Range(void) { return _para.PercnetOfRange; }
//Burst Mode Seet
void Set_Burst_Mode(u8 bt_code) {_para.BurstModeCode = bt_code;}
u8 Get_Burst_Mode(void) {return _para.BurstModeCode;}
/* burst mode command number */
void Set_Burst_Mode_Cmd_Num(u8 bt_cmd) { _para.BurstModeCmdNum = bt_cmd; }
unsigned char Get_Burst_Mode_Cmd_Num(void) { return _para.BurstModeCmdNum; }
//Response Preamble Number
void Set_Response_Preamble_Num(u8 rsp_preamble_num) {_para.ResposePreambleNum = rsp_preamble_num;}
u8 Get_Response_Preamble_Num(void) {return _para.ResposePreambleNum;}
/* upper/lower range unit */
void Set_Ul_Range_Unit(u8 ul_range_unit) { _para.ULRangeUnit = ul_range_unit; }
u8 Get_Ul_Range_Unit(void) { return _para.ULRangeUnit; }
//Polling Address Setting
void Set_Polling_Addr(u8 polling_addr) {_para.PollingAddr = polling_addr;}
u8 Get_Polling_Addr(void) {return _para.PollingAddr;}
void Set_Loop_Current_Mode(u8 current_mode) { _para.LoopCurrentMode = current_mode; }
u8 Get_Loop_Current_Mode(void) { return _para.LoopCurrentMode; }
//Extended Device Status Setting 
void Set_Extended_Device_Status(u8 status) {_para.ExtendedDeviceStatus = status;}
u8 Get_Extended_Device_Status(void) {return _para.ExtendedDeviceStatus;}
//Standardized Status Setting
void Set_Std_Status_0(u8 status_0) {_para.Standardized_status_0 = status_0;}
u8 Get_Std_Status_0(void) {return _para.Standardized_status_0;}
void Set_Std_Status_1(u8 status_1){_para.Standardized_status_1 = status_1;}
u8 Get_Std_Status_1(void){return _para.Standardized_status_1;}
void Set_Std_Status_2(u8 status_2){_para.Standardized_status_2 = status_2;}
u8 Get_Std_Status_2(void){return _para.Standardized_status_2;}
void Set_Std_Status_3(u8 status_3){_para.Standardized_status_3 = status_3;}
u8 Get_Std_Status_3(void){return _para.Standardized_status_2;}
void Set_Analog_Channel_Saturation(u8 Acs){_para.AnalogChannelSaturation = Acs;}
u8 Get_Analog_Channel_Saturation(void){return _para.AnalogChannelSaturation;}
void Set_Analog_Channel_Fixed(u8 Acf){_para.AnalogChannelFixed = Acf;}
u8 Get_Analog_Channel_Fixed(void){return _para.AnalogChannelFixed;}
void Set_Config_Change_Flag(u8 cfg_change_flag) { _para.ConfigChangeFlag = cfg_change_flag; }
u8 Get_Config_Change_Flag(void) { return _para.ConfigChangeFlag; }
void Set_Pv_Zero(float pv_zero) { _para.PVZero = pv_zero; }
float Get_Pv_Zero(void) { return _para.PVZero; }
void Set_Message(u8 *msg){
	u8 i = 0;
	for(i = 0;i < 24;i++){
		_para.Msg[i] = *(msg+i);
	}
}
u8 *Get_Message(void) { return _para.Msg; }

void Set_Tag(u8 *tag){
	u8 i = 0;
	for(i = 0;i < 6;i++){
		_para.Tag[i] = *(tag+i);
	}
}
u8 *Get_Tag(void) { return _para.Tag; }

void Set_Descriptor(u8 *dscp){
	u8 i = 0;
	for(i = 0;i < 12;i++){
		_para.Dscp[i] = *(dscp+i);
	}
}
u8 *Get_Descriptor(void) { return _para.Dscp; }

void Set_Date(u8 *date){
	u8 i = 0;
	for(i = 0;i < 3;i++){
		_para.Date[i] = *(date+i);
	}
}
u8 *Get_Date(void) { return _para.Date; }

void Set_LongTag(u8 *longtag){
	u8 i = 0;
	for(i = 0;i < 32;i++){
		_para.LongTag[i] = *(longtag+i);
	}
}
u8 *Get_LongTag(void) { return _para.LongTag; }

/* PV transducer information */
void Set_Transducer_Serial_Num(u8 *tsn){
	u8 i = 0;
	for(i = 0;i < 3;i++){
		_para.Tsn[i] = *(tsn+i);
	}
}
u8 *Get_Transducer_Serial_Num(void) { return _para.Tsn; }
void Set_Transducer_Upper(float tsd_upper) { _para.TransducerUpper = tsd_upper; }
float Get_Transducer_Upper(void) { return _para.TransducerUpper; }
void Set_Transducer_Lower(float tsd_lower) { _para.TransducerLower = tsd_lower; }
float Get_Transducer_Lower(void) { return _para.TransducerLower; }
void Set_Pv_Upper_Range(float upper_range) { _para.PVUpperRange = upper_range; }
float Get_Pv_Upper_Range(void) { return _para.PVUpperRange; }
void Set_Pv_Lower_Range(float lower_range) { _para.PVlowerRange = lower_range; }
float Get_Pv_Lower_Range(void) { return _para.PVlowerRange; }
void Set_Pv_Damping_Time(float damping_time) { _para.PVDampingTime = damping_time; }
float Get_Pv_Damping_Time(void) { return _para.PVDampingTime; }
void Set_Pv_Min_Span(float min_span) { _para.PVMinSpan = min_span; }
float Get_Pv_Min_Span(void) { return _para.PVMinSpan; }
void Set_Alarm_Sw(enum alarm alarm) { _para.Alarm = alarm; }
enum alarm Get_Alarm_Sw(void) { return _para.Alarm; }
void Set_Transfer_Func(enum transfer_func tsf_func) { _para.TransferFunc = tsf_func; }
enum transfer_func Get_Transfer_Func(void) { return _para.TransferFunc; }
void Set_Protect(enum protect protect_status) { _para.ProtectStatus = protect_status; }
enum protect Get_Protect(void) { return _para.ProtectStatus; }
void Set_Analog_Channel(enum analog_channel analog_chl) { _para.AnalogChlFlg = analog_chl; }
enum analog_channel Get_Analog_Channel(void) { return _para.AnalogChlFlg; }
void Set_Final_Assembly_Num(u8 *fan){
	u8 i = 0;
	for(i = 0;i < 3;i++){
		_para.Fan[i] = *(fan+i);
	}
}
u8 *Get_Final_Assembly_Num(void) { return _para.Fan; }
void Set_Fixed_Current(float fixed_current) { _para.FixedCurrent = fixed_current; }
float Get_Fixed_Current(void) { return _para.FixedCurrent; }
void Set_Act_Zero_Current(float act_zero_curr) { _para.ActMeasuredZeroCurr = act_zero_curr; }
float Get_Act_Zero_Current(void) { return _para.ActMeasuredZeroCurr; }
void Set_Act_Gain_Current(float act_gain_current) { _para.ActMeasuredGainCurr = act_gain_current; }
float Get_Act_Gain_Current(void) { return _para.ActMeasuredGainCurr; }
void Set_Device_Specific_Status(u8 *dss){
	u8 i;
	
	for(i = 0;i < 6;i++){
		_para.Dss[i] = *(dss+i);
	}
}
u8 *Get_Device_Specific_Status(void) { return _para.Dss; } 
void Set_Device_Operating_Mode(u8 mode) { _para.DeviceOperatingMode = mode; }
u8 Get_Device_Operating_Mode(void) { return _para.DeviceOperatingMode; }
u8 Get_Host_Type(void){return (g_Rx.data_buf[1] & 0x80);}


void Soft_Timer_Init(void){
	u8 i;
	for(i=0; i<TMR_CNT; i++){
		g_tmr[i].cnt = 0;
		g_tmr[i].flg = 0;
	}
}

void TIMER0_Init(void){
	//--------- Timer 0 Setup 1 msec -------------
   GptLd(pADI_TM0, 32);	//2Mhz PCLK
   //GptLd(pADI_TM0, 0x3E8);	//16Mhz PCLK
   GptCfg(pADI_TM0,TCON_CLK_LFOSC,TCON_PRE_DIV1,TCON_MOD_PERIODIC|TCON_RLD_DIS|TCON_ENABLE_EN|TCON_UP_DIS);
   
}

//HART Initialize
void HART_Init(void){
	TIMER0_Init();
	UART_Init();												//UART Initialize

	NVIC_EnableIRQ(TIMER0_IRQn);  								// TMR0 Interrupt Enable !!!
	NVIC_EnableIRQ(UART_IRQn);
	
	GLCD_Initialize();
	GLCD_ClearScreen();
	
	UART_Enable(TRUE, FALSE);									//Rx Enable, Tx DIsable
	Set_Burst_Mode(FALSE);										//No Burst Mode
	Set_Response_Preamble_Num(PREAMBLE_DEFAULT_NUM);			//Initiate Preamble
	Set_Polling_Addr(0);										//Initiate Polling Address
#ifdef DEBUG
	GLCD_GoTo(10, 1);
    GLCD_WriteString5x7("HART_Init");
#endif	
}

//HART Process Polling
void HART_polling(void){
	if(RcvFrameCnt){											//Received message 
		RcvFrameCnt--;											//Received Frame Count to Zero
		if(g_HartState == HART_WAIT){
			HART_Wait();
#ifdef DEBUG
		GLCD_GoTo(10, 1);
    	GLCD_WriteString5x7("HART_WAIT");
#endif				
		}
		if(g_HartState == HART_PROCESS){						//Transmitt Message
			HART_Process();
			s_XmtPreambleNum = Get_Response_Preamble_Num();
			s_XmtBufferCnt = g_Tx.address_size + 3 + g_Tx.byte_count +1;
			pXmtBufferCur = g_Tx.data_buf;
			DioClr(pADI_GP0, BIT5);
			UART_Enable(FALSE,TRUE);
#ifdef DEBUG
		GLCD_GoTo(10, 1);
    	GLCD_WriteString5x7("HART_PROCESS");
#endif			
		}
	}
}

//It Received Hart Msg in Wait Mode
static void HART_Wait(void){
	Rcv_Msg_Type rcv_msg_t;
	u8 BurstMode;

	rcv_msg_t = g_RcvMsgType;
	BurstMode = Get_Burst_Mode();
	//Burst Mode
	if(BurstMode){
		g_Burst = TRUE;
		Set_Delay_Time(BT_TIMER,0);
		g_Host = PRIMARY_MASTER;
	}
	//No Burst Mode
	else{
		g_Burst = FALSE;
	}
	//Burst Mode Start.
	if(g_Burst && (Is_Timeout_Id(BT_TIMER))){
		g_XmtMsgType = XMT_BACK;
		g_Host = (~g_Host)&0x01;					//Why?
		Set_Delay_Time(BT_TIMER,HART_RT2);
	}
	//No Burst Mode
	else{
		switch(rcv_msg_t){
			case RCV_ERR:							//Receive Msg Error
				break;				
			case RCV_ACK:							//Receive Msg ACK
				Set_Delay_Time(BT_TIMER,0);
				Set_Delay_Time(SLAVE_TIMER, HART_STO);
				break;				
			case RCV_STX:							//Reccive Msg STX
				if(Is_Addr_Match()){				//Address Check
					Set_Delay_Time(BT_TIMER,0);
					Set_Delay_Time(SLAVE_TIMER, HART_STO);
					g_HartState = HART_PROCESS;		//HART Process Start
				}
				else{
					Set_Delay_Time(BT_TIMER, HART_PRI_RT1);
				}
				break;
			default:
				break;
		}
	}
}


//HART Command Response Function
static void HART_Process(void){
	u8 cnt, is_burst_mode;
	
	is_burst_mode = Get_Burst_Mode();
	cnt = g_Rx.address_size + 3 + g_Rx.data_buf[g_Rx.address_size+2];
	if(Is_Timeout_Id(SLAVE_TIMER)){		//slave TimeOut
		g_HartState = HART_WAIT;
	}
	else{
		if(Longitudinal_Parity(g_Rx.data_buf,cnt) == g_Rx.data_buf[cnt]){	//No Comm Error
			g_XmtMsgType = XMT_ACK;
		}
		else{		//Comm Error
			g_XmtMsgType = XMT_COMM_ERR;			
		}
	}
	Frame_Cmd_Data();		
	cnt = g_Tx.address_size + 3 + g_Tx.byte_count;
	g_Tx.data_buf[cnt] = Longitudinal_Parity(g_Tx.data_buf, cnt);
	if(is_burst_mode){
		g_HartState = HART_WAIT;
	}
	else{
		g_HartState = HART_WAIT;
	}

}

void Enter_Critical_Section(void){
  __disable_irq();
}

void Exit_Critical_Section(void){
  __enable_irq();
}

u16 Cmd_Function(u8 cmd, u8 *data){
	u8 burst_cmd, is_burst_mode;
	
	is_burst_mode = Get_Burst_Mode();
	HrtByteCnt = 0;
	
	if(is_burst_mode){
		burst_cmd = Get_Burst_Mode_Cmd_Num();
		
		if(cmd == 109){ //burst mode control : enter or exit burst mode.
			command = C109_BurstModeControl;
		}
		switch(burst_cmd){
			case 1:	
				*(data-2) = 1;
				command = C1_RdPV;	  
				break;
			case 2:	
				*(data-2) = 2;
				command = C2_RdLoopCurrPerOfRange;		
				break;
			case 3:	
				*(data-2) = 3;
				command = C3_RdDVLoopCurr;		
				break;
			default: 
				*(data-2) = 1;
				command = C1_RdPV;		
				break;
		}
	}
	else{
		switch(cmd){
			case 0: command = C0_RdUniqueId;				break;
			case 1:	command = C1_RdPV;	  					break;
			case 2:	command = C2_RdLoopCurrPerOfRange;		break;
			case 3:	command = C3_RdDVLoopCurr;				break;
			case 6:	command = C6_WrPollingAddr;				break;
			case 7:	command = C7_RdLoopConfiguration; 		break;
			case 8:	command = C8_RdDVClass;					break;
			case 9:	command = C9_RdStatusDV;				break;
			case 11: command = C0_RdUniqueId;				break;
			case 12: command = C12_RdMessage;				break;
			case 13: command = C13_RdTagDescriptorDate;		break;
			case 14: command = C14_RdPVTransducerInfo;		break;
			case 15: command = C15_RdDeviceInfo;			break;
			case 16: command = C16_RdFinalAssemblyNum;		break;
			case 17: command = C17_WrMessage;				break;
			case 18: command = C18_WrTagDescriptorDate;		break;
			case 19: command = C19_WrFinalAssemblyNum;		break;
			case 20: command = C20_RdLongTag;				break;
			case 21: command = C0_RdUniqueId;				break;
			case 22: command = C22_WrLongTag;				break;
			case 33: command = C33_RdDeviceVariable;    	break;
			case 34: command = C34_WrPVDamping;				break;
			case 35: command = C35_WrPVRange;				break;
			case 36: command = C36_SetPVUpperRange;			break;
			case 37: command = C37_SetPVLowerRange;			break;
			case 38: command = C38_ResetCfgChangeFlag;		break;
			case 40: command = C40_EnterOrExitFixedCurrent;	break;
			case 41: command = C41_PerformSelfTest;			break;
			case 42: command = C42_PerformDeviceReset;		break;
			case 43: command = C43_PVZero;					break;
			case 44: command = C44_WrPVUnit;				break;
			case 45: command = C45_TrimLoopCurrentZero;		break;
			case 46: command = C46_TrimLoopCurrentGain;		break;
			case 47: command = C47_WrPVTransferFunction;	break;
			case 48: command = C48_RdAdditionalDeviceStatus;break;
			case 49: command = C49_WrPVTransducerSerialNum;	break;
			case 50: command = C50_RdDVAssignments;			break;
			case 51: command = C51_WrDVAssignments;			break;
			case 59: command = C59_WrNumOfResposePreambles;	break;
			case 108: command = C108_WrBurstModeCmdNum;		break;
			case 109: command = C109_BurstModeControl;		break;
			case 200: command = C200_WrTest;				break;
			default:										break;
		}
	}
	Enter_Critical_Section( );
	command(data);
	Exit_Critical_Section( );
	return HrtByteCnt;
}

void Frame_Cmd_Data(void){
	u8 cmd;
	u8 *data;
	
	if(g_Rx.address_size == LONG_ADDR_SIZE){
		cmd = g_Rx.data_buf[HART_LONGF_CMD_OFF];
		data = &g_Tx.data_buf[HART_LONGF_RSPCODE1_OFF];
		
		g_Tx.data_buf[HART_LONGF_CMD_OFF] = cmd;
		g_Tx.byte_count = Cmd_Function(cmd,data);
		g_Tx.data_buf[HART_LONGF_LEN_OFF] = g_Tx.byte_count;
	}
	else{
		cmd = g_Rx.data_buf[HART_SHORTF_CMD_OFF];
		data = &g_Tx.data_buf[HART_SHORTF_RSPCODE1_OFF];
		
		g_Tx.data_buf[HART_SHORTF_CMD_OFF] = cmd;
		g_Tx.byte_count = Cmd_Function(cmd,data);		
		g_Tx.data_buf[HART_SHORTF_LEN_OFF] = g_Tx.byte_count;
	}
}


//TODO Must be Adrress Matching between Master Request and Slave address
static u8 Is_Addr_Match(void){
	u8 polling_addr;
	
	polling_addr = Get_Polling_Addr();
	if(g_Rx.address_size == SHORT_ADDR_SIZE){
		if( polling_addr == (g_Rx.data_buf[HART_SHORTF_ADDR_OFF]&0x3F) ){
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	else{
//		if( (long_addr.device_type[0] == (g_Rx.data_buf[HART_LONGF_ADDR_OFF]&0x3F)) && \
//			(long_addr.device_type[1] == g_Rx.data_buf[HART_LONGF_ADDR_OFF+1]) && \
//			(long_addr.unique_device_id[0] == g_Rx.data_buf[HART_LONGF_ADDR_OFF+2]) && \
//			(long_addr.unique_device_id[1] == g_Rx.data_buf[HART_LONGF_ADDR_OFF+3]) && \
//			(long_addr.unique_device_id[2] == g_Rx.data_buf[HART_LONGF_ADDR_OFF+4]) ){
			return TRUE;
//		}
//		else{
//			return FALSE;
//		}
	}
}

static u8 Longitudinal_Parity(u8 *data, u16 cnt){
	u8 i;
	u8 check_byte = 0x00;
	
	for(i = 0; i < cnt; i++){
		check_byte ^= *(data+i);
	}
	return check_byte;
}

//Initialize _parm
//TODO Init Value Change
void Init_Param(void){
	Set_Pv(0.0f);
	Set_Sv(0.0f);
	Set_Tv(0.0f);
	Set_Qv(0.0f);
	Set_Loop_Current(4.0f);
	Set_Percent_Of_Range(0.0f);
	Set_Fixed_Current(4.0f);
	Set_Pv_Zero(0.0f);
	_para.ExtMeasuredZeroCurr = 0.0f;
	_para.ActMeasuredZeroCurr = 0.0f;
	_para.ExtMeasuredGainCurr = 0.0f;
	_para.ActMeasuredGainCurr = 0.0f;
	Set_Pv_Unit(0);
	Set_Sv_Unit(0);
	Set_Tv_Unit(0);
	Set_Qv_Unit(0);
	Set_Pv_Code(0);
	Set_Sv_Code(0);
	Set_Tv_Code(0);
	Set_Qv_Code(0);
	Set_Ul_Range_Unit(0);
	Set_Polling_Addr(0);
	Set_Loop_Current_Mode(0);
	Set_Response_Preamble_Num(PREAMBLE_DEFAULT_NUM);
	Set_Burst_Mode_Cmd_Num(0);
	Set_Burst_Mode(0);
	Set_Config_Change_Flag(0);
	Set_Extended_Device_Status(1);
	Set_Device_Operating_Mode(0);
	Set_Std_Status_0(0);
	Set_Std_Status_1(0);
	Set_Std_Status_2(0);
	Set_Std_Status_3(0);
	Set_Analog_Channel_Saturation(0);
	Set_Analog_Channel_Fixed(0);
	Set_Message(0);
	Set_Tag(0);
	Set_LongTag(0);
	Set_Date(0);
	Set_Transducer_Serial_Num(0);
	Set_Device_Specific_Status(0);
	Set_Transducer_Upper(0.0f);
	Set_Transducer_Lower(0.0f);
	Set_Pv_Min_Span(0.0f);
	Set_Pv_Upper_Range(0.0f);
	Set_Pv_Lower_Range(0.0f);
	Set_Pv_Damping_Time(0.0f);
	Set_Alarm_Sw(NONE_ALARM);
	Set_Transfer_Func(LINEAR);
	Set_Protect(NO_PROTECT);
	Set_Analog_Channel(CHANNEL_FLAG);
	Set_Final_Assembly_Num(0);
}

//UART Initialize
void UART_Init(){
	UrtCfg(pADI_UART,B1200,COMLCR_WLS_8BITS,COMLCR_PEN_EN);  // setup baud rate for 1200, 8-bits
   	UrtMod(pADI_UART,COMMCR_DTR,0);              // Setup modem bits
   	UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI);  // Setup UART IRQ sources
   	
}

//UART Enable/Disable
void UART_Enable(u8 rx_enable, u8 tx_enable){
	if((rx_enable) && (tx_enable)){
		UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI);
	}
	else if((!rx_enable) && (tx_enable)){
		UrtIntCfg(pADI_UART,COMIEN_ERBFI_DIS|COMIEN_ETBEI);
	}
	else if((rx_enable) && (!tx_enable)){
		UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI_DIS);
	}
	else{
		UrtIntCfg(pADI_UART,COMIEN_ERBFI_DIS|COMIEN_ETBEI_DIS);
	}
}

//HART Message Transmitt
void HART_Tx_Msg(void){
	if(g_Tx.byte_count >= 2){	
		switch(g_XmtState){
			case XMT_INIT:
				g_XmtState = XMT_WRITE;
			case XMT_WRITE:
				if(s_XmtPreambleNum != 0){
					Send_Byte(PREAMBLE);
					s_XmtPreambleNum--;
				}
				else{
					if(s_XmtBufferCnt != 0){
						Send_Byte(*pXmtBufferCur);
						pXmtBufferCur++;
						s_XmtBufferCnt--;
					}
					else{
						g_XmtState = XMT_DONE;
					}
				}
				if(g_XmtState == XMT_DONE){
					DioSet(pADI_GP0, BIT5);
					UART_Enable(TRUE,FALSE);
					g_XmtState = XMT_INIT;
#ifdef DEBUG
				GLCD_GoTo(10, 2);
    			GLCD_WriteString5x7("XMT_DONE");
#endif
				}
				break;
			default :
				break;
		}	
	}
}

//HART Message Receive
void HART_Rx_Msg(void){

	static u8 preambleNum = 0;
	static u16 byteCount;
	u8 byte;						//Read byte      

	byte = (u8)UrtRx(pADI_UART); 	//Read One Byte
	
	switch(g_RcvState){
		//Preamble Check
		case RCV_WAIT_IDLE:
			if(byte == PREAMBLE){
				preambleNum = 1;
				g_RcvState = RCV_WAIT_START;
				Set_Delay_Time(GAP_TIMER, HART_GAPT);
			}
			break;

		//Delimiter Check
		case RCV_WAIT_START:		
			if(Is_Timeout_Id(GAP_TIMER)){	//Timeout Check
				g_RcvMsgType = RCV_ERR;
			}
			else{
				if(byte == PREAMBLE){
					preambleNum++;
				}
				else{
					switch(byte){
						//Polling Addr, STX
						case(SHORT_FRAME | RCV_STX):
							if(preambleNum > 1){
								g_Rx.address_size = SHORT_ADDR_SIZE;
								g_RcvMsgType = RCV_STX;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;				
							}
							else{
								g_RcvMsgType = RCV_ERR;								
							}
							break;

						//Polling Addr, ACK
						case(SHORT_FRAME | RCV_ACK):
							if(preambleNum > 1){
								g_Rx.address_size = SHORT_ADDR_SIZE;
								g_RcvMsgType = RCV_ACK;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;
							}
							else{
								g_RcvMsgType = RCV_ERR;									
							}
							break;

						//Polling Addr, BACK
						case(SHORT_FRAME | RCV_BACK):
							if(preambleNum > 1){
								g_Rx.address_size = SHORT_ADDR_SIZE;
								g_RcvMsgType = RCV_BACK;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;
							}
							else{
								g_RcvMsgType = RCV_ERR;								
							}
							break;

						//Unique Addr, STX
						case(LONG_FRAME| RCV_STX):	
							if(preambleNum > 1){
								g_Rx.address_size = LONG_ADDR_SIZE;
								g_RcvMsgType = RCV_STX;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;
							}
							else{
								g_RcvMsgType = RCV_ERR;								
							}
							break;

						//Unique Addr, ACK
						case(LONG_FRAME | RCV_ACK):
							if(preambleNum > 1){
								g_Rx.address_size = LONG_ADDR_SIZE;
								g_RcvMsgType = RCV_ACK;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;
							}
							else{
								g_RcvMsgType = RCV_ERR;									
							}
							break;

						//Unique Addr, BACK
						case(LONG_FRAME | RCV_BACK):
							if(preambleNum > 1){
								g_Rx.address_size = LONG_ADDR_SIZE;
								g_RcvMsgType = RCV_BACK;
								s_RcvBufferPos = 0;
								g_Rx.data_buf[s_RcvBufferPos++] = byte;
								g_RcvState = RCV_READ;
							}
							else{
								g_RcvMsgType = RCV_ERR;										
							}
							break;

						//Receive Error
						default:
							g_RcvMsgType = RCV_ERR;
							g_RcvState = RCV_DONE;		
							break;
					}
				}
			}
			Set_Delay_Time(GAP_TIMER, HART_GAPT);
			break;

		//Data Read
		case RCV_READ:
			if(Is_Timeout_Id(GAP_TIMER)){	//Timeout Check
				g_RcvMsgType = RCV_ERR;				
			}
			else{
				g_Rx.data_buf[s_RcvBufferPos++] = byte;
				if(g_Rx.address_size == SHORT_ADDR_SIZE){
					if(s_RcvBufferPos == HART_SHORTF_LEN_OFF+1){
						byteCount = byte;
					}
					if(s_RcvBufferPos > HART_SHORTF_LEN_OFF+byteCount+1){
						g_RcvState = RCV_DONE;
					}
				}
				else{
					if(s_RcvBufferPos == HART_LONGF_LEN_OFF+1){
						byteCount = byte;
					}
					if(s_RcvBufferPos > HART_LONGF_LEN_OFF+byteCount+1){
						g_RcvState = RCV_DONE;
					}
				}
			}
			Set_Delay_Time(GAP_TIMER,HART_GAPT);
			if(g_RcvState == RCV_DONE){			
				Is_Timeout_Id(GAP_TIMER);
				g_HartState = HART_WAIT;
				g_RcvState = RCV_WAIT_IDLE;
				preambleNum = 0;
				byteCount = 0;			
				Set_Rcv_Frame_Count();			
				UART_Enable(FALSE,FALSE);
#ifdef DEBUG
				GLCD_GoTo(10, 2);
    			GLCD_WriteString5x7("RCV_DONE");
				sprintf(LCD,"addr_size is %02x",g_Rx.address_size);
				GLCD_GoTo(10, 3);
    			GLCD_WriteString5x7(LCD);
				sprintf(LCD,"g_RcvMsgType is %02x",g_RcvMsgType);
				GLCD_GoTo(10, 4);
    			GLCD_WriteString5x7(LCD);
#endif
			}
			break;
		default:
			break;
	}
}

//TODO need to upgrade
void Send_Byte(const u8 ch){
    UrtTx(pADI_UART, ch);
}

void Send_Msg(const u8 *str){
	uint16_t len =0 , i;

	len = strlen((char*)str);
	for(i=0; i<len; i++){
		Send_Byte(str[i]);
	}
}

/* receive frame count */
void Set_Rcv_Frame_Count(void){
	Enter_Critical_Section();
	RcvFrameCnt++;
	Exit_Critical_Section();
}

// ASCII TO Packed-ASCII Converter Function
u8 Packed_ASCII(const u8* Src, u16 SrcLen, u8* Dst, u16 DstLen){
	u8 i;
  	u8 byDstIdx = 0;
  	u8 bySrcIdx = 0;

	//Syntax Error
	if (SrcLen == 0)	return FALSE;

	//Convert packed ascii character
  	for (i=0; i<SrcLen; i++){ 
		if((Dst[i] >= 64) && (Dst[i] <= 95)){
			 Dst[i] -= 64;
		 }
  	}
	
  	DstLen = 0;
  	while (SrcLen){ 
		/* First character */
    	Dst[byDstIdx] &= 0x03;
    	Dst[byDstIdx] |= (Src[bySrcIdx] & 0x3F) << 2;
    	(DstLen)++;
    	if (--SrcLen == 0)	return TRUE;
		
    	/* Second character */
    	Dst[byDstIdx] &= 0xFC;
    	Dst[byDstIdx] |= (Src[bySrcIdx+1] & 0x30) >> 4;
    	Dst[byDstIdx+1] &= 0x0F;
    	Dst[byDstIdx+1] |= (Src[bySrcIdx+1] & 0x0F) << 4;
    	(DstLen)++;
    	if (--SrcLen == 0)	return TRUE;
		
    	/* Third character */
    	Dst[byDstIdx+1] &= 0xF0;
    	Dst[byDstIdx+1] |= (Src[bySrcIdx+2] & 0x3C) >> 2;
    	Dst[byDstIdx+2] &= 0x3F;
    	Dst[byDstIdx+2] |= Src[bySrcIdx+2] << 6;
    	(DstLen)++;
    	if (--SrcLen == 0)	return TRUE;
		
    	/* Fourth character */
    	Dst[byDstIdx+2] &= 0xC0;
    	Dst[byDstIdx+2] |= Src[bySrcIdx+3] & 0x3F;
    	if (--SrcLen == 0)	return TRUE;
		
    	Dst += 3; Src += 4; 
  	}
  	return FALSE;
}

// Packed-ASCII TO ASCII Converter Function
u8 Unpcked_ASCII(const u8* Src,u16 SrcLen,u8 *Dst,u16 DstLen){
  	u8 uc;
  	u8 i;
  	u8 byMyLen;

  	DstLen = 0;
  	if (SrcLen < 3)	return FALSE;
	
  	if ((SrcLen % 3)>0)	return FALSE;
	
  	byMyLen = (u8)((SrcLen - (SrcLen % 3)));
  	for (i=0; i<(byMyLen-2); i+=3){ 
		/* First character */
    	uc = (u8)((Src[i] >> 2) & 0x3F);
    	if (uc < 0x20) uc += 0x40;
    	Dst[(DstLen)++] = uc;
		
    	/* Second character */
    	uc = (u8)(((Src[i] << 4 ) & 0x3F) | ((Src[i+1] >> 4) & 0x3F));
    	if (uc < 0x20) uc += 0x40;
    	Dst[(DstLen)++] = uc;
		
    	/* Third character */
    	uc = (u8)(((Src[i+1] << 2 ) & 0x3F) | ((Src[i+2] >> 6) & 0x3F));
    	if (uc < 0x20) uc += 0x40;
    	Dst[(DstLen)++] = uc;
		
    	/* Fourth character */
    	uc = (unsigned char)(Src[i+2] & 0x3F);
    	if (uc < 0x20) uc += 0x40;
   		Dst[(DstLen)++] = uc;
  	}
  	return TRUE;
}


static void Soft_Timer_Dec(Soft_Timer *tmr){
	if(tmr->flg == 0){
		if(tmr->cnt > 0){
			if(--tmr->cnt == 0){
				tmr->flg = 1;
			}
		}
	}
}

void Set_Delay_Time(u8 id,u16 cnt){
	if(id >= TMR_CNT){
		return ;
	}
	g_tmr[id].cnt = cnt;
	if(cnt > 0){
		g_tmr[id].flg = 0;
	}
	else{
		g_tmr[id].flg = 1;
	}
}

u8 Is_Timeout_Id(u8 id){
	if(g_tmr[id].flg == 1){
		g_tmr[id].flg = 0;
		return TRUE;
	}
	return FALSE;
}

//TODO Data Check
void set_ID(u8 *data){
	data[HrtByteCnt++] = 254;										//0: 254
	data[HrtByteCnt++] = DEVICE_TYPE0;								//1-2: Expanded Device Cype
	data[HrtByteCnt++] = DEVICE_TYPE1;
	data[HrtByteCnt++] = Get_Response_Preamble_Num();				//3: Preamble Number
	data[HrtByteCnt++] = HART_REVISION; 							//4: HART Revision
	data[HrtByteCnt++] = DEVICE_REVISION; 							//5: Device Revision
	data[HrtByteCnt++] = SOFT_REVISION;  							//6: Software Revision
	data[HrtByteCnt++] = ((HARD_REVISION)<<3)|BELL_202_CURRENT;		//7(0-2): Physical Signaling Codes(0- Bell 202 Current), (3-7): Hardware Revision, 
	data[HrtByteCnt++] = 0x14; 										//8: FlagAssignment
	data[HrtByteCnt++] = UNIQUE_DEVICE_ID0;							//9-11: Unique Device ID
	data[HrtByteCnt++] = UNIQUE_DEVICE_ID1;
	data[HrtByteCnt++] = UNIQUE_DEVICE_ID2;
	data[HrtByteCnt++] = Get_Response_Preamble_Num();				//12: Minimum Preamble Number
	data[HrtByteCnt++] = 4;											//13: Device Variable Maximum Number
	data[HrtByteCnt++] = ((configurationChangeCounter&0xFF00)>>8);	//14-15: Configuration Changed Counter
	data[HrtByteCnt++] = (configurationChangeCounter&0xFF);
	data[HrtByteCnt++] = 0x02;							//16: Expanded Field Device Status 
	data[HrtByteCnt++] = ((MANUFACTURER_ID&0xFF00)>>8);				//17-18: Manufacturer ID
	data[HrtByteCnt++] = (MANUFACTURER_ID&0xFF);
	data[HrtByteCnt++] = ((DISTRIBUTER_ID&0xFF00)>>8);				//19-20: DISTRIBUTER_ID
	data[HrtByteCnt++] = (DISTRIBUTER_ID&0xFF);
	data[HrtByteCnt++] = 0x01;										//21: Device Profile(1-Process Automation Device)
}

static void Config_Change(void){
	HrtDeviceStatus |= 0x40;
	Set_Config_Change_Flag(0xC0);
}


static float Data_To_Float(u8 *tmp){
	union {
		float tmp_f;
		u8 buf[4];
	}U;
	u8 i;
	
	for(i = 0;i < 4;i++){
		U.buf[i] = *((u8 *)tmp+3-i);
	}
	return U.tmp_f;
}

static void Float_To_Data(u8 *data,float *tmp){
	data[HrtByteCnt++] = *((u8 *)tmp+3);
	data[HrtByteCnt++] = *((u8 *)tmp+2);
	data[HrtByteCnt++] = *((u8 *)tmp+1);
	data[HrtByteCnt++] = *((u8 *)tmp);
}

/* get a pointer to rx_data_buf */
u8 *Get_Rx_Data_Pointer(void){
	u8 *data;
	
	if(g_Rx.address_size == LONG_ADDR_SIZE){
		data = &g_Rx.data_buf[HART_LONGF_REQDATA_OFF];
	}
	else{
		data = &g_Rx.data_buf[HART_SHORTF_REQDATA_OFF];
	}
	return data;
}

void Set_Data_Link(void){
	u8 polling_addr;
	
	polling_addr = Get_Polling_Addr();
	if(g_Rx.address_size == LONG_ADDR_SIZE){
		g_Tx.address_size = LONG_ADDR_SIZE;
	}
	else{
		g_Tx.address_size = SHORT_ADDR_SIZE;
	}
	
	if(g_Burst){               //burst mode command-byte is ?
		g_Tx.delimiter = 0x01;
	}
	else{
		g_Tx.delimiter = 0x06;
		if(g_Rx.data_buf[1] & 0x80){
			g_Host = PRIMARY_MASTER;
		}
		else{
			g_Host = SECONDARY_MASTER;
		}
	}
	if(g_Tx.address_size == LONG_ADDR_SIZE){
		g_Tx.data_buf[0] = g_Tx.delimiter | LONG_FRAME;
		g_Tx.data_buf[1] = (long_addr.device_type[0] & 0x3F) | (g_Host<<7);
		g_Tx.data_buf[2] = long_addr.device_type[1];
		g_Tx.data_buf[3] = long_addr.unique_device_id[0];
		g_Tx.data_buf[4] = long_addr.unique_device_id[1];
		g_Tx.data_buf[5] = long_addr.unique_device_id[2];
	}
	else{
		g_Tx.data_buf[0] = g_Tx.delimiter | SHORT_FRAME;
		g_Tx.data_buf[1] = polling_addr | (g_Host<<7);
	}
}

void Set_Response_Code(u8 *data){
	Set_Data_Link( );
	data[HrtByteCnt++] = HrtResposeCode;
	data[HrtByteCnt++] = HrtDeviceStatus;
}

/* the same as C11 */
void C0_RdUniqueId(u8 *data) {
	Set_Response_Code(data);
	if(!HrtResposeCode){
		set_ID(data);
	}
}

void C1_RdPV(u8 *data){
	float tmp;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Pv();
		data[HrtByteCnt++] = Get_Pv_Unit();
		Float_To_Data(data,&tmp);
	}
}

void C2_RdLoopCurrPerOfRange(u8 *data){
	float tmp1,tmp2;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp1 = Get_Loop_Current();
		tmp2 = Get_Percent_Of_Range();
		Float_To_Data(data,&tmp1);
		Float_To_Data(data,&tmp2);
	}
}


//TODO  Do SV,TV,QV need to this system?
void C3_RdDVLoopCurr(u8 *data){
	float tmp1,tmp2,tmp3,tmp4,tmp5;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp1 = Get_Loop_Current();
		tmp2 = Get_Pv();
		tmp3 = Get_Sv();
		tmp4 = Get_Tv();
		tmp5 = Get_Qv();
		Float_To_Data(data,&tmp1);
		data[HrtByteCnt++] = Get_Pv_Unit();
		Float_To_Data(data,&tmp2);
		data[HrtByteCnt++] = Get_Sv_Unit();
		Float_To_Data(data,&tmp3);
		data[HrtByteCnt++] = Get_Tv_Unit();
		Float_To_Data(data,&tmp4);
		data[HrtByteCnt++] = Get_Qv_Unit();	
		Float_To_Data(data,&tmp5);
	}
}

void C6_WrPollingAddr(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode)
	{
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		data[HrtByteCnt++] = *(dat+1);
		Set_Polling_Addr(*dat);
		Set_Loop_Current_Mode(*(dat+1));
	}
}

void C7_RdLoopConfiguration(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
		data[HrtByteCnt++] = Get_Polling_Addr();
		data[HrtByteCnt++] = Get_Loop_Current_Mode();
	}
}

//TODO Data Check(Table 21)
void C8_RdDVClass(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
		data[HrtByteCnt++] = PV_CLASS;
		data[HrtByteCnt++] = SV_CLASS;
		data[HrtByteCnt++] = TV_CLASS;
		data[HrtByteCnt++] = QV_CLASS;
	}
}

//TODO Data Check
void C9_RdStatusDV(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
/*		data[HrtByteCnt++] = fieldDeviceStatus;
		data[HrtByteCnt++] = DVCode0;
		data[HrtByteCnt++] = DVClass0;
		data[HrtByteCnt++] = DVUnit0;
		Float_To_Data(data,&DVValue0);
		data[HrtByteCnt++] = DVStatus0;
		
		data[HrtByteCnt++] = DVCode1;
		data[HrtByteCnt++] = DVClass1;
		data[HrtByteCnt++] = DVUnit1;
		Float_To_Data(data,&DVValue1);
		data[HrtByteCnt++] = DVStatus1;

		data[HrtByteCnt++] = DVCode2;
		data[HrtByteCnt++] = DVClass2;
		data[HrtByteCnt++] = DVUnit2;
		Float_To_Data(data,&DVValue2);
		data[HrtByteCnt++] = DVStatus2;

		data[HrtByteCnt++] = DVCode3;
		data[HrtByteCnt++] = DVClass3;
		data[HrtByteCnt++] = DVUnit3;
		Float_To_Data(data,&DVValue3);
		data[HrtByteCnt++] = DVStatus3;

		data[HrtByteCnt++] = DVCode4;
		data[HrtByteCnt++] = DVClass4;
		data[HrtByteCnt++] = DVUnit4;
		Float_To_Data(data,&DVValue4);
		data[HrtByteCnt++] = DVStatus4;

		data[HrtByteCnt++] = DVCode5;
		data[HrtByteCnt++] = DVClass5;
		data[HrtByteCnt++] = DVUnit5;
		Float_To_Data(data,&DVValue5);
		data[HrtByteCnt++] = DVStatus5;
		
		data[HrtByteCnt++] = DVCode6;
		data[HrtByteCnt++] = DVClass6;
		data[HrtByteCnt++] = DVUnit6;
		Float_To_Data(data,&DVValue6);
		data[HrtByteCnt++] = DVStatus6;

		data[HrtByteCnt++] = DVCode7;
		data[HrtByteCnt++] = DVClass7;
		data[HrtByteCnt++] = DVUnit7;
		Float_To_Data(data,&DVValue7);
		data[HrtByteCnt++] = DVStatus7;

		Float_To_Data(data,&TIMEStamp);*/
	}
}

void C11_RdUniqueIDWithTag(u8 *data){
	//Same C0
}

/* message type : packed */
void C12_RdMessage(u8 *data) {
	u8 i = 0;
	u8 *dat;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Message();
		for(i = 0;i < 24;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
	}
}

void C13_RdTagDescriptorDate(u8 *data){
	u8 i = 0;
	u8 *dat1, *dat2, *dat3;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat1 = Get_Tag();
		dat2 = Get_Descriptor();
		dat3 = Get_Date();
		for(i = 0;i < 6;i++){
			data[HrtByteCnt++] = *(dat1+i);
		}
		for(i = 0;i < 12;i++){
			data[HrtByteCnt++] = *(dat2+i);
		}
		for(i = 0;i < 3;i++){
			data[HrtByteCnt++] = *(dat3+i);
		}
	}
}

void C14_RdPVTransducerInfo(u8 *data){
	u8 i = 0;
	u8 *dat;	
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Transducer_Serial_Num();
		for(i = 0;i < 3;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
		data[HrtByteCnt++] = Get_Pv_Unit();
		tmp = Get_Transducer_Upper();
		Float_To_Data(data,&tmp);
		tmp = Get_Transducer_Lower();
		Float_To_Data(data,&tmp);
		tmp = Get_Pv_Min_Span();
		Float_To_Data(data,&tmp);
	}
}

void C15_RdDeviceInfo(u8 *data){
	float tmp1,tmp2,tmp3;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp1 = Get_Pv_Upper_Range();
		tmp2 = Get_Pv_Lower_Range();
		tmp3 = Get_Pv_Damping_Time();
		data[HrtByteCnt++] = Get_Alarm_Sw();
		data[HrtByteCnt++] = Get_Transfer_Func();
		data[HrtByteCnt++] = Get_Ul_Range_Unit();
		Float_To_Data(data,&tmp1);
		Float_To_Data(data,&tmp2);
		Float_To_Data(data,&tmp3);
		data[HrtByteCnt++] = Get_Protect();
		data[HrtByteCnt++] = 250;
		data[HrtByteCnt++] = Get_Analog_Channel();
	}
}

void C16_RdFinalAssemblyNum(u8 *data){
	u8 *dat;
	u8 i;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Final_Assembly_Num();
		for(i = 0;i < 3;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
	}
}

void C17_WrMessage(u8 *data){
	u8 *dat;
	u8 i;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		for(i = 0;i < 24;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
		Set_Message(dat);
		Config_Change();
	}
}

void C18_WrTagDescriptorDate(u8 *data){
	u8 *dat;
	u8 i;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		for(i = 0;i < 21;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
		Set_Tag(dat);
		Set_Descriptor(dat+6);
		Set_Date(dat+18);
		Config_Change();
	}
}

void C19_WrFinalAssemblyNum(u8 *data){
	u8 *dat;
	u8 i;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		for(i = 0;i < 3;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
		Set_Final_Assembly_Num(dat);
		Config_Change();
	}
}

void C20_RdLongTag(u8 *data){
	u8 i = 0;
	u8 *dat1;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat1 = Get_LongTag();
		for(i = 0;i < 32;i++){
			data[HrtByteCnt++] = *(dat1+i);
		}
	}
}

void C21_RdUniqueIDWithLongTag(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
		//SAME C0
	}
}

void C22_WrLongTag(u8 *data){
	u8 *dat;
	u8 i;	
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		for(i = 0;i < 32;i++){
			data[HrtByteCnt++] = *(dat+i);
		}
		Set_LongTag(dat);
		Config_Change();
	}
}

void C33_RdDeviceVariable(u8 *data){
	u8 *dat;
	u8 i;
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		for(i = 0;i < 4;i++){
			switch(*(dat+i)){
				case PERCENT_RANGE:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = PERCENT_UNIT;
					tmp = Get_Percent_Of_Range();
					Float_To_Data(data,&tmp);
					break;
				case LOOP_CURRENT:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = CURRENT_UNIT;
					tmp = Get_Loop_Current();
					Float_To_Data(data,&tmp);
					break;
				case PV_CODE:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = Get_Pv_Unit();
					tmp = Get_Pv();
					Float_To_Data(data,&tmp);
					break;
				case SV_CODE:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = Get_Sv_Unit();
					tmp = Get_Sv();
					Float_To_Data(data,&tmp);
					break;
				case TV_CODE:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = Get_Tv_Unit();
					tmp = Get_Tv();
					Float_To_Data(data,&tmp);
					break;
				case QV_CODE:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = Get_Qv_Unit();
					tmp = Get_Qv();
					Float_To_Data(data,&tmp);
					break;
				default:
					data[HrtByteCnt++] = *(dat+i);
					data[HrtByteCnt++] = 250;   //not used
					data[HrtByteCnt++] = 0x7F;
					data[HrtByteCnt++] = 0xA0;
					data[HrtByteCnt++] = 0x00;
					data[HrtByteCnt++] = 0x00;
					break;
			}
		}
	}
}

void C34_WrPVDamping(u8 *data){
	u8 *dat;
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode)
	{
		dat = Get_Rx_Data_Pointer();
		tmp = Data_To_Float(dat);
		Float_To_Data(data,&tmp);
		Set_Pv_Damping_Time(tmp);
		Config_Change();
	}
}

void C35_WrPVRange(u8 *data){
	u8 *dat;
	float tmp1,tmp2;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		tmp1 = Data_To_Float(dat+1);
		tmp2 = Data_To_Float(dat+5);
		data[HrtByteCnt++] = *dat;		
		Float_To_Data(data,&tmp1);	
		Float_To_Data(data,&tmp2);	
		Set_Ul_Range_Unit(*dat);
		Set_Pv_Upper_Range(tmp1);
		Set_Pv_Lower_Range(tmp2);
		Config_Change();
	}
}

void C36_SetPVUpperRange(u8 *data){
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Pv();
		Set_Pv_Upper_Range(tmp);
		Config_Change();
	}	
}

void C37_SetPVLowerRange(u8 *data){
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Pv();
		Set_Pv_Lower_Range(tmp);
		Config_Change();
	}
}

void C38_ResetCfgChangeFlag(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
		HrtDeviceStatus &= ~0x40;
		if(Get_Host_Type()){
			Set_Config_Change_Flag(0x80); //primary 
		}
		else{
			Set_Config_Change_Flag(0x40);  //second
		}
	}
}

void C40_EnterOrExitFixedCurrent(u8 *data){
	float tmp;
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Loop_Current();
		Float_To_Data(data,&tmp);
		dat = Get_Rx_Data_Pointer();
		tmp = Data_To_Float(dat);
		Set_Fixed_Current(tmp);
	}
}

void C41_PerformSelfTest(u8 *data){
	//PerformSelfTest func;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		//func = (PerformSelfTest)Get_Perform_Self_Test_Ptr();
		//func();
	}
}

void C42_PerformDeviceReset(u8 *data){
	//PerformDeviceReset func;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		//func = (PerformDeviceReset)Get_Perform_Device_Reset_Ptr();
		//func();
	}
}

void C43_PVZero(u8 *data){
	float tmp;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Pv();
		Set_Pv_Zero(tmp);
	}
}

void C44_WrPVUnit(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		Set_Pv_Unit(*dat);
		Config_Change();
	}
}

void C45_TrimLoopCurrentZero(u8 *data){
	u8 *dat;
	float tmp;
	//TrimLoopCurrent func;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Fixed_Current();
		if((tmp>=3.0) && (tmp<=5.0)){          //enter fixed current mode
			dat = Get_Rx_Data_Pointer();
			tmp = Data_To_Float(dat);
			//func = (TrimLoopCurrent)Get_Zero_Trim_Ptr();
			//func(&tmp);   //
			tmp = Get_Act_Zero_Current();
			Float_To_Data(data,&tmp);	
		}
		else{
			tmp = 9.0f;
			Float_To_Data(data,&tmp);
		}
	}
}

void C46_TrimLoopCurrentGain(u8 *data){
	u8 *dat;
	float tmp;
	//TrimLoopCurrent func;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		tmp = Get_Fixed_Current();
		if((tmp>=19.0f) && (tmp<=21.0f)){           //enter fixed current mode
			dat = Get_Rx_Data_Pointer();
			tmp = Data_To_Float(dat);
			//func = (TrimLoopCurrent)Get_Gain_Trim_Ptr();
			//func(&tmp);
			tmp = Get_Act_Gain_Current();
			Float_To_Data(data,&tmp);	
		}
		else{
			tmp = 9.0f;
			Float_To_Data(data,&tmp);
		}
	}
}

void C47_WrPVTransferFunction(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		Set_Transfer_Func((enum transfer_func)(*dat));
		Config_Change();
	}
}

void C48_RdAdditionalDeviceStatus(u8 *data){
	u8 i;
	u8 *dat;
	
	if(!HrtResposeCode){
		dat = Get_Device_Specific_Status();
		for(i = 0;i < 6;i++){
				data[HrtByteCnt++] = *(dat+i);
		}
		data[HrtByteCnt++] = Get_Extended_Device_Status();
		data[HrtByteCnt++] = Get_Device_Operating_Mode();
		data[HrtByteCnt++] = Get_Std_Status_0();
		data[HrtByteCnt++] = Get_Std_Status_1();
		data[HrtByteCnt++] = Get_Analog_Channel_Saturation();
		data[HrtByteCnt++] = Get_Std_Status_2();
		data[HrtByteCnt++] = Get_Std_Status_3();
		data[HrtByteCnt++] = Get_Analog_Channel_Fixed();
	}
}

void C49_WrPVTransducerSerialNum(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		data[HrtByteCnt++] = *(dat+1);
		data[HrtByteCnt++] = *(dat+2);
		Set_Transducer_Serial_Num(dat);
		Config_Change();
	}
}

void C50_RdDVAssignments(u8 *data){
	Set_Response_Code(data);
	if(!HrtResposeCode){
		data[HrtByteCnt++] = Get_Pv_Code();
		data[HrtByteCnt++] = Get_Sv_Code();
		data[HrtByteCnt++] = Get_Tv_Code();
		data[HrtByteCnt++] = Get_Qv_Code();
	}
}

void C51_WrDVAssignments(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		data[HrtByteCnt++] = *(dat+1);
		data[HrtByteCnt++] = *(dat+2);
		data[HrtByteCnt++] = *(dat+3);
		Set_Pv_Code(*dat);
		Set_Sv_Code(*(dat+1));
		Set_Tv_Code(*(dat+2));
		Set_Qv_Code(*(dat+3));
		Config_Change();
	}
}

void C59_WrNumOfResposePreambles(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		Set_Response_Preamble_Num(*dat);
		Config_Change();
	}
}

void C108_WrBurstModeCmdNum(u8 *data){ //command 1,2,3,9 should be supported by all devices
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		Set_Burst_Mode_Cmd_Num(*dat);
		Config_Change();
	}
}

void C109_BurstModeControl(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *dat;
		Set_Burst_Mode(*dat);
	}
}

void C200_WrTest(u8 *data){
	u8 *dat;
	
	Set_Response_Code(data);
	if(!HrtResposeCode){
		dat = Get_Rx_Data_Pointer();
		data[HrtByteCnt++] = *(dat);
		data[HrtByteCnt++] = *(dat+1);
		data[HrtByteCnt++] = *(dat+2);
		data[HrtByteCnt++] = *(dat+3);
	}
}

//UART Interrupt Handler
void UART_Int_Handler(void){
	volatile u8 ucCOMIID0 = 0;
		
	ucCOMIID0 = UrtIntSta(pADI_UART);   	// Read UART Interrupt ID register
   	if ((ucCOMIID0 & 0x7) == 0x2){       	// Transmit buffer empty
   		HART_Tx_Msg();
   	}
   	if ((ucCOMIID0 & 0x7) == 0x4){       	// Receive byte
   		HART_Rx_Msg();
   	}
}

void GP_Tmr0_Int_Handler(void)
{
	u8 i;
	
   // Timer0 Interrupt : 1msec
   GptClrInt(pADI_TM0, TSTA_TMOUT);
   dly_cnt++;

	for(i = 0; i < TMR_CNT; i++){
		Soft_Timer_Dec(&g_tmr[i]);
	}
}