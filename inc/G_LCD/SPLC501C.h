
#define SCREEN_WIDTH 	132
#define SCREEN_HEIGHT	64
#define PIXELS_PER_PAGE	8

#define SPLC501C_DISPLAY_ON 		0xAF
#define SPLC501C_DISPLAY_OFF		0xAE

#define SPLC501C_START_LINE			0x40

#define SPLC501C_PAGE_ADDRESS		0xB0

#define SPLC501C_COLUMN_ADDRESS_HI	0x10
#define SPLC501C_COLUMN_ADDRESS_LO	0x00

#define SPLC501C_ADC_NORMAL			0xA0
#define SPLC501C_ADC_REVERSE		0xA1

#define SPLC501C_DISPLAY_NORMAL		0xA6
#define SPLC501C_DISPLAY_REVERSE	0xA7

#define SPLC501C_DISPLAY_ALL_ON		0xA5
#define SPLC501C_DISPLAY_ALL_OFF	0xA4

#define SPLC501C_BIAS_19			0xA2
#define SPLC501C_BIAS_15			0xA3

#define SPLC501C_RMW_START			0xE0
#define SPLC501C_RMW_END			0xEE

#define SPLC501C_RESET				0xE2

#define SPLC501C_COM0				0xC0
#define SPLC501C_COM63				0xC8

#define SPLC501C_POWERON			0x2F

#define SPLC501C_VOLTAGE_RATIO		0x20

#define SPLC501C_VOLUME_MODE		0x81
#define SPLC501C_VOLUME_SET			0x00

#define SPLC501C_PAGE_BLINKING_MODE	0xD5
#define SPLC501C_PAGE_BLINKING_0	0x01
#define SPLC501C_PAGE_BLINKING_1	0x02
#define SPLC501C_PAGE_BLINKING_2	0x04
#define SPLC501C_PAGE_BLINKING_3	0x08
#define SPLC501C_PAGE_BLINKING_4	0x10
#define SPLC501C_PAGE_BLINKING_5	0x20
#define SPLC501C_PAGE_BLINKING_6	0x40
#define SPLC501C_PAGE_BLINKING_7	0x80

#define LCD_RST_H	   DioSet(pADI_GP2, 0x02)
#define LCD_RST_L	   DioClr(pADI_GP2, 0x02)
#define LCD_A0_H	   DioSet(pADI_GP1, 0x20)
#define LCD_A0_L	   DioClr(pADI_GP1, 0x20)
#define LCD_CS1_H	   DioSet(pADI_GP1, 0x80)
#define LCD_CS1_L	   DioClr(pADI_GP1, 0x80)
#define LCD_SCL_H	   DioSet(pADI_GP2, 0x01)
#define LCD_SCL_L	   DioClr(pADI_GP2, 0x01)
#define LCD_SI_H	   DioSet(pADI_GP1, 0x40)
#define LCD_SI_L	   DioClr(pADI_GP1, 0x40)

void GLCD_GoTo(unsigned char, unsigned char);
void GLCD_WriteString(char *);
void GLCD_Initialize(void);
void GLCD_ClearScreen(void);
void GLCD_Bitmap(char *, unsigned char, unsigned char, unsigned char, unsigned char);
