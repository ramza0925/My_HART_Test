#include "SPLC501C.h"
#include "font5x7.h"
#include "font12x16.h"
#include "font19x24.h"

void GLCD_WriteCommand(unsigned char);
void GLCD_WriteData(unsigned char);
unsigned char GLCD_ReadData(void);
void GLCD_InitializePorts(void);

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitializePorts(void)
{
	LCD_CS1_L; delay_us(30);

	LCD_RST_H; delay_ms(1);
	LCD_RST_L; delay_ms(1);	
	LCD_RST_H; delay_ms(1);
	
	LCD_CS1_H;
	LCD_A0_H;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
	LCD_CS1_L; delay_us(1);
	LCD_A0_H; delay_us(1);	

	for (int i = 0; i < 8; i++) 
	{		
		LCD_SCL_L;	
		
		if (dataToWrite & 0x80) LCD_SI_H;
		else LCD_SI_L;
			
		LCD_SCL_H;
		dataToWrite = dataToWrite << 1;		
	}

	LCD_CS1_H;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite)
{
	LCD_CS1_L; delay_us(1);
	LCD_A0_L; delay_us(1);

	for (int i = 0; i < 8; i++) 
	{	
		LCD_SCL_L; 	
		
		if (commandToWrite & 0x80) LCD_SI_H;
		else LCD_SI_L;
		
		LCD_SCL_H;
		commandToWrite = commandToWrite << 1;
	}	

	LCD_A0_H;
	LCD_CS1_H;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Initialize(void)
{
	GLCD_InitializePorts();

	GLCD_WriteCommand(SPLC501C_RESET);					
	GLCD_WriteCommand(SPLC501C_ADC_REVERSE);
	GLCD_WriteCommand(SPLC501C_BIAS_19);						
	GLCD_WriteCommand(SPLC501C_COM0);
	GLCD_WriteCommand(SPLC501C_POWERON);
		
	delay_ms(100);
	
	GLCD_WriteCommand(0xA4);
	GLCD_WriteCommand(SPLC501C_VOLUME_MODE);						
	GLCD_WriteCommand(SPLC501C_VOLUME_SET | 32);
	GLCD_WriteCommand(SPLC501C_VOLTAGE_RATIO | 3);				
					
	GLCD_WriteCommand(SPLC501C_DISPLAY_ON);
	GLCD_WriteCommand(SPLC501C_DISPLAY_NORMAL);		
	
	GLCD_WriteCommand(SPLC501C_PAGE_ADDRESS | 0);
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_HI | 0);		
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_LO | 0);
	GLCD_WriteCommand(SPLC501C_START_LINE | 0);	
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_GoTo(unsigned char x, unsigned char y)
{	
    glcd_y = y;
    glcd_x = x;
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_HI | (x >> 4));
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_LO | (x & 0x0F));
	GLCD_WriteCommand(SPLC501C_PAGE_ADDRESS | y);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_ClearScreen(void)
{
	unsigned char x = 0, y = 0;
	for(y = 0; y < (SCREEN_HEIGHT/PIXELS_PER_PAGE); y++)
	{
		GLCD_GoTo(0,y);
		for(x = 0; x < SCREEN_WIDTH; x++)
		{
			GLCD_WriteData(0);
		}
	}
}		
//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteChar5x7
// Artuments : Char ASCII code
// Return value : none
//-------------------------------------------------------------------------------------------------
void GLCD_WriteChar5x7(char charCode)
{
	unsigned char fontCollumn;
	
	for(fontCollumn = 0; fontCollumn < FONT_WIDTH; fontCollumn++)
	  GLCD_WriteData(font5x7[((charCode- FONT_OFFSET) * FONT_WIDTH) + fontCollumn]);
	GLCD_WriteData(0);
}
//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteString5x7
// Arguments : pointer to null-terminated ASCII string
// Return value : none
//-------------------------------------------------------------------------------------------------
void GLCD_WriteString5x7(char * string)
{
	while(*string)
	{
		GLCD_WriteChar5x7(*string++);
	}
}

//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteChar15x17
// Artuments : Char ASCII code
// Return value : none
//-------------------------------------------------------------------------------------------------
void GLCD_WriteChar12x16(char charCode) {
    unsigned int fontCollumn;
    unsigned int x,y;

    y = glcd_y;
    x = glcd_x;
	
    for (fontCollumn = 1; fontCollumn < FONT_WIDTH12x16; fontCollumn+=2) {
        GLCD_GoTo(x,y);
        GLCD_WriteData(font12x16[((charCode - FONT_OFFSET12x16) * FONT_WIDTH12x16) + fontCollumn]);
        GLCD_GoTo(x,y+1);
        GLCD_WriteData(font12x16[((charCode - FONT_OFFSET12x16) * FONT_WIDTH12x16) + fontCollumn + 1]);
        x++;
    }
    GLCD_WriteData(0);
    if (charCode == '.') x -= 5;
    GLCD_GoTo(x,y);
}

void GLCD_InvWriteChar12x16(char charCode) {
    unsigned int fontCollumn;
    unsigned int x,y;

    y = glcd_y;
    x = glcd_x;
	
    for (fontCollumn = 1; fontCollumn < FONT_WIDTH12x16; fontCollumn+=2) {
        GLCD_GoTo(x,y);
        GLCD_WriteData(~font12x16[((charCode - FONT_OFFSET12x16) * FONT_WIDTH12x16) + fontCollumn]);
        GLCD_GoTo(x,y+1);
        GLCD_WriteData(~font12x16[((charCode - FONT_OFFSET12x16) * FONT_WIDTH12x16) + fontCollumn + 1]);
        x++;
    }
    GLCD_WriteData(0);
    GLCD_GoTo(x,y);
}
//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteString10x16
// Arguments : pointer to null-terminated ASCII string
// Return value : none
//-------------------------------------------------------------------------------------------------

void GLCD_WriteString12x16(char * string) {
    while (*string) {
        GLCD_WriteChar12x16(*string++);
    }
}
void GLCD_InvWriteString12x16(char * string) {
    while (*string) {
        GLCD_InvWriteChar12x16(*string++);
    }
}
//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteChar10x16
// Artuments : Char ASCII code
// Return value : none
//-------------------------------------------------------------------------------------------------
void GLCD_WriteChar19x24(char charCode) {
    unsigned int fontCollumn;
    unsigned int x,y;

    y = glcd_y;
    x = glcd_x;
	
    for (fontCollumn = 1; fontCollumn < FONT_WIDTH19x24; fontCollumn+=3) {
        GLCD_GoTo(x,y);
        GLCD_WriteData(font19x24[((charCode - FONT_OFFSET19x24) * FONT_WIDTH19x24) + fontCollumn]);
        GLCD_GoTo(x,y+1);
        GLCD_WriteData(font19x24[((charCode - FONT_OFFSET19x24) * FONT_WIDTH19x24) + fontCollumn + 1]);
        GLCD_GoTo(x,y+2);
        GLCD_WriteData(font19x24[((charCode - FONT_OFFSET19x24) * FONT_WIDTH19x24) + fontCollumn + 2]);
        x++;
    }
    GLCD_WriteData(0);
	if (charCode == '.') x -= 5;
    GLCD_GoTo(x,y);
}
//-------------------------------------------------------------------------------------------------
// Function : GLCD_WriteString10x16
// Arguments : pointer to null-terminated ASCII string
// Return value : none
//-------------------------------------------------------------------------------------------------

void GLCD_WriteString19x24(char * string) {
    while (*string) {
        GLCD_WriteChar19x24(*string++);
    }
}
//-------------------------------------------------------------------------------------------------
// Function : GLCD_SetPixel
// Arguments : x-location, y-location, color (0 or 1)
// Return value : None
//-------------------------------------------------------------------------------------------------
void GLCD_SetPixel(int x, int y, int color)
{
	unsigned char temp = 0;  
	GLCD_GoTo(x, (y/8)); 
	//temp = GLCD_ReadData(); 
	if(color)
		temp |= (1 << (y % 8));
	else
		temp &= ~(1 << (y % 8));
	GLCD_GoTo(x, (y/8)); 
	GLCD_WriteData(temp); 
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Bitmap(char * bitmap,unsigned char left, unsigned char top, unsigned char width, unsigned char height)
{
	unsigned char pageIndex, columnIndex;
	for(pageIndex = 0; pageIndex < height / 8; pageIndex++)
	{
		GLCD_GoTo(left, top + pageIndex);
		for(columnIndex = 0; columnIndex < width; columnIndex++)
			GLCD_WriteData(*(bitmap++)); 
	}
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

		
