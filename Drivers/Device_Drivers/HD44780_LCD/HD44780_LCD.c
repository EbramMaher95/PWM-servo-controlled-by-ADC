/*
 * HD44780_LCD.c
 *
 *  Created on: Feb 3, 2024
 *      Author: ramys
 */

#include "HD44780_LCD.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
/*although some LCDs support 4-line options, the 2 extra lines are treated as extensions to the
 first 2 lines in terms of memory location
 for example:
 for a 4x16 display:
 - the address of the first character on line 1 is 0x00
 - the address of the first character on line 2 is 0x40
 - the address of the first character on line 3 is 0x00 + 16

 Display Data RAM (DDRAM) Allocation for a 4x16 display
 ------------------------------------------
 |0x0									0x0f| --> Line 1
 |0x40									0x4f| --> Line 2
 |0x10									0x1f| --> Line 3
 |0x50									0x5f| --> Line 4
 ------------------------------------------

 For this reason, we select 2 lines during initialization (which will mean that we will have 2 areas, 0x00 and 0x40)
 The other option will be 1 line, which will mean that we will only have one area inside the DDRAM leaving only Line 1 and Line 3 (suitable only for 1-line display panels)
 */
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define SendByte(cmd_data, value) Alcd_SendByte(lcd, cmd_data, value)

#define PulseEn                     \
  do                                \
  {                                 \
    EN_SET(lcd,1);    \
    usDelay(10);   \
    EN_SET(lcd,0);    \
    usDelay(100); \
  } while (0)

/* Hardware interface functions */
static void usDelay(uint16_t delay_us) {
	DWT_Delay_us(delay_us);
}
/// the top 4 bits are ignored
/// transfers the status of the 4-bit data to the hardware GPIO pins
static void Write_HalfByte(Alcd_t *lcd, uint8_t HalfByte) {
	lcd->Data_GPIO->ODR &= ~(0xf << lcd->Data_GPIO_Start_Pin);
	lcd->Data_GPIO->ODR |= HalfByte << lcd->Data_GPIO_Start_Pin;
}
/// @brief 1 for set and 0 for reset
static void RS_SET(Alcd_t *lcd, uint8_t R_S_Stat) {
	HAL_GPIO_WritePin(lcd->RS_GPIO, lcd->RS_GPIO_Pin, R_S_Stat);
}
/// @brief 1 for set and 0 for reset
static void EN_SET(Alcd_t *lcd, uint8_t EN_Stat) {
	HAL_GPIO_WritePin(lcd->EN_GPIO, lcd->EN_GPIO_Pin, EN_Stat);
}

static void Alcd_Init_GPIO(Alcd_t *lcd) {
	GPIO_InitTypeDef G = { .Mode = GPIO_MODE_OUTPUT_PP, .Pin = lcd->RS_GPIO_Pin,
			.Speed = GPIO_SPEED_FREQ_LOW };
	HAL_GPIO_Init(lcd->RS_GPIO, &G);
	G.Pin = lcd->EN_GPIO_Pin;
	HAL_GPIO_Init(lcd->EN_GPIO, &G);

	G.Pin = 0xf << lcd->Data_GPIO_Start_Pin;
	HAL_GPIO_Init(lcd->Data_GPIO, &G);
}

/* User Functions  */
void Alcd_Display(Alcd_t *lcd, uint8_t ON_OFF);

static inline void Alcd_SendByte(Alcd_t *lcd, uint8_t CMD_Data, uint8_t value) {
	RS_SET(lcd, CMD_Data);
	// send the higher 4 bits
	Write_HalfByte(lcd, value >> 4);
	// pulse the enable pin
	PulseEn
	;
	Write_HalfByte(lcd, value);
	PulseEn
	;
}

void Alcd_Init(Alcd_t *lcd, uint8_t Lines, uint8_t Chars) {
	//INitialize the delay function using the ARM core cycle counter
	DWT_Delay_Init();
	Alcd_Init_GPIO(lcd);
	uint8_t x;

	lcd->RowOffsets[0] = 0;
	lcd->RowOffsets[1] = 0x40;
	lcd->RowOffsets[2] = 0 + Chars;
	lcd->RowOffsets[3] = 0x40 + Chars;

	RS_SET(lcd, 0);
	EN_SET(lcd, 0);
	usDelay(50000);

	// init display in 4-bit mode
	for (x = 0; x < 2; x++) {
		Write_HalfByte(lcd, 0x03);
		PulseEn
		;
		usDelay(4500);
	}
	Write_HalfByte(lcd, 0x03);
	PulseEn
	;
	usDelay(150);
	Write_HalfByte(lcd, 0x02);
	PulseEn
	;

	// finally, set # lines, font size, etc.
	SendByte(0, LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS);

	// turn the display on with no cursor or blinking default
	// lcd->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

	// turn on the display
	Alcd_Display_Control(lcd, 1, 0, 0);
	Alcd_Clear(lcd);
}

void Alcd_CursorAt(Alcd_t *lcd, uint8_t Row, uint8_t Col) {
	SendByte(0, LCD_SETDDRAMADDR | (Col + lcd->RowOffsets[Row]));
}

void Alcd_Put_n(Alcd_t *lcd, char *text, uint8_t len) {
	for (uint8_t x = 0; x < len; x++) {
		SendByte(1, *(text++));
	}
}

void Alcd_PutAt_n(Alcd_t *lcd, uint8_t Row, uint8_t Col, char *text,
		uint8_t len) {
	Alcd_CursorAt(lcd, Row, Col);
	Alcd_Put_n(lcd, text, len);
}

void Alcd_Home(Alcd_t *lcd) {
	SendByte(0, LCD_RETURNHOME);
	usDelay(2000);
}

void Alcd_Clear(Alcd_t *lcd) {
	SendByte(0, LCD_CLEARDISPLAY);
	usDelay(2000);
}

void Alcd_Display_Control(Alcd_t *lcd, uint8_t ON_OFF, uint8_t CUR_ON_OFF,
		uint8_t BLINK_ON_OFF) {
	lcd->_displaycontrol = 0;
	if (ON_OFF) {
		lcd->_displaycontrol |= LCD_DISPLAYON;
	}
	if (CUR_ON_OFF) {
		lcd->_displaycontrol |= LCD_CURSORON;
	}
	if (BLINK_ON_OFF) {
		lcd->_displaycontrol |= LCD_BLINKON;
	}
	lcd->_displaycontrol |= LCD_DISPLAYON;
	SendByte(0, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void Alcd_CreateChar(Alcd_t *lcd, uint8_t Location, uint8_t Map[]) {
	uint8_t x = 0;
	// only 8 locations are
	Location &= 7;
	SendByte(0, LCD_SETCGRAMADDR | (Location << 3));
	for (x = 0; x < 8; x++) {
		SendByte(1, Map[x]);
	}
}

void Alcd_PutChar(Alcd_t *lcd, char chr) {
	SendByte(1, chr);
}

int Str_Len(char *string) {
	int len = 0;
	while (*(string++)) {
		len++;
	}
	return len;
}
