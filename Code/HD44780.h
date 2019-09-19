#ifndef __HD44780_H
#define __HD44780_H

//-------------------------------------------------------------------------------------------------
//
// Instructions of Hitachi HD44780 controller
//
//-------------------------------------------------------------------------------------------------

// Sync
#define HD44780_SYNC			0x30

// Commands to LCD module
#define HD44780_CMD_CLEAR           0x01
#define HD44780_CMD_HOME            0x02
#define HD44780_CMD_ENTRY           0x04
#define HD44780_CMD_DISPLAY         0x08
#define HD44780_CMD_CD_SHIFT        0x10
#define HD44780_CMD_FUNCTION        0x20
#define HD44780_CMD_CGRAMADDR       0x40
#define HD44780_CMD_SET_DDADDR      0x80

// Settings for HD44780_CMD_ENTRY
#define HD44780_ENTRY_MOVE_DISPLAY  0x01
#define HD44780_ENTRY_MOVE_CURSOR   0x00
#define HD44780_ENTRY_INC           0x02    
#define HD44780_ENTRY_DEC           0x00

// Settings for HD44780_CMD_DISPLAY
#define HD44780_DISPLAY_BLINK       0x01
#define HD44780_DISPLAY_NOBLINK     0x00
#define HD44780_DISPLAY_CURSOR      0x02
#define HD44780_DISPLAY_NOCURSOR    0x00
#define HD44780_DISPLAY_ON          0x04
#define HD44780_DISPLAY_OFF         0x00

// Settings for HD44780_CMD_CD_SHIFT (shift cursor or display without changing data)
#define HD44780_CD_SHIFT_RIGHT      0x04
#define HD44780_CD_SHIFT_LEFT       0x00
#define HD44780_CD_SHIFT_DISPLAY    0x08
#define HD44780_CD_SHIFT_CURSOR     0x00

// Settings for HD44780_CMD_FUNCTION
#define HD44780_FUNCTION_5X10FONT   0x04
#define HD44780_FUNCTION_5X8FONT    0x00
#define HD44780_FUNCTION_2LINES     0x08
#define HD44780_FUNCTION_1LINE      0x00
#define HD44780_FUNCTION_8BIT       0x10
#define HD44780_FUNCTION_4BIT       0x00



#endif /* __HD44780_H */
