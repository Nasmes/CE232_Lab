#ifndef MAIN_SSD1366_H_
#define MAIN_SSD1366_H_

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with contrast ratio (0 -> 255), 127 is default value
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 (H mode), 0x01 (V mode), 0x02 (P mode)
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // HORZ/VERT mode only - follow with 2 bytes (eg: 0x00, 0x7F means COL 0 - 127)
#define OLED_CMD_SET_PAGE_RANGE         0x22    // HORZ/VERT mode only - follow with 2 bytes (eg: 0x00, 0x07 means PAGE 0 - 7)
#define OLED_CMD_SET_PAGE_ADDRESS       0xB0    // PAGE mode only - add with page number (0 -> 7) (eg: PAGE 2, OLED_CMD_SET_PAGE_ADDRESS + 2)
#define OLED_CMD_SET_COL_ADDR_LOW       0x00    // PAGE mode only - add with lower half of COL pointer (0x0 -> 0xF) (eg: COL 18 = 0x12, OLED_CMD_SET_COL_ADDR_LOW + 0x2)
#define OLED_CMD_SET_COL_ADDR_HIGH      0x10    // PAGE mode only - add with upper half of COL pointer (0x0 -> 0x7) (eg: COL 18 = 0x12, OLED_CMD_SET_COL_ADDR_HIGH + 0x1)

// Hardware Config (pg.31)
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with number of MUX-1 (15 -> 64), 64 is default value
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40    // add with RAM2MUX offset (0 -> 63) (eg: OLED_CMD_SET_DISPLAY_START_LINE + 16)
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // add with MUX2PIN offset (0 -> 63) (eg: OLED_CMD_SET_DISPLAY_OFFSET + 16)
#define OLED_CMD_SET_X_NORMAL           0xA0
#define OLED_CMD_SET_X_FLIPPED          0xA1
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    
#define OLED_CMD_SET_Y_NORMAL           0xC0    
#define OLED_CMD_SET_Y_FLIPPED          0xC8    
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14 (=ON) or 0x10 (=OFF)

#endif /* MAIN_SSD1366_H_ */