#include "./include/display.h"

/**
 * Hex -> 段码
 * 0 - 9
 */
const char num2code[] = {
	0x3f,
	0x06,
	0x5b,
	0x4f,
	0x66,
	0x6d,
	0x7d,
	0x07,
	0x7F,
	0x6F,
};

/**
 * 小数点
 */
const char DOT = 0x80;

void display_init() {
    wiringPiSetup();
    pinMode(PIN_DATASERIAL, OUTPUT);   // PIN17
    pinMode(PIN_SCK, OUTPUT);          // PIN18
    pinMode(PIN_LATCH, OUTPUT);        // PIN27

    digitalWrite(PIN_SCK, LOW);
    digitalWrite(PIN_LATCH, LOW);

    // 默认空闲状态
    display_write_idle();
}

/**
 * 空闲状态（省电）
 */
void display_write_idle() {
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_latch();  
}

/**
 * 使能显示屏显示速度以及超速信息
 * @param speed 速率
 * @param isexceed 是否超速
 */
void display_write_speed(double speed, bool isexceed) {
    // TODO: isexceed 未使用
    if (speed < 10) {
        /**
         * 显示小数点
         */
        int a = (int)(speed*10) % 10;
        int b = (int)speed % 10;
        lcd_write_byte(num2code[a]);
        lcd_write_byte(num2code[b] | DOT);
    } else {
        lcd_write_byte(num2code[(int)speed % 10]);
        lcd_write_byte(num2code[(int)speed / 10 ]);
    }
    lcd_latch();
}

void lcd_write_byte(char byte) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(PIN_DATASERIAL, byte >> 7);
        byte <<= 1;
        digitalWrite(PIN_SCK, HIGH);
        asm("nop");
        asm("nop");
        digitalWrite(PIN_SCK, LOW);
    }
}

void lcd_latch() {
    digitalWrite(PIN_LATCH, HIGH);
    asm("nop");
    asm("nop");
    digitalWrite(PIN_LATCH, LOW);	
}

