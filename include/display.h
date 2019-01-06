#ifndef DISPLAY_H
#define DISPLAY_H

#include <wiringPi.h>
#include <stdbool.h>

#define PIN_DATASERIAL	0
#define PIN_SCK 	    1
#define PIN_LATCH       2

extern const char num2code[];
extern const char DOT;

// Interface
extern void display_init();
extern void display_write_idle();
extern void display_write_speed(double speed, bool isexceed);

/**
 * Internal implement
 */
static void lcd_write_byte(char byte);
static void lcd_latch();

#endif // DISPLAY_H