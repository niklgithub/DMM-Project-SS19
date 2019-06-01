
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


#include "includes/uart.h"
#include "includes/lcd.h"
#include "includes/twi.h"
#include "includes/dataflash.h"
#include "includes/music.h"

#define wait_joy_button()       {LCD_GotoXY(20,7);  \
	LCD_PutChar(0x10); \
	LCD_Update();      \
while(((PINA)&0x08));while(!((PINA)&0x08));_delay_ms(20);while(((PINA)&0x08)); }	// Required for DMM Board 2013
//while(PINA&0xf8);while(!(PINA&0x08));_delay_ms(20);while(PINA&0x08);}			// Required for DMM Board 2012 and before




const uint8_t PROGMEM emg_logo[640] =
{
	0x80,0xe0,0xf0,0xf8,0xf8,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xf8,0xf8,0xf0,0xe0,0x80,0x00,0x00,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xf0,0xf8,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xf8,0xf8,0xf0,0xe0,0xc0,0xe0,0xf0,0xf8,0xf8,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xf8,0xf8,0xf0,0xe0,0x80,0x00,0x00,0x80,0xe0,0xf0,0xf8,0xf8,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,0xf8,0xf8,0xf0,0xe0,0x80,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf7,0xf3,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf7,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x1f,0x07,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x03,0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x07,0x03,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x03,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x1f,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x03,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf1,0xe1,0xc1,0xc1,0x81,0x81,0x81,0x81,0xc1,0xc1,0xc1,0xc1,0xc1,0xc1,0xe1,0xe1,0xe1,0xe1,0xe1,0xe1,0xe1,0xe1,0xe1,0xe1,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0xe0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xe0,0xf0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0x00,0x01,0x07,0x07,0x0f,0x0f,0x0f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x0f,0x0f,0x0f,0x07,0x07,0x03,0x00,0x00,0x00,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x03,0x07,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x8f,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x0f,0x07,0x01,
};
uint32_t time_sys = 0; //number of milliseconds since µC started
uint32_t time_syslast = 0; //number of milliseconds since µC started
int8_t flag_sensor = 0; //set on bouncing cycle rotation impulse
int8_t flag_turn = 0; //set on debounced cycle rotation impulse

void demo_backlight (void);
void demo_start (void);
void demo_display (void);
void demo_show_emg (void);
void demo_uart (void);
void demo_music (void);


int main (void)
{
	//local variables
	static uint32_t time_lastdet = 0; //time of last sensor detection in ms for debouncing
	static uint8_t time_debounce = 25; //blocking time to prevent bouncing
	
	// set PA3-PA7 as input and activated internal Pull-Up
	DDRA &= ~((1<<PINA3)|(1<<PINA4)|(1<<PINA5)|(1<<PINA6)|(1<<PINA7));		// Required for DMM Board 2013
	PORTA |= ((1<<PINA3)|(1<<PINA4)|(1<<PINA5)|(1<<PINA6)|(1<<PINA7));		// Required for DMM Board 2013
	
	// set PB0-PB3 as output
	DDRB = 0x0F;
	// set PB0-PB3 on high-level
	PORTB |= 0x0F;		// Required for DMM Board DMM Board 2013
	/*
	LCD_Init();
	UART_Init();
	
	demo_start();
	demo_show_emg();

	demo_uart();
	demo_display();
	demo_backlight();
	demo_music();
	
	Backlight_Off();
	LCD_Clear();
	*/
	
	//++++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++
	cli();
	PCICR |= (1<<PCIE0); //enable interrupt on pinchange
	PCMSK0 = (1<<PCINT3); //mask for port pin PA3
	sei();
	init_sysclk();
	
	//++++++++++++++++++++++++++++++++++++ LOOP +++++++++++++++++++++++++++++++++++++
	while(1)
	{
		if (flag_sensor) //debouncing the sensor input
		{
			if (time_sys > (time_lastdet + time_debounce))
			{
				flag_turn = 1;
			}
			
			time_lastdet = time_sys;
			flag_sensor = 0;
		}
		
		/*if(!(time_sys % 1000)) 
		{
			PORTB ^= (1<<PINB0);
		}*/
		
		if(time_sys > 1000)
		{
			time_sys = 0;
			PORTB ^= (1<<PINB0);

		}
		
		if (flag_turn)
		{
			PORTB ^= (1<<PINB3);
			flag_turn = 0;
		}
			
		/*if(PINA&(1<<PINA3))
		{
		PORTB &= ~(1<<PINB0);
		}
		else
		{
		PORTB |= (1<<PINB0);
		}*/
		/*while (time_sys = time_syslast)
		{
		}
		time_syslast = time_sys;*/
	}
}

ISR (PCINT0_vect)
{
	flag_sensor |= !(PINA & (1<<PINA3)); //check whether PA3 is high or low, interrupt is on every pinchange, dont delete
}

ISR (TIMER0_COMPA_vect)
{
	time_sys++; //count ms since µC started
}


void init_sysclk () //sysclk uses Timer0 with output comparision
{
	cli();
	TCCR0B |= ((1<<CS01)|(1<<CS00)); //prescaler 64 (for 16 MHz quarz)
	OCR0A = 250; //compare to this. (250 for 1 ms)
	TIMSK0 |= (1<<OCIE0A); //enable interrupt
	sei();
}

void
demo_start (void)
{
	// Turn on green backlight
	Backlight_LED(BL_GREEN_ON);
	
	// Place program memory strings in framebuffer
	LCD_PutString_P(PSTR("-DMM AVR-Board Demo- \r\n\n"));
	LCD_PutString_P(PSTR("Always push the      \r\n"));
	LCD_PutString_P(PSTR("joystick button to   \r\n"));
	LCD_PutString_P(PSTR("proceed to the next  \r\n"));
	LCD_PutString_P(PSTR("screen.              \r\n\n"));
	LCD_PutString_P(PSTR("Enjoy the demo!      "));
	
	// Copy framebuffer to display
	LCD_Update();

	// Wait for joystick button
	wait_joy_button();
}

void
demo_show_emg (void)
{
	// Some counters
	uint16_t emg_offset = 0;
	uint8_t fb_x = 0;
	uint8_t fb_y = 0;

	// Turn on white backlight (red, green and blue)
	Backlight_LED(BL_RED_ON | BL_GREEN_ON | BL_BLUE_ON);
	
	// Clear precautionally
	LCD_Clear();
	
	// Copy emglogo into framebuffer
	for (fb_y = 0; fb_y < 5; fb_y++)
	for (fb_x = 0; fb_x < 128; fb_x++)
	lcd_framebuffer[fb_y][fb_x] = pgm_read_byte(&emg_logo[emg_offset++]);
	
	// Mark pages 0-4 for update
	lcd_frameupdate = 0x1f;

	// Add some text
	LCD_GotoXY(0,5);
	LCD_PutString_P(PSTR(" Institut f\x81r Messt. \r\n"));
	LCD_PutString_P(PSTR(" und Grundlagen der  \r\n"));
	LCD_PutString_P(PSTR("   Elektrotechnik    \r\n"));

	// And copy framebuffer to display
	LCD_Update();
	
	wait_joy_button();
}


void
demo_uart (void)
{
	unsigned char loop = 1;
	signed int    c = 0;
	char buffer[10];
	
	LCD_Clear();
	Backlight_LED(BL_BLUE_ON);
	
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("UART-Demonstration   \r\n\n"));
	LCD_PutString_P(PSTR("Setup your terminal: \r\n"));
	LCD_PutString_P(PSTR("9600 Baud, 8N1       \r\n"));
	LCD_PutString_P(PSTR("Use joystick to send \r\n"));
	LCD_PutString_P(PSTR("some strings.        \r\n"));
	
	LCD_GotoXY(20,7);
	LCD_PutChar(0x10);
	
	LCD_Update();


	while((!((PINA)&0x08)));
	_delay_ms(20);
	while(!((PINA)&0x08));

	while(loop)
	{
		// Get net character from uart fifo
		c = UART_GetChar();
		
		
		// c = -1 means fifo empty
		if (c > -1)
		{
			// wipe last lcd line
			LCD_WipeLine(7);
			
			LCD_GotoXY(0,7);
			LCD_PutString_P(PSTR("RX-ASCII-Code: "));
			// Convert interger to ascii
			itoa(c, buffer, 10);
			
			LCD_PutString(buffer);
			LCD_GotoXY(20,7);
			LCD_PutChar(0x10);
			LCD_Update();
			_delay_ms(10);
			
			
		}


		_delay_ms(20);
		while(!((PINA)&0xF8));			// Different to DMM 2012 and before
		
		switch(((PINA)&0xF8))
		{
			case 0x78:
			// Joystick Up
			UART_PutString("Joystick Up.\r\n");
			break;
			case 0xB8:
			// Joystick Down
			UART_PutString("Joystick Down.\r\n");
			break;
			case 0xD8:
			// Joystick Left
			UART_PutString("Joystick Left.\r\n");
			break;
			case 0xE8:
			// Joystick Right
			UART_PutString("Joystick Right.\r\n");
			break;
			case 0xF0:
			// Joystick Button
			UART_PutString("Exit.\r\n\n");
			loop = 0;
			break;
		}
		
		
		
		
		_delay_ms(200);
	}
}

void
demo_music (void)
{
	LCD_Clear();
	Backlight_LED(BL_RED_ON);
	
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Let's dance!\r\n\n"));
	LCD_PutString_P(PSTR("(Finest PWM sounds!)\r\n\n"));
	LCD_Update();
	
	Music_PlayTrack(MUSIC_Tetris);
	
	LCD_PutString_P(PSTR("OK, that's enough...\r\n\n"));
	LCD_Update();
	
	wait_joy_button();
}

void
demo_display (void)
{
	// switch white backlight on
	Backlight_LED(BL_RED_ON | BL_GREEN_ON | BL_BLUE_ON);
	
	// clear display
	LCD_Clear();
	
	// talking to the user
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Set some pixels."));

	LCD_DrawPixel(10,10,1);
	LCD_DrawPixel(12,44,1);
	LCD_DrawPixel(111,62,1);
	LCD_DrawPixel(101,22,1);
	LCD_Update();
	
	wait_joy_button();
	
	// one display page requires 8 pages in dataflash
	LCD_SavePage(0);
	LCD_WipeLine(0);
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Connecting."));
	
	LCD_DrawLine(10,10,12,44,1);
	LCD_DrawLine(12,44,111,62,1);
	LCD_DrawLine(111,62,101,22,1);
	LCD_DrawLine(101,22,10,10,1);
	LCD_Update();
	
	wait_joy_button();
	
	// first display page was stored at flash page 0
	// second display page will start at 8
	LCD_SavePage(8);
	LCD_WipeLine(0);
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Adding circles."));
	
	LCD_DrawCircle(64,32,10,1);
	LCD_DrawCircle(70,30,20,1);
	LCD_Update();
	
	wait_joy_button();
	
	// talking to the user
	LCD_SavePage(16);
	LCD_Clear();
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Clear/Set/XOR Demo."));
	
	for (unsigned int fill = 0; fill < 128; fill++)
	LCD_DrawLine(fill, 10, fill, 53, (fill/8)&1);

	// Clear
	LCD_DrawLine(0,20,127,20,0);
	
	// Set
	LCD_DrawLine(0,30,127,30,1);
	
	// XOR
	LCD_DrawLine(0,40,127,40,2);

	LCD_Update();
	
	wait_joy_button();
	
	// talking to the user
	LCD_SavePage(24);
	LCD_Clear();
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Use the joystick to\r\n"));
	LCD_PutString_P(PSTR("recall the last\r\n"));
	LCD_PutString_P(PSTR("screens. Again,\r\n"));
	LCD_PutString_P(PSTR("button means exit.\r\n"));
	LCD_Update();
	
	unsigned char loop = 1;
	
	while(loop)
	{
		// wait
		while((!((PINA)&0xF8)));		// Different to DMM 2012 and before
		_delay_ms(300);
		while(!((PINA)&0xF8));		// Different to DMM 2012 and before
		
		switch((~PINA)&0xF8)			// Different to DMM 2012 and before
		{
			case 0x80:
			// UP
			LCD_LoadPage(0);
			LCD_WipeLine(0);
			LCD_GotoXY(0,0);
			LCD_PutString_P(PSTR("Screen 1"));
			LCD_Update();
			break;
			case 0x40:
			// DOWN
			LCD_LoadPage(24);
			LCD_WipeLine(0);
			LCD_GotoXY(0,0);
			LCD_PutString_P(PSTR("Screen 4"));
			LCD_Update();
			break;
			case 0x20:
			// LEFT
			LCD_LoadPage(8);
			LCD_WipeLine(0);
			LCD_GotoXY(0,0);
			LCD_PutString_P(PSTR("Screen 2"));
			LCD_Update();
			break;
			case 0x10:
			// RIGHT
			LCD_LoadPage(16);
			LCD_WipeLine(0);
			LCD_GotoXY(0,0);
			LCD_PutString_P(PSTR("Screen 3"));
			LCD_Update();
			break;
			case 0x08:
			// BUTTON
			loop = 0;
			break;
		}
	}
	
	// talking to the user
	LCD_Clear();
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Push button for\r\n"));
	LCD_PutString_P(PSTR("next demonstration.\r\n"));

	wait_joy_button();
}

void
demo_backlight (void)
{
	unsigned char led_active = 0x80;
	
	// white backlight
	Backlight_LED(BL_RED_ON | BL_GREEN_ON | BL_BLUE_ON);
	
	LCD_Clear();
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("Use the joystick to\r\n"));
	LCD_PutString_P(PSTR("toggle the backlight\r\n"));
	LCD_PutString_P(PSTR("on the next screen."));
	LCD_Update();
	
	wait_joy_button();

	// no backlight
	Backlight_Off();
	
	LCD_Clear();
	LCD_GotoXY(0,0);
	LCD_PutString_P(PSTR("         Red         \r\n"));
	LCD_PutString_P(PSTR("                     \r\n"));
	LCD_PutString_P(PSTR("          \x18          \r\n"));
	LCD_PutString_P(PSTR(" Green \x1b Exit \x1a Blue \r\n"));
	LCD_PutString_P(PSTR("          \x19          \r\n"));
	LCD_PutString_P(PSTR("                     \r\n"));
	LCD_PutString_P(PSTR("       Toggle        \r\n"));
	LCD_PutString_P(PSTR("         all        \x10\r\n"));
	LCD_Update();
	
	while(!((PINA)&0x08));
	_delay_ms(20);
	while(!((PINA)&0x08));
	
	while(led_active & 0x80)
	{

		while((~(PINA)&0xF8));		// Different to DMM 2012 and before
		_delay_ms(300);
		while(!((PINA)&0xF8));		// Different to DMM 2012 and before
		
		switch((~PINA)&0xF8)			// Different to DMM 2012 and before
		{
			case 0x08:
			// Button -> Exit
			led_active = 0x00;
			
			break;
			case 0x80:
			// Up -> Toggle Red
			led_active ^= 0x01;
			break;
			case 0x20:
			// Up -> Toggle Green
			led_active ^= 0x04;
			break;
			case 0x10:
			// Up -> Toggle Blue
			led_active ^= 0x10;
			break;
			case 0x40:
			// Up -> Toggle All
			led_active ^= 0x15;
			break;
		}

		while(!(PINA & 0xf8));
		
		Backlight_LED(led_active & 0x15);
		
		PORTB &= ~0x0f;
		if (led_active & 0x01)
		PORTB |= 0x01;
		if (led_active & 0x04)
		PORTB |= 0x02;
		if (led_active & 0x10)
		PORTB |= 0x04;
	}
	
	LCD_Clear();
}

