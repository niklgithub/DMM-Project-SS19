
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
uint32_t time_sensorlastnegedge = 0; //Sensing wont work within first time_debounce ms
uint32_t time_lasthigh = 0;
uint16_t time_debounce = 50;
int8_t flag_pcint = 0; //set on bouncing cycle rotation impulse
int8_t flag_turn = 0; //set on debounced cycle rotation impulse
int8_t lock_debounce = 0; //additional locking to debounce until time_sensorlastnegedge is updated
int8_t flag_time = 0; //to count time_sys in main function
uint16_t size_wheel = 2215; //wheels circumference in mm
uint32_t time_measarray[5] = {0};
uint8_t flag_measarray[5] = {0};
uint8_t speed_threshold = 1; // in m/s. Lower speeds ain't used for calculations
uint8_t speed_table[250] = {0}; //8-bit representation: 0 -> 0 km/h; 255 -> (255*0.2) = 51 km/h
uint8_t power_table[250] = {0}; //8-bit representation: 0 -> 0 W; 255 -> (255*2) = 510 W
double speed_current = 0; //speed from time 2 samples before
double acc_current = 0; //acceleration from time 2 samples before
double power_current = 0; //power in W from time 2 samples before
float mass_eff = 80; //total effective mass in kg: bike + rider + transformed inertia torque 


void demo_backlight (void);
void demo_start (void);
void demo_display (void);
void demo_show_emg (void);
void demo_uart (void);
void demo_music (void);


int main (void)
{
	//local variables
	uint32_t time_syssec = 0;	
	int8_t state_sensor = -1; 
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
	init_pcint();
	init_sysclk();
	//++++++++++++++++++++++++++++++++++++ LOOP +++++++++++++++++++++++++++++++++++++
	while(1)
	{
		if((time_sys - (1000*time_syssec))>= 1000) 
		{
			time_syssec++;
			PORTB ^= (1<<PINB0);
		}
		
		
		state_sensor = !(PINA & (1<<PINA3)); //inverted because of pull-up on PA3. state_sensor == 1 means magnet is near reed contact
		if(flag_pcint || (state_sensor == 1))
		{
			if(time_sys > time_lasthigh + time_debounce)
			{
				flag_turn = 1;
			}
			time_lasthigh = time_sys;
			flag_pcint = 0;
		}
		
		
		if (flag_turn)
		{
			PORTB ^= (1<<PINB3);
			
			time_measarray[0] = time_measarray[1];
			time_measarray[1] = time_measarray[2];
			time_measarray[2] = time_measarray[3];
			time_measarray[3] = time_measarray[4];
			time_measarray[4] = time_sys;
				
			flag_measarray[0] = 1;	
			flag_measarray[1] = flag_measarray[2];
			flag_measarray[2] = flag_measarray[3];
			flag_measarray[3] = flag_measarray[4];
			flag_measarray[4] = (((double) (size_wheel/(time_measarray[4]-time_measarray[3]))) >= speed_threshold)
			
			if (flag_measarray[0] && flag_measarray[1] && flag_measarray[2] && flag_measarray[3] && flag_measarray[4])
			{
				//differentiation based on method of central difference
				speed_current = 2*size_wheel/(time_measarray[3]-time_measarray[1]); //speed in mm/ms = m/s
				acc_current = 2*1000*size_wheel*(((1/(time_measarray[4]-time_measarray[2]))-(1/(time_measarray[2]-time_measarray[0])))/(time_measarray[3]-time_measarray[1])); //acceleration in 1000mm/(ms)²=m/s²
				power_current = (-1) * mass_eff * speed_current * acc_current; //power in W
			}
			
			flag_turn = 0;
		}
		
		/*if(time_sys > (time_debounce + time_sensorlastnegedge))
		{
			PORTB |= (1<<PINB2);
		}
		else
		{
			PORTB &= ~(1<<PINB2);
		}*/
			
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
		
		
		/*Dont write main-code after following block!*/
		do //to realise 1 ms system period
		{
		} while (flag_time==0);
		time_sys += flag_time; // checking if flag_time > 1 would mean main takes too much time
		flag_time = 0;
	}
}

ISR (PCINT0_vect)
{
	flag_pcint = 1;
	/*if((PINA & (1<<PINA3))) //Magnet has left reed-contact
	{
		time_sensorlastnegedge = time_sys;
		lock_debounce = 0;
	}
	else if (!(PINA & (1<<PINA3))) //Magnet has moved into range of reed-contact
	{
		if ((time_sys > (time_sensorlastnegedge + time_debounce)) && (lock_debounce == 0)) //debouncing filter
		{
			flag_turn = 1;
			lock_debounce = 1;
		}
	}*/
}

ISR (TIMER0_COMPA_vect)
{
	//time_sys++; //count ms since µC started
	flag_time++;
}

void init_pcint (void)
{
	cli();
	PCICR |= (1<<PCIE0); //enable interrupt on pinchange
	PCMSK0 = (1<<PCINT3); //mask for port pin PA3
	sei();
}
void init_sysclk (void) //sysclk uses Timer0 with output comparision
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

