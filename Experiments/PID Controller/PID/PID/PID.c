/*
 * Final_Int_2.c
 *
 * Created: 05-04-2023 12:43:45
 *  Author: Bhaskar
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 14745600UL

#include <math.h> //included to support power function
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
float Setpoint=0;
float error=0;
float kp=4;
float ki=0.26;
float kd=0.6;
float Processedvar =0;
float PID=0;
float integral=0;
float derivative=0;
float lasterror=0;
int kc = 4;
unsigned char max = 0xFF;


void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

// ***************************************************************************************************

float data; //to store received data from UDR1
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x06);
}

void stop (void)
{
	motion_set (0x00);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}
/*
void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}
*/
//Function to initialize ports
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}
/*

void back_white_line (data)
{
	while(data != 0x20) {
			data = UDR0; 				//making copy of data from UDR0 in 'data' variable

			UDR0 = data;
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3

		if(Center_white_line<0x28)	//40
		{
			flag=1;
			back();
			velocity(150,150);
		}

		if((Left_white_line>0x28) && (flag==0))
		{
			flag=1;
			back();
			velocity(50,130);
		}

		if((Right_white_line>0x28) && (flag==0))
		{
			flag=1;
			back();
			velocity(130,50);
		}

		if(Center_white_line>0x28 && Left_white_line>0x28 && Right_white_line>0x28)
		{
			forward();
			velocity(0,0);
		}
	}	
}

*/

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable

	UDR0 = data; 				//echo data back to PC
	
	if(data == 0x31)	//Unicode value of q
	{
		kp += 0.5;
		UDR0 = kp;
	}
	
	if(data == 0x32) //Unicode value of w
	{
		kp-=0.5;
		UDR0 = kp;
	}
	
	if(data == 0x33) //Unicode value of a
	{
		kd+=0.05;
		UDR0 = kd;
	}

	if(data == 0x34) //Unicode value of s
	{
		kd -= 0.05;
		UDR0 = kd;
	}
	
	if(data == 0x35) //Unicode value of e
	{
		ki += 0.01;
		UDR0 = ki;
	}
	
	if(data == 0x36)	//Unicode value of r
	{
		ki -= 0.01;
		UDR0 = ki;
	}
	
	if(data == 0x37)
	{
		kc += 1;
	}
	
}


//Function To Initialize all The Devices
void init_devices()
{
	cli(); //Clears the global interrupts
	port_init();  //Initializes all the ports
	uart0_init(); //Initailize UART1 for serial communiaction
	adc_init();
	lcd_init();
	timer5_init();
	sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	while(1)
	{
		
		
		Left_white_line = max - ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = max - ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = max - ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		Setpoint = 0x28;
		
		
		Processedvar = kc *Center_white_line -Left_white_line + Right_white_line;
		
		if (Processedvar < 0)
		{
			error = Setpoint - Processedvar;
		}
		else
		{
			error = Processedvar - Setpoint;
		}			
		integral += error;
		derivative = -lasterror+error;
		lasterror = error;
		
		
		PID = kp * error + ki * integral + kd * derivative;
		
		if(Left_white_line < 0x28){
		forward();
		velocity(200+PID, 200-PID);
		_delay_ms(10);
		}
		else
		
		{
			forward();
			velocity(200-PID, 200+PID);
			_delay_ms(10);
		}		
	}
}
