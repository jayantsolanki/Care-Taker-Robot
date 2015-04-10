/************************************************************************************
Written by: Devendra Jangir
Edited by: e-Yantra team
AVR Studio Version 6

Date: 3 March 2015

 Application example: Robot control over serial port via XBee wireless communication module 
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication
 
 Serial Port used: UART0

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:  	
 						
  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.


 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 						options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same. 

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/



#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

unsigned char data; //to store received data from UDR1
void velocity(unsigned char, unsigned char);
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
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}
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

void led_pin_config (void)
{
	DDRH = DDRH | 0x70;		//Setting PORTH 4,5,6 as output
	DDRL = DDRL | 0xC0;     //Setting PORTL as output
}
//LED


void led1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void led2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void led3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//TIMER1 initialization in 8 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 8bit fast, TOP=0x00FF
// actual value: 52.25Hz 
void timer1_init()
{
	TCCR1B = 0x00;	//Stop
	TCNT1H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT1L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR1AH = 0x00;	//Output compare register high value for Left Motor
	OCR1AL = 0xFF;	//Output compare register low value for Left Motor
	OCR1BH = 0x00;	//Output compare register high value for Right Motor
	OCR1BL = 0xFF;	//Output compare register low value for Right Motor
	OCR1CH = 0x00;	//Output compare register high value for Motor C1
	OCR1CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR1A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR1B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


//Function to initialize all the peripherals


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void led_control(unsigned char value1,unsigned char value2,unsigned char value3)  
{
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) value1;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) value2;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) value3;
 }
 
 void red_led1_on(void)
 {
   led_control(240,0,0);
   _delay_ms(500);
 } 
 
 void green_led1_on(void)
 {
	 led_control(0,240,0);
	 _delay_ms(500);
}
 
 void blue_led1_on(void)
 {
	 led_control(0,0,240);
	 _delay_ms(500);
 }
 
 void yellow_led1_on(void)
 {
	 led_control(255,255,0);
	 _delay_ms(500);
 }
 void led1_off(void)
 {
 led_control(0,0,0);
   _delay_ms(500);
 } 
void red_led2_on (void)
{
	
	PORTL = 0x40;     //to turn on LED
}
void green_led2_on (void)
{
	
	PORTL = 0x80;     //to turn on LED
}
void blue_led2_on (void)
{
	
	PORTH = 0x40;     //to turn on LED
}
void yellow_led2_on (void)
{
	
	PORTL = 0xC0;     //to turn on LED
}

void led2_off (void)
{
	
	PORTH = 0x00;     //to turn on LED
	PORTL = 0x00;
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void fast_left(void)
{
	motion_set(0x05);
}

void fast_right(void)
{
	
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}


void stop (void) //hard stop
{
	motion_set(0x00);
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	buzzer_pin_config();
	led_pin_config();
	led1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	led2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	led3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
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
// UBRR0L = 0x47; //11059200 Hz
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 
	
	UDR0 = data; 				//echo data back to PC

		if(data == 0x38) //ASCII value of 8
		{
			
			forward();  //forward
			velocity(255,255);
		}

		if(data == 0x32) //ASCII value of 2
		{
			back(); //back
			//velocity(150,150);
		}

		if(data == 0x34) //ASCII value of 4
		{
			left();  //left
			velocity(170,170);
		}

		if(data == 0x36) //ASCII value of 6
		{
			right(); //right
			velocity(170,170);
		}

		if(data == 0x35) //ASCII value of 5
		{
			PORTA=0x00; //stop
		}

		if(data == 0x37) //ASCII value of 7
		{
			buzzer_on();
		}

		if(data == 0x39) //ASCII value of 9
		{
			buzzer_off();
		}
		
		if(data == 0x52) // ASCII value of R
		{
			red_led1_on();
		}
		
		if (data == 0x72) //ASCII value of r
		{
			red_led2_on();
		}
		
		if (data == 0x42) // ASCII value of B
		{
			blue_led1_on();
		}
		
		if (data == 0x62) // ASCII value of b
		{
			blue_led2_on();
		}
		
		if (data == 0x59) //ASCII value of Y
		{
			
			yellow_led1_on();
		}			
		
		if (data == 0x79) //ASCII value of y
		{
			yellow_led2_on();
		}
		
		if (data == 0x47)	//ASCII value of G
		{
			
			green_led1_on();
		}
		
		if (data == 0x67) //ASCII value of g
		{
			green_led2_on();
		}
		
		if (data== 0x4F) //ASCII value of O
		{
			 led1_off();
		}
					
		if (data== 0x6F) //ASCII value of o
		{
			led2_off();
		}
		if (data == 0x41) // ASCII value of A
		{
			fast_left();
			velocity(255,255);
		}
		
		if (data == 0x44)// ASCII value of D
		{
			fast_right();
			velocity(255,255);
		}
}


//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART1 for serial communiaction
 timer5_init();
 timer1_init();
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	led1_off();
	led2_off();
	while(1);
}

