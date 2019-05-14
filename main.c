/*
 * GH_ControlUnit.c
 *
 * Created: 2.5.2019 12:46:51
 * Author : premysl.strakos
 */ 

#define F_CPU	8000000UL
#define ADC2	2
// maximum pumping time limit in seconds
// if max. time reached, pumping stopped and error indicated
#define PUMPING_TIME	240

// Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

// Global variables
uint8_t sensorError = 0;
uint8_t pumpError = 0;
uint8_t pumping = 0;
volatile uint8_t readSensors = 0;
int temperature = 0;

// functions prototypes
void uartTransmit(char msg[15]);
char *numToStr(int number, char id);

// Timer ovf. 32.768ms
ISR(TIMER0_OVF_vect){
	char *msg;
	
	// (1 / F_CPU) * PRESCALLER * 256 * 1000 = time[ms]
	static int pumpingTime = 0;
	static uint8_t scanningTime = 0;
	static int sendAD = 0;
	
	// pumping time counting	
	if (pumping){
		if (pumpingTime < (PUMPING_TIME*30)) pumpingTime++;
		else pumpError = 1;
	}
	else pumpingTime = 0;
	
	// sensors reading interval
	if (scanningTime < 2) scanningTime++;
	else {
		scanningTime = 0;
		readSensors = 1;
	}
	
	// sending temperature once per minute
	if (sendAD < 1831) sendAD++;
	//if (sendAD < 500) sendAD++;
	else{
		sendAD = 0;
		
		msg = numToStr(temperature, 'T');
		uartTransmit (msg); // transmitting
		free(msg); 
	}
}

// Port initialization
void initPort(void){
	// input DDRs
	DDRB &= ~(1 << PB0);	// float sensor - empty
	DDRB &= ~(1 << PB1);	// float sensor - full
	DDRC &= ~(1 << PC2);	// temperature sensor
	//DDRD &= ~(1 << PD0);	// RXD
	DDRD &= ~(1 << PD7);	// light button
		
	// input pull ups
	PORTB |= (1 << PB0);	// pull-up on
	PORTB |= (1 << PB1);	// pull-up on
	PORTC &= ~(1 << PC2);	// pull-up off		
	//PORTD &= ~(1 << PD0);	// pull-up off	
	PORTD |= (1 << PD7);	// pull-up on

	// output DDRs
	DDRD |= (1 << PD5);		// water pump relay
	DDRD |= (1 << PD6);		// LED light transistor
	//DDRD |= (1 << PD1);		// TXD
	DDRC |= (1 << PC0);		// LED diode
	DDRC |= (1 << PC1);		// LED diode

	// output values
	PORTD &= ~(1 << PD5);	// output off
	PORTD &= ~(1 << PD6);	// output off
	//PORTD &= ~(1 << PD1);	// output off
	PORTC &= ~(1 << PC0);	// output off
	PORTC &= ~(1 << PC1);	// output off
}

// Timer initialization
void initTimer(void){
	TCCR0B = (1 << CS00)|(1 << CS02);	// prescaller 1024
	TIMSK0 = (1 << TOIE0);	// TCNT0 overflow interrupt enabled
}

// ADC initialization
void initADC(void){
	// ADC multiplexer selection register
	ADMUX =
	(0 << REFS1)|
	(1 << REFS0)|	// External reference 5V
	(0 << ADLAR)|	// Right shift result
	(0 << MUX3) |
	(0 << MUX2) |
	(0 << MUX1) |
	(0 << MUX0) ;

	// ADC control and status register
	ADCSRA =
	(1 << ADEN)|	// ADC enable
	(0 << ADIE)|	// ADC interrupt disable
	(1 << ADPS2)|	// division factor between XTAL and ADC clock
	(1 << ADPS1)|	// 1/1/1 - 128 -> 62,5 kHZ (50-200 ideal)
	(1 << ADPS0);
}

// ADC reading
int readADC(uint8_t channel){
	ADMUX &= 0xE0;	// clear MUX bits
	ADMUX |= channel & 0x1F;  // set MUX bits

	ADCSRA |= (1 << ADSC);  // start conversion
	while (ADCSRA & (1 << ADSC));  // wait to complete conversion

	return ADC;
}

// USART initialization
void initUart(void)
{
	UBRR0H=0; UBRR0L=51;  // baud rate 9600
	UCSR0B = (0<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);	// RX enabled, TX enabled, interrupt enabled
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	// 8-bit, 1 stop bit, parity none
}

// USART data sending
void uartTransmit(char msg[15])
{
	uint8_t i=0;
	
	while(msg[i]!='\0')
	{
		while (!(UCSR0A & (1<<UDRE0)));
		UDR0 = msg[i];
		i++;
	}
	
	while (!(UCSR0A & (1<<UDRE0))){
	PORTC ^= (1 << PC1);  // green LED on	
		}
		
	UDR0 = 10;  // 13
	
	PORTC ^= (1 << PC1);  // green LED on
	_delay_ms(50);
	PORTC ^= (1 << PC1);  // green LED on
}
	
// Conversion from number to *char[]	
char *numToStr(int number, char id)
{
	char temp[10], *message;
	long rank=1;
	unsigned char i=0, counter=0;
	
	temp[0] = id;
	
	// For zero
	if (number==0)
	{
		temp[1] = '0';
		temp[2] = '\0';
	}
	else
	{
		// For negative numbers
		if (number<0)
		{
			number*=-1;
			temp[1]='-';
			
			while (number>=rank)
			{
				rank *= 10;
				counter++;
			}
			rank/=10;
			
			for (i=1; i<(counter+1); i++)
			{
				temp[i+1] = number/rank + 0x30;
				number -= (number/rank)*rank;
				rank /= 10;
			}
			
			temp[i+1] = '\0';
		}
		// For positive numbers
		else
		{
			while (number>=rank)
			{
				rank *= 10;
				counter++;
			}
			rank/=10;
			
			for (i=0; i<counter; i++)
			{
				temp[i+1] = number/rank + 0x30;
				number -= (number/rank)*rank;
				rank /= 10;
			}
			
			//temp[i] = ' ';
			temp[i+1] = '\0';
		}
	}
	message = (char*)malloc(strlen(temp)+1);
	strcpy(message,temp);
	
	return message;
}	
	
// Main function
int main(void)
{
	uint8_t emptySent = 0, fullSent = 0;
	uint8_t emptySensor = 0;
	uint8_t fullSensor = 0;
	unsigned char last_lightbtn_state=0, actual_lightbtn_state = 2;
	
	initPort();	 // Port initialization
	initTimer();  // Timer initialization
	initADC();  // ADC initialization
	initUart();  // USART initialization
	sei();  // Global interrupt enable
	
	_delay_ms(500);
	
	PORTC |= (1 << PC1);  // green LED on
	
	// infinite main loop
    while (1) 
    {
		// save last state of light button
		last_lightbtn_state = actual_lightbtn_state;
		
		if (readSensors == 1)
		{
			// save water sensors state
			emptySensor = !(PINB&(1 << PB0));
			fullSensor = !(PINB&(1 << PB1));
					
			// read actual state of light button
			actual_lightbtn_state = !(PIND&(1 << PD7));
			
			// read temperature
			temperature = readADC(ADC2);
			
			// reset read sensor flag
			readSensors = 0; 
		}
		
		// not error - normal run
		if (!sensorError && !pumpError)
		{
			 PORTC &= ~(1 << PC0);  // red LED off
			 
			// tank is empty
			if (emptySensor && !fullSensor){
				// filling watter
				pumping = 1;
				PORTD |= (1 << PD5);  // pump relay enable
				if (!emptySent){
					uartTransmit ("START");
					emptySent = 1;
					fullSent = 0;
				}
			}
			// tank is full
			else if (!emptySensor && fullSensor){
				// stop filling
				pumping = 0;
				PORTD &= ~(1 << PD5);  // pump relay disable
				if (!fullSent){
					uartTransmit ("STOP");
					fullSent = 1;
					emptySent = 0;
				}				
			}
			// mismatch error
			else if (emptySensor && fullSensor){
				sensorError = 1;
			}
			// normal run - discharging
			else{

			}		
		}
		else{
			PORTC |= (1 << PC0);  // red LED on
			PORTD &= ~(1 << PD5);  // pump relay disable
			if (!fullSent){
				uartTransmit ("STOP");
				fullSent = 1;
				emptySent = 0;
			}
		}	

		// if light button push down - toggle LED light
		if (actual_lightbtn_state && !last_lightbtn_state)
		{
			PORTD ^= (1 << PD6);	// toggle LED light
		}
				
    }
}

