/*
 * Proyecto2.c
 *
 * Created: 28/04/2025 21:32:39
 * Author : axell
 */ 

/********************************************************************/
/*--------------------- Header (Libraries)--------------------------*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/********************************************************************/



/********************************************************************/
/*-----------------------Function Prototypes------------------------*/
	// Setup
void setup();
void configMODE();
void configUART();
void configADC();
void configChannel(uint8_t canal);
	// UART
void writeChar(char caracter);
void writeString(char* texto);
	// EEPROM
void writeEEPROM(uint8_t dato, uint16_t direccion);
uint8_t readEEPROM(uint16_t direccion);
	
void initPWMFast1A(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle1A(uint16_t dutycycle);
	
void initPWMFast1B(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle1B(uint16_t dutycycle);
	// Servo 3
void initPWMFast0A(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle0A(uint16_t dutycycle);
	// Servo 4
void initPWMFast0B(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle0B(uint16_t dutycycle);
	// Servo 1
void initPWMFast2A(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle2A(uint16_t dutycycle);
	// Servo 2
void initPWMFast2B(uint8_t invertido, uint16_t prescaler);
void updateDutyCycle2B(uint16_t dutycycle);

	// Main program
void program();

/********************************************************************/



/********************************************************************/
/*-----------------------Variable Declaration-----------------------*/
#define invert 1
#define non_invert 0

// Change of mode variables
uint8_t state = 0; // Controls the actual state
uint8_t pcintFlag = 0; // Flag that indicates a pin change interruption
uint8_t rxintFlag = 0; // Flag that indicates a UART receive interruption

// ADC variables
uint8_t channel = 0; // Channel control
uint16_t adcval0 = 0; // Stores ADC0 value
uint16_t adcval1 = 0; // Stores ADC1 value
uint16_t adcval2 = 0; // Stores ADC2 value
uint16_t adcval3 = 0; // Stores ADC3 value

// UART variables
char caracterIN; // Stores the received character

// PCInt variables
uint8_t buttonstate0 = 0; // PINB state at first press
uint8_t buttonstate1 = 0; // PINB state after 200 ms (anti bounce)
uint8_t buttonstate2 = 0; // PIND state at first press
uint8_t buttonstate3 = 0; // PIND state after 200 ms (anti bounce)

// EEPROM variables
uint8_t servo1 = 0; // Stores the OCRXn of the servo 1
uint8_t servo2 = 0; // Stores the OCRXn of the servo 2
uint8_t servo3 = 0; // Stores the OCRXn of the servo 3
uint8_t servo4 = 0; // Stores the OCRXn of the servo 4
uint8_t stateEEPROM = 0; // There will be four positions for the arm (4 x 4 = 16 EEPROM directions)
uint8_t eeprom_address = 0x0000; // First position of EEPROM

/********************************************************************/



/********************************************************************/
/*------------------------------Main--------------------------------*/
int main(void)
{
	// Go to uC configuration
	setup();
	// Print welcome message
	//writeString("Welcome User\n");
	while (1)
	{
		program();
	}
}

/********************************************************************/



/********************************************************************/
/*---------------------Non-interrupt Subroutines--------------------*/
void setup()
{
	// Disable global interruptions
	cli();
	// Enable PreScaler change
	// Set PreScaler = 1 --> F_CPU = 16MHz
	CLKPR = (1 << CLKPCE);
	// Configure change of mode
	configMODE();
	// Start the UART communication
	configUART();
	// Start ADC configuration
	configADC();
	// Setup the PWM2 channels
	initPWMFast2A(non_invert, 1024);
	initPWMFast2B(non_invert, 1024);
	// Setup the PWM0 channels
	initPWMFast0A(non_invert, 1024);
	initPWMFast0B(non_invert, 1024);
	// Enable global interruptions
	sei();
}

void configMODE()
{
	// Define state led indicators as OUT
	DDRD |= (1 << DDD4) | (1 << DDD2);
	DDRB |= (1 << DDB4);
	
	// Define pushbutton: change of state as IN
	DDRB &= ~(1 << DDB0);
	DDRB &= ~(1 << DDB1);
	DDRB &= ~(1 << DDB2);
	DDRD &= ~(1 << DDD7);
	// Enable pull up
	PORTB |= (1 << PORTB0);
	PORTB |= (1 << PORTB1);
	PORTB |= (1 << PORTB2);
	PORTD |= (1 << PORTD7);
	
	// Pin change interruption enable
	PCICR = (1 << PCIE0);
	PCICR |= (1 << PCIE2);
	// Enable pin change interruptions
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2);
	PCMSK2 |= (1 << PCINT23);
	
	// Start in manual mode
	state = 1;
	PORTD |= (1 << PORTD2);
	// Start in first EEPROM state
	stateEEPROM = 1;
	
	// EEPROM state led
	DDRB |= (1 << DDB5);
	DDRC |= (1 << DDC4) | (1 << DDC5);
	// Turn them off except
	PORTB &= ~(1 << PORTB5);
	PORTC &= ~(1 << PORTC4);
	PORTC &= ~(1 << PORTC5);
}

	// UART
void configUART() // Desired mode: asynchronous normal, 9600 baud rate, 8 bit payload, 1 stop bit, no parity
{
	// Define TX (PD0) as OUT
	DDRD |= (1 << DDD1);
	// Define RX (PD1) as IN
	DDRD &= ~(1 << DDD0);
	// Configure UCSR0A
	// Set flags as zero, register works as indicator, no setup is needed unless double speed is desired
	UCSR0A = 0x00;
	// Configure UCSR0B
	// Enable transmission TX, enable reception RX, and enable RX interrupt
	UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0);
	// Configure UCSR0C
	// Define Asynchronous mode
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	// Configure baud rate
	UBRR0 = 103;
}

void writeChar(char caracter)
{
	while((UCSR0A & (1 << UDRE0)) == 0); // Repeat while loop while UDR0 is not empty (when empty UDRE0 is 1)
	UDR0 = caracter;
}

void writeString(char* texto)
{
	for (uint16_t i = 0; *(texto + i) != '\0'; i++) // write characters until the string is empty
	{
		writeChar(*(texto + i)); // text direction with pointer
	}
}

	// ADC
void configADC()
{
	// Begin ADC Multiplexer
	ADMUX = 0x00;
	// AVcc
	ADMUX |= (1	<< REFS0);
	// Activate justification (Right)
	ADMUX &= ~(1 << ADLAR);
	// Select channel
	configChannel(channel);
	// Begin ADC Control & Status Register
	ADCSRA = 0x00;
	// Enable interruptions
	ADCSRA |= (1 << ADIE);
	// PreScaler = 128 --> f_adc = 125kHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// ADC enable
	ADCSRA |= (1 << ADEN);
	// Start first conversion
	ADCSRA |= (1 << ADSC);
}
	
void configChannel(uint8_t canal)
{
	switch(canal)
	{
		case 0:
			ADMUX &= ~( (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0) ); // (MUX0123 = 0000)
			break;
		case 1:
			ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
			ADMUX |=  (1 << MUX0) ; // (MUX0123 = 0001)
			break;
		case 2:
			ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
			ADMUX |= (1 << MUX1); // (MUX0123 = 0010)
			break;
		case 3:
			ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
			ADMUX |= (1 << MUX1) | (1 << MUX0); // (MUX0123 = 0011)
			break;
		default:
			ADMUX &= ~( (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0) ); // Channel 0
			break;
	}
}
	
	// PWM 1
void initPWMFast1A(uint8_t invertido, uint16_t prescaler)
{
	// Define pin
	DDRB |= (1<<DDB1);
	// Define top value
	ICR1 = 39999; // ---------------------------------- CAMBIAR -----------------------------------------
	// Mode options
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
	if (invertido == invert)
	{
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0); // Inverted
	}
	else
	{
		TCCR1A &= ~(1 << COM1A0);
		TCCR1A |= (1 << COM1A1); // Non-inverted
	}
	// Fast PWM mode 14
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	TCCR1A |= (1 << WGM11);
	TCCR1A &= ~(1 << WGM10);
	// PreScaler options
	switch(prescaler)
	{
		case 1:
		TCCR1B |= (1 << CS10);
		break;
		case 8:
		TCCR1B |= (1 << CS11);
		break;
		case 64:
		TCCR1B |= (1 << CS11) | (1 << CS10);
		break;
		case 256:
		TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
		break;
		case 1024:
		TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
		break;
		default:
		TCCR1B |= (1 << CS10);
		break;
	}
}

void updateDutyCycle1A(uint16_t dutycycle){
	OCR1A = dutycycle;
}

void initPWMFast1B(uint8_t invertido, uint16_t prescaler)
{
	// Define pin
	DDRB |= (1<<DDB2);
	// Define top value
	ICR1 = 39999;
	// Mode options
	TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
	if (invertido == invert)
	{
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0); // Inverted
	}
	else
	{
		TCCR1A &= ~(1 << COM1B0);
		TCCR1A |= (1 << COM1B1); // Non-inverted
	}
	// Fast PWM mode 14
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	TCCR1A |= (1 << WGM11);
	TCCR1A &= ~(1 << WGM10);
	// PreScaler options
	switch(prescaler)
	{
		case 1:
		TCCR1B |= (1 << CS10);
		break;
		case 8:
		TCCR1B |= (1 << CS11);
		break;
		case 64:
		TCCR1B |= (1 << CS11) | (1 << CS10);
		break;
		case 256:
		TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
		break;
		case 1024:
		TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
		break;
		default:
		TCCR1B |= (1 << CS10);
		break;
	}
}
	
void updateDutyCycle1B(uint16_t dutycycle){
	OCR1B = dutycycle;
}
	
	// PWM 0
void initPWMFast0A(uint8_t invertido, uint16_t prescaler){
	// Define pin
	DDRD |= (1<<DDD6);
	// Mode options
	TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
	if (invertido == invert)
	{
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0); // Inverted
	}
	else
	{
		TCCR0A |= (1 << COM0A1); // Non-inverted
	}
	// Fast PWM mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// PreScaler options
	TCCR0B = 0;
	switch(prescaler){
		case 1:
		TCCR0B |= (1 << CS00);
		break;
		case 8:
		TCCR0B |= (1 << CS01);
		break;
		case 64:
		TCCR0B |= (1 << CS01) | (1 << CS00);
		break;
		case 256:
		TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00);
		break;
		case 1024:
		//TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
		TCCR0B |= (1 << CS02) | (1 << CS00);
		TCCR0B &= ~(1 << CS01);
		break;
		default:
		TCCR0B |= (1 << CS00);
		break;
	}
	
}

void updateDutyCycle0A(uint16_t dutycycle){
	OCR0A = dutycycle;
}

void initPWMFast0B(uint8_t invertido, uint16_t prescaler){
	// Define pin
	DDRD |= (1<<DDD5);
	// Mode options
	TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
	if (invertido == invert)
	{
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0); // Inverted
	}
	else
	{
		TCCR0A |= (1 << COM0B1); // Non-inverted
	}
	// Fast PWM mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// PreScaler options
	TCCR0B = 0;
	switch(prescaler){
		case 1:
		TCCR0B |= (1 << CS00);
		break;
		case 8:
		TCCR0B |= (1 << CS01);
		break;
		case 64:
		TCCR0B |= (1 << CS01) | (1 << CS00);
		break;
		case 256:
		TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00);
		break;
		case 1024:
		TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
		break;
		default:
		TCCR0B |= (1 << CS00);
		break;
	}
	
}

void updateDutyCycle0B(uint16_t dutycycle){
	OCR0B = dutycycle;
}
	
	// PWM 2
void initPWMFast2A(uint8_t invertido, uint16_t prescaler){
	// Define pin
	DDRB |= (1<<DDB3);
	// Mode options
	TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0));
	if (invertido == invert)
	{
		TCCR2A |= (1 << COM2A1) | (1 << COM2A0); // Inverted
	}
	else
	{
		TCCR2A |= (1 << COM2A1); // Non-inverted
	}
	// Fast PWM mode
	TCCR2A |= (1 << WGM21) | (1 << WGM20);
	// PreScaler options
	TCCR2B = 0;
	switch(prescaler){
		case 1:
		TCCR2B |= (1 << CS20);
		break;
		case 8:
		TCCR2B |= (1 << CS21);
		break;
		case 64:
		TCCR2B |= (1 << CS21) | (1 << CS20);
		break;
		case 256:
		TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
		break;
		case 1024:
		TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
		break;
		default:
		TCCR2B |= (1 << CS20);
		break;
	}
	
}

void updateDutyCycle2A(uint16_t dutycycle){
	OCR2A = dutycycle;
}

void initPWMFast2B(uint8_t invertido, uint16_t prescaler){
	// Define pin
	DDRD |= (1<<DDD3);
	// Mode options
	TCCR2A &= ~((1 << COM2B1) | (1 << COM2B0));
	if (invertido == invert)
	{
		TCCR2A |= (1 << COM2B1) | (1 << COM2B0); // Inverted
	}
	else
	{
		TCCR2A |= (1 << COM2B1); // Non-inverted
	}
	// Fast PWM mode
	TCCR2A |= (1 << WGM21) | (1 << WGM20);
	// PreScaler options
	TCCR2B = 0;
	switch(prescaler){
		case 1:
		TCCR2B |= (1 << CS20);
		break;
		case 8:
		TCCR2B |= (1 << CS21);
		break;
		case 64:
		TCCR2B |= (1 << CS21) | (1 << CS20);
		break;
		case 256:
		TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
		break;
		case 1024:
		TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
		break;
		default:
		TCCR2B |= (1 << CS20);
		break;
	}
	
}

void updateDutyCycle2B(uint16_t dutycycle){
	OCR2B = dutycycle;
}

 // EEPROM
 void writeEEPROM(uint8_t dato, uint16_t direccion)
 {
	 // Disable global interruption
	 cli();
	 // Erase and write mode
	 while(EECR & (1 << EEPE)); // Wait until bit EEPE becomes zero
	 // Establish direction
	 EEAR = direccion;
	 // Establish payload
	 EEDR = dato;
	 // Master write enable
	 EECR |= (1 << EEMPE);
	 // Write enable
	 EECR |= (1 << EEPE);
	 // Enable global interruption
	 sei();
 }

 uint8_t readEEPROM(uint16_t direccion)
 {
	 // Erase and write mode
	 while(EECR & (1 << EEPE)); // Wait until bit EEPE becomes zero
	 // Establish direction
	 EEAR = direccion;
	 // Read the payload
	 EECR |= (1 << EERE);
	 // Return the data
	 return EEDR;
 }


	// Main program
void program()
{
	// De bounce
	_delay_ms(150);
	// Save new state
	buttonstate1 = PINB;
	buttonstate3 = PIND;
	// Change of state with pushbutton
	if (buttonstate0 == buttonstate1)
	{
		if (!(PINB & (1 << PINB0))) // mask: 0bXXXX XXXX & 0b0000 0001 = 0b0000 000X
		{
			// Change the current state
			state++;
			// Reset the state when count is finished
			if(state >= 4) state = 1;
			
			// Restore ADC
			if(state == 1)
			{
				ADCSRA |= (1 << ADEN);
				ADCSRA |= (1 << ADSC);
			}
			
		}
		else if ( (!(PINB & (1 << PINB1))) && state == 3)
		{
			// Increment by one
			stateEEPROM++;
			// If state 4 is surpassed, return to 1
			if(stateEEPROM >= 5) stateEEPROM = 1;
		}
		else if ( (!(PINB & (1 << PINB2))) && state == 3)
		{	
			// Store in EEPROM first position
			writeEEPROM(servo1, (eeprom_address + 0) + 4*(stateEEPROM - 1) );
			// Write position in monitor
			writeString("1. Stored: ");
			writeChar(readEEPROM( (eeprom_address + 0) + 4*(stateEEPROM - 1) ));
			writeChar('\n');
			
			// Store in EEPROM second position
			writeEEPROM(servo2, (eeprom_address + 1) + 4*(stateEEPROM - 1) );
			// Write position in monitor
			writeString("2. Stored: ");
			writeChar(readEEPROM((eeprom_address + 1) + 4*(stateEEPROM - 1) ));
			writeChar('\n');
			
			// Store in EEPROM third position
			writeEEPROM(servo3, (eeprom_address + 2) + 4*(stateEEPROM - 1) );
			// Write position in monitor
			writeString("3. Stored: ");
			writeChar(readEEPROM((eeprom_address + 2) + 4*(stateEEPROM - 1) ));
			writeChar('\n');
			
			// Store in EEPROM third position
			writeEEPROM(servo4, (eeprom_address + 3) + 4*(stateEEPROM - 1) );
			// Write position in monitor
			writeString("4. Stored: ");
			writeChar(readEEPROM((eeprom_address + 3) + 4*(stateEEPROM - 1) ));
			writeChar('\n');
			
		}
		pcintFlag = 0;
		buttonstate1 = 0;
	}
	
	// Pin change D
	if ((buttonstate2 == buttonstate3) && state == 3)
	{
		if ( !(PIND & (1 << PIND7)) ) // mask: 0bXXXX XXXX & 0b0000 0001 = 0b0000 000X
		{
			// ADC disable
			ADCSRA &= ~(1 << ADEN);
			// Replicate position
			switch(stateEEPROM)
			{
				case 1:
				updateDutyCycle2A(readEEPROM(eeprom_address + 0));
				updateDutyCycle2B(readEEPROM(eeprom_address + 1));
				updateDutyCycle0A(readEEPROM(eeprom_address + 2));
				updateDutyCycle0B(readEEPROM(eeprom_address + 3));
				break;
				
				case 2:
				updateDutyCycle2A(readEEPROM(eeprom_address + 4));
				updateDutyCycle2B(readEEPROM(eeprom_address + 5));
				updateDutyCycle0A(readEEPROM(eeprom_address + 6));
				updateDutyCycle0B(readEEPROM(eeprom_address + 7));
				break;
				
				case 3:
				updateDutyCycle2A(readEEPROM(eeprom_address + 8));
				updateDutyCycle2B(readEEPROM(eeprom_address + 9));
				updateDutyCycle0A(readEEPROM(eeprom_address + 10));
				updateDutyCycle0B(readEEPROM(eeprom_address + 11));
				break;
				
				case 4:
				updateDutyCycle2A(readEEPROM(eeprom_address + 12));
				updateDutyCycle2B(readEEPROM(eeprom_address + 13));
				updateDutyCycle0A(readEEPROM(eeprom_address + 14));
				updateDutyCycle0B(readEEPROM(eeprom_address + 15));
				break;
			}
		}
		buttonstate3 = 0;
	}
	
	// Main program
	switch(state)
	{
		case 1: // Manual
			// Led sequence
			PORTD |= (1 << PORTD2);
			PORTB &= ~(1 << PORTB4);
			PORTD &= ~(1 << PORTD4);
			// Shutdown EEPROM states
			PORTB &= ~(1 << PORTB5);
			PORTC &= ~(1 << PORTC4);
			PORTC &= ~(1 << PORTC5);
			break;
			
		case 2: // Serial
			// Led sequence
			PORTD |= (1 << PORTD4);
			PORTB &= ~(1 << PORTB4);
			PORTD &= ~(1 << PORTD2);
			// Shutdown EEPROM states
			PORTB &= ~(1 << PORTB5);
			PORTC &= ~(1 << PORTC4);
			PORTC &= ~(1 << PORTC5);
			break;
			
		case 3: // EEPROM
			// Led sequence
			PORTB |= (1 << PORTB4);
			PORTD &= ~(1 << PORTD2);
			PORTD &= ~(1 << PORTD4);
			// Introduce EEPROM case states
			switch(stateEEPROM)
			{
				case 1:
				PORTB |= (1 << PORTB5);
				PORTC &= ~(1 << PORTC4);
				PORTC &= ~(1 << PORTC5);
				break;
				
				case 2:
				PORTB &= ~(1 << PORTB5);
				PORTC |= (1 << PORTC4);
				PORTC &= ~(1 << PORTC5);
				break;
				
				case 3:
				PORTB |= (1 << PORTB5);
				PORTC |= (1 << PORTC4);
				PORTC &= ~(1 << PORTC5);
				break;
				
				case 4:
				PORTB &= ~(1 << PORTB5);
				PORTC &= ~(1 << PORTC4);
				PORTC |= (1 << PORTC5);
				break;
			}
			
			break;
	}
	
}



/********************************************************************/



/********************************************************************/
/*----------------------Interrupt Subroutines-----------------------*/
ISR(ADC_vect)
{
	// Multiplexing of channels sequence
	if (channel == 0)
	{
		// Servo 1
		adcval0 = ADC;
		if (state == 1 || state == 3)
		{
			servo1 = adcval0 * 0.03 + 8;
			updateDutyCycle2A(adcval0 * 0.03 + 8);
		}
		channel = 1;
	}
	else if (channel == 1)
	{
		// Servo 2
		adcval1 = ADC;
		if (state == 1 || state == 3)
		{
			servo2 = adcval1 * 0.03 + 8;
			updateDutyCycle2B(adcval1 * 0.03 + 8);
		}
		channel = 2;
	}
	else if (channel == 2)
	{
		// Servo 3
		adcval2 = ADC;
		if (state == 1 || state == 3)
		{
			servo3 = adcval2 * 0.03 + 8;
			updateDutyCycle0A(adcval2 * 0.03 + 8);
		}
		
		channel = 3;
	}
	else if (channel == 3)
	{
		// Servo 4
		adcval3 = ADC;
		if (state == 1 || state == 3)
		{
			servo4 = adcval3 * 0.03 + 8;
			updateDutyCycle0B(adcval3 * 0.03 + 8);
		}
		channel = 0;
	}

	// Configure next channel and start conversion
	configChannel(channel);
	ADCSRA |= (1 << ADSC);
	
	// Convert the ADC values into angles from 0 to 180
	/*uint16_t angle1 = (adcval0 * 180) / 1023;
	uint16_t angle2 = (adcval1 * 180) / 1023;
	uint16_t angle3 = (adcval2 * 180) / 1023;
	uint16_t angle4 = (adcval3 * 180) / 1023;
	
	writeChar(angle1);
	writeChar(',');	
	writeChar(angle2);
	writeChar(',');
	writeChar(angle3);
	writeChar(',');
	writeChar(angle4);
	writeChar('\n');*/
}

ISR(USART_RX_vect)
{
	// Set the variable for further control
	rxintFlag = 1;
	caracterIN = UDR0;
}

ISR(PCINT0_vect)
{
	// Set the variable for further control
	pcintFlag = 1;
	buttonstate0 = PINB;
}

ISR(PCINT2_vect)
{
	buttonstate2 = PIND;
}
/********************************************************************/
