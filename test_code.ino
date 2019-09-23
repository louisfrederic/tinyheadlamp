#include "OneButton.h"
#include <avr/sleep.h> 							// Sleep Modes
#include <avr/power.h> 							// Power management
#include <avr/wdt.h>							// Watchdog Timer

#define mid 4000								// BAttery Level Mid
#define low 3200								// Battery Level low, flash and start dimming the Led
#define crit 2600								// Battery Level critical, shut down


const byte LEDr = 1; 							// pin 1
const byte LEDw = 4; 							// pin 4
const byte ModeNr = 3;
volatile bool WDR_flag = false;

OneButton button1(2, true);
int count = 1;
int flag = 0;
int Vcc;
int PWMr = 255;
int PWMw_l = 50;
int PWMw_h = 255;

ISR(PCINT0_vect)
{
    // do something interesting here
}

ISR(WDT_vect){
	WDR_flag=true;
 }

void setup()
{
    pinMode(LEDr, OUTPUT); 						// turn the LED on (HIGH is the voltage level)
    pinMode(LEDw, OUTPUT);
    digitalWrite(LEDr, HIGH); 					// turn the LED on (HIGH is the voltage level)
    digitalWrite(LEDw, HIGH);
    delay(1000);
    button1.attachClick(sleep);
    button1.attachLongPressStart(changeMode);
}

void loop()
{
	uint8_t i = 0;
    button1.tick();								// keep watching the push buttons:
	if(WDR_flag == true){						// do every 8s 
		WDT_off();
		ADC_on();
		Vcc = readVcc();						//read Battery voltage
		if(Vcc < low && count != 1){
			i = 0;
				while (i++<3) {					//flash 3 times the red LED befor dimm the white LED
					analogWrite(LEDr, PWMr);
					_delay_ms(500);
					digitalWrite(LEDr, LOW);
					_delay_ms(250);
				}			
			PWMw_h = PWMw_h / 2;				//halve the PWM output of the white LED
			if(PWMw_h <= PWMw_l){				//change Mode to low when PWM output is smaler then low PWM
				count = 2;
			}
			if(Vcc <= crit){					//go to sleep when Battery is empty
				sleep();
			}
		}
		if(Vcc < low && count == 1){
			i = 0;
			while (i++<3) {
				digitalWrite(LEDr, LOW);
				_delay_ms(250);
				analogWrite(LEDr, PWMr);
				_delay_ms(500);
				}
			PWMr = PWMr / 2;
			if(Vcc <= crit){
				sleep();	
			}
		}
		ADC_off();								//save power	
		WDT_on;									//start watchdog again
		WDR_flag = false;						//reset WDT Flag
	}
	switch (count) {
		case 1:									//Mode red LED 
		analogWrite(LEDr, PWMr);
		digitalWrite(LEDw, LOW);
		break;
	case 2:										//low Mode white LED
		digitalWrite(LEDr, LOW);
		analogWrite(LEDw, PWMw_l);
		break;
	case 3:										//high Mode white LED
		digitalWrite(LEDr, LOW);
		analogWrite(LEDw, PWMw_h);
		break;
	}
}



void sleep()									// This function will be called when the button1 was pressed 1 time
{
    digitalWrite(LEDr, LOW);
    digitalWrite(LEDw, LOW);
    //prepare sleep
	WDT_off();
	GIFR |= bit(PCIF); 							// clear any outstanding interrupts
    GIMSK |= _BV(PCIE); 						// Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT2); 						// Use PB2 as interrupt pin
    ADC_off();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); 		// replaces above statement

    sleep_enable(); 							// Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei(); 										// Enable interrupts
    sleep_cpu(); 								// sleep

    cli(); 										// Disable interrupts
    PCMSK &= ~_BV(PCINT2); 						// Turn off PB2 as interrupt pin
    sleep_disable(); 							// Clear SE bit
	WDT_on();
    
    flag = 1;									//set flag, uc was in sleepmode before
} 

void changeMode()								// This function will be called once, during pressed for a long time.
{
    if (flag == 1) {
        flag = 0;
    }
    else {
        if (count < ModeNr) {
            count++;
        }
        else {
            count = 1;
        }
    }
}

void WDT_on() {
	// Setup watchdog timer to only interrupt, not reset
	cli();										// Disable interrupts
	wdt_reset(); 								// Reset the WDT
	MCUSR &= ~(1 << WDRF); 						// Clear Watchdog reset flag
	WDTCR = (1 << WDCE) | (1 << WDE); 			// Start timed sequence
	WDTCR = (1 << WDP3) | (1<<WDP0); 			// Watchdog cycle = 8 s
	WDTCR |= (1 << WDIE); 						// Watchdog Interrupt enable
 	sei();										// Enable interrupts
}

void WDT_off(){
	cli();										// Disable interrupts
	wdt_reset();								// Reset the WDT
	MCUSR &= ~(1<<WDRF);						// Clear Watchdog reset flag
	WDTCR |= (1<<WDCE) | (1<<WDE);  			// Start timed sequence
	WDTCR = 0x00;								// Disable WDT
	sei();										// Enable interrupts
}

void ADC_on() {
	ADCSRA |= _BV(ADEN);						// ADC power on
}

void ADC_off() {
	ADCSRA &= ~(1<<7); 							//ADC off
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference

  ADMUX = _BV(MUX3) | _BV(MUX2);
	
  _delay_ms(20); 									// Wait for Vref to settle
  ADCSRA |= _BV(ADSC); 							// Start conversion
  while (bit_is_set(ADCSRA,ADSC)); 				// measuring
 
  uint8_t low_adc  = ADCL; 							// must read ADCL first - it then locks ADCH  
  uint8_t high_adc = ADCH; 							// unlocks both
 
  long result = (high_adc<<8) | low_adc;
 
  result = 1126400L / result; 					// Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return result; 								// Vcc in millivolts
}
