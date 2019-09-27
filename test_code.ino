#include "OneButton.h"				//Button Library
#include <avr/sleep.h>              // Sleep Modes
#include <avr/power.h>              // Power management
#include <avr/wdt.h>              	// Watchdog Timer
//#include <util/delay.h>

#define mid 4000                	// Battery Level Mid
#define low 3200                	// Battery Level low, flash and start dimming the Led
#define crit 2600               	// Battery Level critical, shut down


const byte LEDr = 1;              	// pin 1
const byte LEDw = 4;              	// pin 4
const byte ModeNr = 3;				// number of modis; change the number cases in the switch statement
volatile bool WDR_flag = false;		// flag inside the ISR(WDT_vec) 
bool PWR_flag = false;				// flag gets set when voltage drops bellow

OneButton button1(2, true);
int count = 1;
int flag = 0;
long Vcc;
int PWMr = 255;
int PWMw_l = 50;
int PWMw_h = 255;

ISR(PCINT0_vect)
{
}

ISR(WDT_vect){
  WDR_flag=true;
}

void setup()
{
    pinMode(3, OUTPUT);
    pinMode(LEDr, OUTPUT);            // turn the LED on (HIGH is the voltage level)
    pinMode(LEDw, OUTPUT);
    digitalWrite(LEDr, HIGH);           // turn the LED on (HIGH is the voltage level)
    digitalWrite(LEDw, HIGH);
    delay(1000);
    button1.attachClick(sleep);
    button1.attachLongPressStart(changeMode);
    WDT_on();
}

void loop()
{
  uint8_t i = 0;
    button1.tick();               					// keep watching the push buttons:
  if(WDR_flag == true){           					// do every 8s 
    WDR_flag = false;           					//reset WDT Flag
    ADC_on();
    Vcc = readVcc();            					//read Battery voltage
    if(Vcc < low && count == 3){
      if(PWMw_h <= PWMw_l){       					//change Mode to low when PWM output is smaler then low PWM
        count = 2;
      }else{
    i = 0;
        while (i++<3 && PWR_flag == false) {         //flash 3 times the red LED befor dimm the white LED
          analogWrite(LEDr, PWMr);
          _delay_ms(500);
          digitalWrite(LEDr, LOW);
          _delay_ms(250);
      PWR_flag = true;
        }     
      PWMw_h = PWMw_h - 10;        					//halve the PWM output of the white LED
      }
	  if(Vcc <= crit){
        batsave();  
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
      PWMr = PWMr - 10;
      if(Vcc <= crit){
        batsave();  
      }
    }
    ADC_off();                //save power  
  }
  switch (count) {
    case 1:                 //Mode red LED 
    analogWrite(LEDr, PWMr);
    digitalWrite(LEDw, LOW);
    break;
  case 2:                   //low Mode white LED
    digitalWrite(LEDr, LOW);
    analogWrite(LEDw, PWMw_l);
    break;
  case 3:                   //high Mode white LED
    digitalWrite(LEDr, LOW);
    analogWrite(LEDw, PWMw_h);
    break;
  }
}



void sleep(){                  // This function will be called when the button1 was pressed 1 time

    digitalWrite(LEDr, LOW);
    digitalWrite(LEDw, LOW);
    //prepare sleep
    WDT_off();
    GIFR |= bit(PCIF);              // clear any outstanding interrupts
    GIMSK |= _BV(PCIE);             // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT2);             // Use PB2 as interrupt pin
    ADC_off();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();               // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                    // Enable interrupts
    sleep_cpu();                // sleep

    cli();                    // Disable interrupts
    PCMSK &= ~_BV(PCINT2);            // Turn off PB2 as interrupt pin
    sleep_disable();              // Clear SE bit
    WDT_on();
    flag = 1;                 //set flag, uc was in sleepmode before
} 

void changeMode(){               // This function will be called once, during pressed for a long time.

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

void batsave(){
	PWMw_l = 10;
    PWMr = 10;
    WDT_off();
}

void WDT_on(){
  cli();                    // Disable interrupts
  wdt_reset();                // Reset the WDT
  WDTCR = (1 << WDIE) | (1 << WDP3) | (1<<WDP0);      // Watchdog cycle = 8 s
  sei();                    // Enable interrupts
}

void WDT_off(){
  cli();                    // Disable interrupts
  wdt_reset();                // Reset the WDT
  wdt_disable();
  sei();                    // Enable interrupts
}

void ADC_on(){
  ADCSRA = (1 << ADEN );            // ADC power on
}

void ADC_off(){
  ADCSRA &= ~(1<<7);              //ADC off
}

long readVcc(){
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
   
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2);                  // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);              // Start conversion
  while (bit_is_set(ADCSRA,ADSC));        // measuring
 
  uint8_t low_adc  = ADCL;              // must read ADCL first - it then locks ADCH  
  uint8_t high_adc = ADCH;              // unlocks both
 
  long result = (high_adc<<8) | low_adc;
 
  result = 1126400L / result;           // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  int x = 0;
  return result;                // Vcc in millivolts
}
