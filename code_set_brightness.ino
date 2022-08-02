#include "OneButton.h"                          //Button Library
#include <avr/sleep.h>                          // Sleep Modes
#include <avr/power.h>                          // Power management
#include <avr/wdt.h>                            // Watchdog Timer
#include <util/delay.h>

#define low 3200
#define crit 2600                               // Battery Level critical, shut down

int mode = 0;
int const modenum = 3;
int LED[3] = {1,4,3};
int LEDpwm[modenum] = {20,50,100};
int set_array[2] = {0,0};
int x = 0;
int y = 0;
int s = 0;
int volatile newPWM;
long Vcc;
// Flags
bool flag = false;
bool volatile stop_flag = true;
bool volatile WDR_flag = false;
bool low_flag = false;
bool block_flag = false;

OneButton button1(2, true);

ISR(PCINT0_vect)
{
}

ISR(WDT_vect){
  WDR_flag=true;
}

void setup(){
for (int i = 0; i <= 1; i++){
  pinMode(LED[i],OUTPUT); 
  digitalWrite(LED[i], HIGH);
}
delay(1000);
button1.attachClick(sleep);      
//button1.attachClick(stop_dim);
button1.attachLongPressStart(settings);
button1.attachDoubleClick(changeMode);
    
WDT_on();
}

void loop(){
uint8_t i = 0;
button1.tick();                                   // keep watching the push buttons:
if(WDR_flag == true){                             // do every 8s 
    WDR_flag = false;                             //reset WDT Flag
    ADC_on();
    Vcc = readVcc();                              //read Battery voltage
    ADC_off();
    if(Vcc <= low && low_flag == false){
      while (i++<5) {                             //flash 5 times the red LED
        analogWrite(LED[0], 255);
        _delay_ms(500);
        digitalWrite(LED[0], LOW);
        _delay_ms(250);
        }
      low_flag = true;       
    }
    if(Vcc <= crit ){
      i = 0;
      while (i++<5) {                             //flash 5 times the red LED befor dimm the white LED
        analogWrite(LED[0], 255);
        _delay_ms(500);
        digitalWrite(LED[0], LOW);
        _delay_ms(250);
        }     
    batsave();                                    //put PWM output of every LED to 10% duty cycle
    }
}
ADC_off();


if(mode == 0){                                    //check witch mode, mode 1 = red LED; mode 2-4 = white LED;
  x = 0;                                          //red LED
  y = 1;
}else{
  x = 1;                                          //white LED
  y = 0;
}
fade();
analogWrite(LED[x], LEDpwm[mode]);                //light the LED with the duty cycle from the array
digitalWrite(LED[y], LOW);
}

void sleep(){                                     // This function will be called when the button1 was pressed 1 time
for (int i = 0; i <= 1; i++){
    digitalWrite(LED[i], LOW);
}
//prepare sleep
WDT_off();
GIFR |= bit(PCIF);                            // clear any outstanding interrupts
GIMSK |= _BV(PCIE);                           // Enable Pin Change Interrupts
PCMSK |= _BV(PCINT2);                         // Use PB2 as interrupt pin
ADC_off();
set_sleep_mode(SLEEP_MODE_PWR_DOWN);          // set sleepmode

sleep_enable();                               // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
sei();                                        // Enable interrupts for wakeup
sleep_cpu();                                  // sleep

cli();                                        // Disable interrupts
PCMSK &= ~_BV(PCINT2);                        // Turn off PB2 as interrupt pin
sleep_disable();                              // Clear SE bit
WDT_on();                                     // turn the watchdog on
flag = true;                                     //set flag, µc was in sleepmode before
delay(500);
} 

inline void changeMode(){                         // This function will be called once, during pressed for a long time.
 
    if (flag == true) {                           // when true lamp was in sleepmode befor, don't change mode because you are waking up rigth now
        flag = false;                             // resetset flag
    }                 // check for regular mode change
      else{
        if (mode < modenum - 1) {
            mode++;                               // change mode
        }
        else {
            mode = 0;
        }
    }
}

 void settings(){                           // set the brightness
    if(flag == false){
  
   if(stop_flag == false){
      LEDpwm[mode] = newPWM;
      stop_flag = true;
   }else{
    stop_flag = false;
   }
 }/*else if(WDR_flag == false){
 button1.tick();                                   // keep watching the push buttons:
 button1.attachClick(next_set);
 uint8_t a = 0; 
  switch(s){
    case 0:
        analogWrite(LED[0], 255);
        _delay_ms(500);
        digitalWrite(LED[0], LOW);
        _delay_ms(250);
        delay(2000);
    break;
    case 1:
         while (a++<2) {                             //flash 5 times the red LED
        digitalWrite(LED[2], HIGH);
        _delay_ms(500);
        digitalWrite(LED[2], LOW);
        _delay_ms(500);
        }
        delay(2000);
    break;
  }
 }*/
}

inline void next_set(){
 
  if(s == 0){
    s == 1;
  }else{
    s = 0;
  }
  return s;
}

inline void fade(){
  //block_flag = true;
  int fadeAmount = 1;
  newPWM = LEDpwm[mode];
  while(stop_flag == false && button1.isLongPressed()){
    button1.tick();
    analogWrite(LED[x], newPWM);
    newPWM = newPWM + fadeAmount;
    if (newPWM <= 0 || newPWM >= 255) {
        fadeAmount = -fadeAmount;
    }
    delay(20);
  }
 //block_flag = false; 
}

void batsave(){
  for (int i = 0; i <= 3; i++){
      LEDpwm[i] = 10;
  }
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
  //int x = 0;
  return result;                // Vcc in millivolts
}
