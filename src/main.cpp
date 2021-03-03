////////////////////////////////////////////////
///     
///     R  -  RST  -  reset button
///     0  -  SDA  -  FRAM, RTC
///     1  -       -  setting switch?
///     2  -  SCL  -  FRAM, RTC
///     3  -  A3   -  light sensor
///     4  -       -  RTC wakeup?
///     
///     build inside diffuser globe
///     
///     

#include <Arduino.h>

#include <TinyWireM.h>

#define Wire TinyWireM

// include FRAM -> modify for TinyWire
// include RTC  -> modify for TinyWire
// include sleep

constexpr byte PCF8523_ADDRESS = 0x68;
constexpr byte Control_2       = 0x01;
constexpr byte CONTROL_3       = 0x02;
constexpr byte Tmr_CLKOUT_ctrl = 0x0F;
constexpr byte Tmr_A_freq_ctrl = 0x10;
constexpr byte Tmr_A_reg       = 0x11;

constexpr byte settingPin =
constexpr byte ldrPin     =

Wire.begin();

// interrupt to count RTC's WDT output

//=================================================================================

EMPTY_INTERRUPT(PCINT0_vect)

//=================================================================================

void goToSleep() {
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK = 0b00010000;    // turn on interrupts on pins PB4

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_mode();
  //--SLEEP HAPPENS HERE -- WITH ISR BETWEEN--
  sleep_disable();

  GIMSK = 0b00000000;
}

//=================================================================================

int getReading (byte port) {
  power_adc_enable() ;
  ADCSRA = bit (ADEN) | bit (ADIF);  // enable ADC, turn off any pending interrupt
  
  // set a2d prescale factor to 128
  // 8 MHz / 128 = 62.5 KHz, inside the desired 50-200 KHz range.

  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2); 
  
  if (port >= A0)
    port -= A0;
    
#if defined(__AVR_ATtiny85__)  
  ADMUX = (port & 0x07);  // AVcc   
#else   
  ADMUX = bit (REFS0) | (port & 0x07);  // AVcc   
#endif

  noInterrupts ();
  set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
  sleep_enable();  
  
  // start the conversion
  ADCSRA |= bit (ADSC) | bit (ADIE);
  interrupts ();
  sleep_cpu ();     
  sleep_disable ();

  // reading should be done, but better make sure
  // maybe the timer interrupt fired 

  // ADSC is cleared when the conversion finishes
  while (bit_is_set (ADCSRA, ADSC))
    { }

  byte low  = ADCL;
  byte high = ADCH;

  ADCSRA = 0;  // disable ADC
  power_adc_disable();
  
  return (high << 8) | low;
  
  }  // end of getReading

//=================================================================================

byte checkSun() {
  static int average = 0;
  static byte count  = 0;

  average += getReading;
  count ++;

  if(count == 5){
    saveToFram(average/count);
    average = 0;
    count = 0;
  }

  // on 5th average (or n) convert
    // normalize/linearize and compress to byte and save to FRAM
}

//=================================================================================

long secondsTime() {
  // query RTC for seconds timestamp
  
  // return time;
}

//=================================================================================

void saveToFram(byte data) {
  // read active address
    // write if missing
  // write new data to address ++
  // get time now
  // write time now
  // some sort of signal if full (unlikely)
}

//=================================================================================

void configureRTC() {
  // adjust time
  // CONTROL_3 = 0;
  //


  //Control_2 = 0b00000010; // turn on timerA interrupts;



  // disable clockout for int function
  // countdown timer 8.9.2.2
  // Tmr_CLKOUT_ct = 0b11110010
  // TAM = 1 (pulse)
  // TBM = 1 (not used)
  // COF = 111 (clock out disabled)
  // TAC = 01 (timer A is countdown)
  // TBC = 0 (timer B is disabled)

  // Tmr_A_freq_ctrl = 0b00000011; // one minute clock source
  // Tmr_A_reg = count (as byte) to trigger end of timer
}

//=================================================================================

void setup() {
  pinMode(settingPin, INPUT_PULLUP);
  if(digitalRead(settingPin)) goToSleep(); // switch pulls down for in use (won't be floating)
  pinMode(settingPin, INPUT);

  // set up RTC

}

//=================================================================================

void loop() {
  // checkSun
  // goto sleep
}


// read sun
// write to eeprom
// sleep
// feature for downloading data -> maybe just take the FRAM?

// Temperature??

// datastructure: time in sec, light reading