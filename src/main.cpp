// TODO change date reference system:

// At start, system will store a reference date/time in the data header area
// data points will have the structure: RRRCCCCC CCCCCCDD DDDDDDDD (3 bytes)
// R = date reference number (16 starting date/times)
// C = count (sample number since reference time)
// D = data (10 bit sample data)



#include <Arduino.h>
#include <TinyWireM.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define Wire TinyWireM


constexpr char setDate[] = (__DATE__);
constexpr char setTime[] = (__TIME__);

const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30,
                                       31, 31, 30, 31, 30};

// https://github.com/adafruit/RTClib

//                ┌─────u─────┐
//            NC -│RST     VCC│- 3.3V
//                │           │
//  light sensor -│PB3     SCL│- FRAM, RTC, 4.7k pullup
//                │           │
//   RTC SQW INT -│PB4     PB1│- setting switch?
//                │           │
//           GND -│GND     SDA│- FRAM, RTC, 4.7k pullup
//                └───────────┘
//
//     build inside diffuser globe


// FRAM I2C address
constexpr byte MB85RC_ADRRESS  = 0x50;

// RTC address and registers
constexpr byte PCF8523_ADDRESS = 0x68;
constexpr byte Control_1       = 0x00;
constexpr byte Control_2       = 0x01;
constexpr byte Control_3       = 0x02;
constexpr byte Tmr_CLKOUT_ctrl = 0x0F;
constexpr byte Tmr_A_freq_ctrl = 0x10;
constexpr byte Tmr_A_reg       = 0x11;

constexpr byte settingPin = 1;
constexpr byte sunPin     = 4;


//=================================================================================

// empty interrupt to wake for sample measurement (from RTC)
EMPTY_INTERRUPT(PCINT0_vect)

//=================================================================================

byte writeRegister(byte addr, byte reg, byte data) {
  Wire.beginTransmission(addr);
  Wire.send(reg);
  Wire.send(data);
  return Wire.endTransmission();
}

//=================================================================================

byte readRegister(byte addr, byte reg, byte size = 1) {
  Wire.beginTransmission(addr);
  Wire.send(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, size);
  return Wire.read();
}

//=================================================================================

uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }

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

void updateTime(byte *date) {
  date[5] = readRegister(PCF8523_ADDRESS, 0x03);
  date[4] = readRegister(PCF8523_ADDRESS, 0x04);
  date[3] = readRegister(PCF8523_ADDRESS, 0x05);
  date[2] = readRegister(PCF8523_ADDRESS, 0x06);
  date[1] = readRegister(PCF8523_ADDRESS, 0x08);
  date[0] = readRegister(PCF8523_ADDRESS, 0x09);
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

long date2sec(uint8_t *date){
  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30};
  int days = 0;
  days += date[0] * 365 + (date[0] / 4); // year
  for(uint8_t i=0; i<date[1]-1; i++) { // month
    days += daysInMonth[i];
  }
  days += date[2]; // days; current day made up for by leap in 2000

  long seconds = days * 60 * 60 * 24;
  seconds += date[3] * 60 * 60; // hours
  seconds += date[4] * 60; // minutes
  seconds += date[5]; // seconds

  return seconds + 946684800;
}

//=================================================================================

void saveToFram(int data) {
  static byte date[6];
  static int framPosition = 0;

  if(framPosition == 0){
    Wire.beginTransmission(MB85RC_ADRRESS);
    Wire.write(0);
    Wire.write(0);
    Wire.endTransmission();

    Wire.requestFrom(MB85RC_ADRRESS, uint8_t(2));

    framPosition = (Wire.read() << 8);
    framPosition += Wire.read();
  }

  byte date[6];
  // read active address
    // write if missing
  // write new data to address ++
  // get time now
  // write time now
  // some sort of signal if full (unlikely)

  updateTime(date);
  long seconds = date2sec(date);
  //write data to fram
}

//=================================================================================

void checkSun() {
  static int average = 0;
  static byte count  = 0;

  average += getReading(sunPin);
  count ++;

  if(count == 3){
    saveToFram(average/count);
    average = 0;
    count = 0;
  }
}

//=================================================================================

void configureRTC() {
  // read __DATE__ and __TIME__ into byte array
  byte date[6];
  date[0]     = ((setDate[9] - 48) * 10) + ((setDate[10] - 48));
  date[1]     = (setDate[0] == 'J') ? ((setDate[1] == 'a') ? 1 : ((setDate[2] == 'n') ? 6 : 7))    // Jan, Jun or Jul
                                    : (setDate[0] == 'F') ? 2                                                              // Feb
                                    : (setDate[0] == 'M') ? ((setDate[2] == 'r') ? 3 : 5)                                 // Mar or May
                                    : (setDate[0] == 'A') ? ((setDate[2] == 'p') ? 4 : 8)                                 // Apr or Aug
                                    : (setDate[0] == 'S') ? 9                                                              // Sep
                                    : (setDate[0] == 'O') ? 10                                                             // Oct
                                    : (setDate[0] == 'N') ? 11                                                             // Nov
                                    : (setDate[0] == 'D') ? 12                                                             // Dec
                                    : 0;
  date[2]     = ((setDate[4] - 48) * 10) + (setDate[5] - 48);
  date[3]     = ((setTime[0] - 48) * 10) + (setTime[1] - 48);
  date[4]     = ((setTime[3] - 48) * 10) + (setTime[4] - 48);
  date[5]     = ((setTime[6] - 48) * 10) + (setTime[7] - 48);
  // set time
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.send(byte(3));
  Wire.send(bin2bcd(date[5]));
  Wire.send(bin2bcd(date[4]));
  Wire.send(bin2bcd(date[3]));
  Wire.send(bin2bcd(date[2]));
  Wire.send(bin2bcd(0));
  Wire.send(bin2bcd(date[1]));
  Wire.send(bin2bcd(date[0]));
  Wire.endTransmission();

  // set batter switchover
  writeRegister(PCF8523_ADDRESS, Control_3, byte(0));

  // ensure RTC running
  byte ctlreg = readRegister(PCF8523_ADDRESS, Control_1);
  if (ctlreg & (1 << 5)) {
    writeRegister(PCF8523_ADDRESS, Control_1, ctlreg & ~(1 << 5));
  }

  // turn on timerA interrupts;
  writeRegister(PCF8523_ADDRESS, Control_2,       0b00000010);
  writeRegister(PCF8523_ADDRESS, Tmr_CLKOUT_ctrl,   0b11110010);
  writeRegister(PCF8523_ADDRESS, Tmr_A_freq_ctrl, 0b00000011);
  writeRegister(PCF8523_ADDRESS, Tmr_A_reg,       0b00000101); // 5 minutes

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
  //pinMode(settingPin, INPUT_PULLUP);
  //if(digitalRead(settingPin)) goToSleep(); // switch pulls down for in use (won't be floating)
  //pinMode(settingPin, INPUT);

  configureRTC();
  saveToFram(0xFF); // marker for starting new measurements
}

//=================================================================================

void loop() {
  checkSun();
  goToSleep();
}


// read sun
// write to eeprom
// sleep
// feature for downloading data -> maybe just take the FRAM?

// Temperature??

// datastructure: time in sec, light reading

// what about button to make note in datastore? -> maybe just on rst?