#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "RTClib.h"
#include "SparkFun_MS5803_I2C.h"


#define START_MINUTE -1
#define DATA_DURATION 1
#define SLEEP_DURATION 1
#define SAMPLES_PER_SECOND 1
#define PRECISION 2

// Set to 1 to have info appear on the Serial Monitor when plugged into a computer.
// During deployment, disable (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 1

// Arduino pin for the error LED.
//
#define LED_PIN 7
================================================================================
// Pin used for detecting an alarm on the RTC to wake up the device from deep sleep.
#define INTERRUPT_PIN 2 
#define INTERRUPT_INTPIN 0
#define SD_CS_PIN 8

// Real Time Clock object.
RTC_DS3231 rtc;
// Used for writing data to the current day's log file on the SD card.
File logfile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_HIGH);

DateTime now;
uint8_t oldDay = 0;
volatile bool sampling = true;
volatile bool takeSample = true;


void setup() {
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  ADCSRA = 0;

  if (!rtc.begin()) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC setup error"));
#endif
    error(1);
  }
  rtc.disable32K();
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.clearAlarm(1);
  rtc.disableAlarm(2);


  if (!SD.begin(SD_CS_PIN)) {
#if ECHO_TO_SERIAL
    Serial.println(F("SD setup error"));
#endif
    error(3);
  }

  sensor.reset();
  sensor.begin();
  
  now = rtc.now();
  
  if ((START_MINUTE >= 0) && (now.minute() != START_MINUTE)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Waiting for START_MINUTE"));
    Serial.flush();
#endif // ECHO_TO_SERIAL
    deepSleep(now + TimeSpan(0, 0, 60 - now.minute(), 0), DS3231_A1_Minute);
  }

#if ECHO_TO_SERIAL
  Serial.println(F("Setup complete, beginning sampling"));
  Serial.flush();
#endif
  
  enableTimer2();
  attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
  rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, DATA_DURATION, 0), DS3231_A1_Minute);
}



void loop() {
  now = rtc.now();

  if (oldDay != now.day()) {
    logfile.close();
    createDateCSV();
    oldDay = now.day();
  }

  if (sampling) {
    if (takeSample) {
      char strbuffer[PRECISION + 6];
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      dtostrf(pressure, 4, PRECISION, strbuffer);
          
      logfile.print(now.year());
      logfile.print(F("-"));
      logfile.print(now.month());
      logfile.print(F("-"));
      logfile.print(now.day());
      logfile.print(F(","));
      logfile.print(now.hour());
      logfile.print(F(":"));
      logfile.print(now.minute());
      logfile.print(F(":"));
      logfile.print(now.second());
      logfile.print(F(","));
      logfile.print(strbuffer);
      logfile.print(F(","));
      logfile.println(temperature);
      logfile.flush();
      
#if ECHO_TO_SERIAL
      Serial.print(now.year());
      Serial.print(F("-"));
      Serial.print(now.month());
      Serial.print(F("-"));
      Serial.print(now.day());
      Serial.print(F(" @ "));
      Serial.print(now.hour());
      Serial.print(F(":"));
      Serial.print(now.minute());
      Serial.print(F(":"));
      Serial.print(now.second());
      Serial.print(F(", "));
      Serial.print(strbuffer);
      Serial.print(F(", "));
      Serial.println(temperature);
      Serial.flush();
#endif //ECHO_TO_SERIAL
      
      takeSample = false;
    }
    lightSleep();
  } else {
    TIMSK2 = 0;
    rtc.clearAlarm(1);
    rtc.disable32K();

    deepSleep(rtc.now() + TimeSpan(0, 0, SLEEP_DURATION, 0), DS3231_A1_Minute);

    sampling = true;
    takeSample = true;
    enableTimer2();
    attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
    rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, DATA_DURATION, 0), DS3231_A1_Minute);
  }
}

/*
 * Create (or open if the file name already exists) a CSV for the current day.
 * Name is formatted as YYYYMMDD.csv.
 */
void createDateCSV() {
  char filename[] = "YYYYMMDD.csv";
  logfile = SD.open(now.toString(filename), FILE_WRITE);
  if (!logfile) {
#if ECHO_TO_SERIAL
    Serial.println(F("Couldn't create file."));
#endif
    error(4);
  }
  logfile.println(F("date,time,pressure,temperature"));
  logfile.flush();
#if ECHO_TO_SERIAL
  Serial.print(F("Starting new file: ")); 
  Serial.println(filename);
  Serial.flush();
#endif
}


/*
 * 
 */
void deepSleep(const DateTime dt, Ds3231Alarm1Mode alarm_mode) {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif
  rtc.setAlarm1(dt, alarm_mode);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  sleep_enable();
  attachInterrupt(INTERRUPT_INTPIN, deepSleepISR, LOW);
  //wdt_disable();
  sleep_bod_disable();
  interrupts();
  sleep_cpu();  

  rtc.clearAlarm(1);
#if ECHO_TO_SERIAL
  Serial.println(F("Exiting deep sleep"));
  Serial.flush();
#endif
}

/*
 * 
 */
void deepSleepISR() {
  sleep_disable();
  detachInterrupt(INTERRUPT_INTPIN);
}


/*
 * 
 */
void stopSamplingISR() {
  detachInterrupt(INTERRUPT_INTPIN);
  sampling = false;
}


/*
 * 
 */
void enableTimer2() {
  // Switching to an asynchronous clock source as described in section 18.9 
  // (Asynchronous Operation of Timer/Counter2) of the datasheet
  TIMSK2 = 0;
  
  rtc.enable32K();
  ASSR |= _BV(EXCLK) | _BV(AS2);
  
  TCCR2A = 0;
  // Set the prescaler value to clk/128. This will result in the timer overflowing once per second.
  TCCR2B = _BV(CS22) | _BV(CS20);
  // Set the timer to 0.
  TCNT2 = 0;
  // Wait for the above changes to be made.
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB)));
  // Clear the interrupt flags.
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2);

  // Set the timer to create an interrupt whenever it overflows.
  TIMSK2 = _BV(TOIE2);
}

/*
 * 
 */
ISR(TIMER2_OVF_vect) {
  takeSample = true;
}

/*
 * 
 */
void lightSleep() {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering light sleep"));
  Serial.flush();
#endif

  // We have to make sure we don't re-enter power-save mode too quickly to
  // avoid unwanted behaviour.
  OCR2A = 0;
  while (ASSR & _BV(OCR2AUB));

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  
  // Prevent interrupts while getting ready to sleep. If an interrupt were to occur
  // before sleeping, it would detatch and no longer run to wake up the device.
  noInterrupts();
  sleep_enable();
  //wdt_disable();
  sleep_bod_disable();
  interrupts();
  sleep_cpu();
  
  sleep_disable();

#if ECHO_TO_SERIAL
  Serial.println(F("Exiting light sleep"));
  Serial.flush();
#endif
}

void error(short flashes) {
  int duration = 2000 / flashes;
  while (1) {
    for (uint8_t i = 0; i < flashes; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(duration);
      digitalWrite(LED_PIN, LOW);
      delay(duration);
    }
    delay(1000);
  }
}
