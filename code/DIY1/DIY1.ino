// =============================================================================
/* 
 * The original code is the work of the Mississippi State University Coastal 
 * Research & Extension Center and can be found in the DIY Wave Gauge project
 * found here:
 * http://coastal.msstate.edu/waves
 */
// =============================================================================

#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <SD.h>
#include "RTClib.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

#define ECHO_TO_SERIAL 0
#define SDPIN 10

// Length of time that the sensor should be active vs. inactive per cycle (seconds). 
// I.E. 1020 seconds = 17 minutes active, 180 seconds = 3 minutes inactive is 
// one full cycle.
#define SAMPLING_DURATION 1
#define SLEEP_DURATION 1

// This pin is used for detecting an alarm from the RTC and triggering an
// interrupt to wake the device up.
#define INTERRUPT_PIN 2 
// Equivalent to digitalPinToInterrupt(2). Used in attachInterrupt() calls.
#define INTERRUPT_INTPIN 0

// Begin class with selected address
// available addresses (selected by jumper on board) 
// default is ADDRESS_HIGH

//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77

MS5803 sensor(ADDRESS_HIGH);

RTC_PCF8523 rtc; // define the Real Time Clock object

DateTime now;
DateTime stopSampling;
const TimeSpan samplingDuration = TimeSpan(0, 0, SAMPLING_DURATION, 0);

volatile bool sampling;

// the logging file
File logfile;

// This void error setup is to define the error message we'll serial print and/or relay with LED lights if things aren't right
void error(const char str[])
{
#if ECHO_TO_SERIAL
  Serial.print("error: ");
  Serial.println(str);
#endif

  // red LED indicates error
  digitalWrite(LED_BUILTIN, HIGH);

  while(1);
}

void setup() {  
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif

  pinMode(SDPIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  ADCSRA = 0;
  ACSR = _BV(ACD);

  sensor.reset();
  sensor.begin();

  // connect to RTC
  if (!rtc.begin()) {
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  rtc.deconfigureAllTimers();

  // see if the card is present and can be initialized:
  if (!SD.begin(SDPIN)) {
    error("Card failed, or not present");
  }
  
  // create a new file
  char filename[] = "YYYYMMDD.csv";
  logfile = SD.open(rtc.now().toString(filename), FILE_WRITE);
  
  if (!logfile) {
    error("couldnt create file");
  }
#if ECHO_TO_SERIAL
  Serial.print("Logging to: ");
  Serial.println(filename);
#endif

  logfile.println("date, time, pressure, temperature");
  logfile.flush();

  sampling = true;
  stopSampling = rtc.now() + samplingDuration;
  enableTimer();
}

void loop() {  
  now = rtc.now();

  if (now < stopSampling) {
    if (sampling) {
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
    
      // print the results to the Serial Monitor:
#if ECHO_TO_SERIAL
      Serial.print("pressure = ");
      Serial.print(pressure);
      Serial.print("temp = ");
      Serial.print(temperature);
      Serial.print("\t date = ");
      Serial.print(now.year());
      Serial.print("/");
      Serial.print(now.month());
      Serial.print("/");
      Serial.print(now.day());
      Serial.print("\t time = ");
      Serial.print(now.hour());
      Serial.print(":");
      Serial.print(now.minute());
      Serial.print(":");
      Serial.println(now.second());
      Serial.flush();
#endif //ECHO_TO_SERIAL
      
      // Write to SD card:
      logfile.print(now.year()); 
      logfile.print("/");
      logfile.print(now.month());
      logfile.print("/");
      logfile.print(now.day());
      logfile.print(", ");
      logfile.print(now.hour());
      logfile.print(":");
      logfile.print(now.minute());
      logfile.print(":");
      logfile.print(now.second());
      logfile.print(",");
      logfile.print(pressure);
      logfile.print(",");
      logfile.println(temperature);
      logfile.flush();

      sampling = false;
    }

    shortSleep();
  } else {
    disableTimer();

    longSleep(SLEEP_DURATION);

    sampling = true;
    stopSampling = rtc.now() + samplingDuration;
    enableTimer();
  }
}

void enableTimer() {
  attachInterrupt(INTERRUPT_INTPIN, takeSampleISR, LOW);
  rtc.enableSecondTimer();
}


void takeSampleISR() {
  sampling = true;
}


void disableTimer() {
  rtc.disableSecondTimer();
}

/*
 * Puts the device to sleep until being awakened by the RTC's alarm, which
 * should pull pin D2 low and trigger an interrupt. 
 */
void longSleep(uint8_t minutes) {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif
  // When the alarm goes off, it will cause the SQW pin of the RTC to go low.
  rtc.enableCountdownTimer(PCF8523_FrequencyMinute, minutes);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Saves the most power.
  // Disable interrupts while preparing to sleep. Without doing this, the
  // interrupt could go off before the sleep_cpu() command and leave the device
  // stuck in a sleep state with no interrupt to wake it up.
  noInterrupts();
  sleep_enable();
  // When a low signal is detected on pin D2, the longSleepISR will run as an
  // interrupt service routine (ISR) to wake the device up.
  attachInterrupt(INTERRUPT_INTPIN, longSleepISR, LOW);
  // Disable brown out detection to save power (must be done as the last 
  // instruction before sleeping).
  sleep_bod_disable(); 
  interrupts(); // Enable interrupts again.
  sleep_cpu(); // Enter sleep.

  // When the device wakes up, it will resume here after executing the 
  // longSleepISR.

  rtc.disableCountdownTimer();
#if ECHO_TO_SERIAL
  Serial.println(F("Exiting deep sleep"));
  Serial.flush();
#endif
}


/*
 * Interrupt Service Routine used to wake the device from deep sleep.
 */
void longSleepISR() {
  sleep_disable();
  detachInterrupt(INTERRUPT_INTPIN);
}


void shortSleep() {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering short sleep"));
  Serial.flush();
#endif
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Saves the most power.
  // Disable interrupts while preparing to sleep. Without doing this, the
  // interrupt could go off before the sleep_cpu() command and leave the device
  // stuck in a sleep state with no interrupt to wake it up.
  noInterrupts();
  sleep_enable();
  // Disable brown out detection to save power (must be done as the last 
  // instruction before sleeping).
  sleep_bod_disable(); 
  interrupts(); // Enable interrupts again.
  sleep_cpu(); // Enter sleep.

  // When the device wakes up, it will resume here after executing  
  // takeSampleISR

#if ECHO_TO_SERIAL
  Serial.println(F("Exiting light sleep"));
  Serial.flush();
#endif
}
