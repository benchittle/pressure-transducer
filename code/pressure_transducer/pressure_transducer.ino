#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "RTClib.h"
#include "SparkFun_MS5803_I2C.h"

// The device will wait until this minute value to begin. Set a value less than
// 0 to sample right away.
#define START_MINUTE -1
// Length of time to sample for.
#define DATA_DURATION 1
// Length of time to sleep after sampling for the above length of time.
#define SLEEP_DURATION 1
// Number of samples to take per second (NOT IMPLEMENTED)
#define SAMPLES_PER_SECOND 1
// Number of decimal places to keep for the pressure readings.
#define PRECISION 2

// Set to 1 to have info appear on the Serial Monitor when plugged into a 
// computer. Disable during deployment, (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 1

#define ERROR_LED_PIN 7

// This pin is used for detecting an alarm from the RTC and triggering an
// interrupt to wake the device up.
#define INTERRUPT_PIN 2 
// Equivalent to digitalPinToInterrupt(2). Used in attachInterrupt() calls.
#define INTERRUPT_INTPIN 0
// Chip Select pin for the SD card reader.
#define SD_CS_PIN 8

// =============================================================================

// Real Time Clock object.
RTC_DS3231 rtc;
// Used for writing data to the current day's log file on the SD card.
File logfile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_HIGH);

DateTime now;
// Track the current day value to determine when a new day begins and when a new
// CSV file must be started.
uint8_t oldDay = 0;

// When the device is active, it will sample at the specified frequency and go
// into a light sleep between samples. When the device is inactive, it will go
// into deep sleep until being awakened by an alarm pulse on the RTC.
volatile bool active = true;
// The device takes a sample whenver it is active and sampling. If it is active
// but not sampling, then it will go into a light sleep.
volatile bool sampling = true;


void setup() {
// Code between these if statements will only be run if ECHO_TO_SERIAL is not 0.
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);

  // Disable the Analog to Digital Converter to save power, as we don't use it
  // here (the MS5803 uses its own).
  ADCSRA = 0;
  wdt_disable();

  // Test connection with the RTC. If it fails, then the error LED will blink
  // once per second until the device is restarted.
  if (!rtc.begin()) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC setup error"));
#endif
    error(1);
  }
  // If the RTC lost power and therefore lost track of time, it will light up 
  // the LED for 2 seconds. While it is not necessary that the RTC has the right
  // time, if multiple sensors are being deployed and are set to start all at the
  // same time then this will warn the user that the sensors are not synchronized.
  if (rtc.lostPower()) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC time not configured (lost power)"));
#endif
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(2000);
    digitalWrite(ERROR_LED_PIN, LOW);
  }

  // These features should be disabled initially to save power.
  rtc.disable32K(); // 32KHz clock
  rtc.writeSqwPinMode(DS3231_OFF); // Square wave output
  rtc.clearAlarm(1); 
  rtc.disableAlarm(2);

  // Test connection with the SD. If it fails, then the error LED will blink twice
  // per second until the device is restarted.
  if (!SD.begin(SD_CS_PIN)) {
#if ECHO_TO_SERIAL
    Serial.println(F("SD setup error"));
#endif
    error(2);
  }

  // Initialize the MS5803 pressure sensor.
  sensor.reset();
  sensor.begin();
  
  now = rtc.now();
  
  // If a START_MINUTE value between 0 and 59 is set, the device will go into deep
  // sleep until the current minute matches specified minute value. Otherwise, the
  // device will start right away.
  if ((START_MINUTE >= 0) && (now.minute() != START_MINUTE)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Waiting for START_MINUTE"));
    Serial.flush();
#endif
    deepSleep(now + TimeSpan(0, 0, (START_MINUTE - now.minute() + 60) % 60, 0), DS3231_A1_Minute);
  }

#if ECHO_TO_SERIAL
  Serial.println(F("Setup complete, beginning sampling"));
  Serial.flush();
#endif
  
  // Prepare to begin sampling.
  enableTimer();
}


void loop() {
  now = rtc.now();

  // Start a new CSV file each day.
  if (oldDay != now.day()) {
    char filename[] = "YYYYMMDD.csv"; // The file name will follow this format.
    
    logfile.close();
    logfile = SD.open(now.toString(filename), FILE_WRITE);

    // If there was an error creating the new log, the device will stop and blink
    // three times per second until being restarted.
    if (!logfile) {
#if ECHO_TO_SERIAL
      Serial.println(F("Couldn't create file."));
#endif
      error(3);
    }
    // Print a header for the file.
    logfile.println(F("date,time,pressure,temperature"));
    logfile.flush();
#if ECHO_TO_SERIAL
    Serial.print(F("Starting new file: ")); 
    Serial.println(filename);
    Serial.flush();
#endif
    oldDay = now.day();
  }

  // If active, the device will see whether it's time to take a sample. Otherwise,
  // enter deep sleep.
  if (active) {
    if (sampling) {
      char buffer[PRECISION + 6];
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      // Convert the pressure value to a string, as doubles are printed as ints
      // to a logfile and we want to keep the fractional value.
      dtostrf(pressure, 4, PRECISION, buffer);
          
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
      logfile.print(buffer);
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
      Serial.print(buffer);
      Serial.print(F(", "));
      Serial.println(temperature);
      Serial.flush();
#endif //ECHO_TO_SERIAL
      
      sampling = false;
    }
    // Enter light sleep briefly while waiting to take the next sample.
    lightSleep();

  } else {
    disableSampling();

    // Enter deep sleep for the specified SLEEP_DURATION
    deepSleep(rtc.now() + TimeSpan(0, 0, SLEEP_DURATION, 0), DS3231_A1_Minute);

    // Prepare to resume sampling.
    enableTimer();
  }
}


/*
 * Puts the device to sleep until being awakened by the RTC's alarm, which
 * will be detected when pin D2 goes low. Arguments will be passed to (and
 * are the same as) rtc.setAlarm1.
 */
void deepSleep(const DateTime dt, Ds3231Alarm1Mode alarm_mode) {
  byte temp1 = 0;
  byte temp2 = 0;
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif
  rtc.setAlarm1(dt, alarm_mode);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  sleep_enable();
  attachInterrupt(INTERRUPT_INTPIN, deepSleepISR, LOW);
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
  active = false;
}


/*
 * 
 */
void enableTimer() {
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

  active = true;
  sampling = true;

  attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
  rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, DATA_DURATION, 0), DS3231_A1_Minute);
}

void disableSampling() {
  TIMSK2 = 0;
  rtc.disable32K();
  rtc.clearAlarm(1);
}

/*
 * 
 */
ISR(TIMER2_OVF_vect) {
  sampling = true;
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
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(duration);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(duration);
    }
    delay(1000);
  }
}
