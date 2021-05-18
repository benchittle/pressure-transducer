#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "RTClib.h" // https://github.com/adafruit/RTClib
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library

// The device will wait until this minute value to begin. Set a value less than
// 0 to sample right away.
#define START_MINUTE -1
// Length of time to sample for.
#define DATA_DURATION 1
// Length of time to sleep after sampling for the above length of time. Set to
// 0 for continuous sampling.
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

/*
 * Setup the ATMEGA328P and peripherals to make sure connections are correct.
 * We will also wait before baginning sampling until the specified START_MINUTE.
 */
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
  wdt_disable(); // Also disable the watchdog timer.

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
  // time, if multiple sensors are being deployed and are meant to start at the
  // same time, this will warn the user that the sensors are not synchronized.
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
  // Test connection with the SD. If it fails, then the error LED will blink 
  // twice per second until the device is restarted.
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

  // If a START_MINUTE value between 0 and 59 is set, the device will go into 
  // deep sleep until the current minute matches specified minute value.
  // Otherwise, the device will start right away.
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

// =============================================================================
/*
 * Main program loop. First, the program determines whether a new day has 
 * started. If so, it starts a new CSV file for data logging. Next, the device 
 * will either be sampling or entering deep sleep. While sampling, the device 
 * takes a specified number of samples per second, briefly sleeping between 
 * samples, and saves them to the log file. This continues for a duration 
 * specified by DATA_DURATION. While entering deep sleep, the device disables as
 * many components as possible to save power and relies on an alarm pulse from 
 * the DS3231 in order to wake up after a duration specified by SLEEP_DURATION
 */
void loop() {
  now = rtc.now();

  // Start a new CSV file each day.
  if (oldDay != now.day()) {
    char filename[] = "YYYYMMDD.csv"; // The file name will follow this format.
    
    logfile.close();
    logfile = SD.open(now.toString(filename), FILE_WRITE);

    // If there was an error creating the new log, the device will stop and 
    // blink three times per second until being restarted.
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

  // If active, the device will see whether it's time to take a sample.
  // Otherwise, enter deep sleep.
  if (active) {
    if (sampling) {
      char buffer[PRECISION + 6];
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      // Convert the pressure value to a string, as doubles are printed as
      // ints to a logfile and we want to keep the fractional value.
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
    disableTimer();

    // Enter deep sleep for the specified SLEEP_DURATION
    deepSleep(rtc.now() + TimeSpan(0, 0, SLEEP_DURATION, 0), DS3231_A1_Minute);

    // Prepare to resume sampling.
    enableTimer();
  }
}

// =============================================================================

/*
 * Puts the device to sleep until being awakened by the RTC's alarm, which
 * should pull pin D2 low and trigger an interrupt. Arguments will be passed to
 * (and are the same as) rtc.setAlarm1. This sleep mode saves more power than
 * lightSleep
 */
void deepSleep(const DateTime wake_time, Ds3231Alarm1Mode alarm_mode) {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif
  // When the alarm goes off, it will cause the SQW pin of the DS3231 to go low.
  rtc.setAlarm1(wake_time, alarm_mode);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Saves the most power.
  // Disable interrupts while preparing to sleep. Without doing this, the
  // interrupt could go off before the sleep_cpu() command and leave the device
  // stuck in a sleep state with no interrupt to wake it up.
  noInterrupts();
  sleep_enable();
  // When a low signal is detected on pin D2, the deepSleepISR will run as an
  // interrupt service routine (ISR) to wake the device up.
  attachInterrupt(INTERRUPT_INTPIN, deepSleepISR, LOW);
  // Disable brown out detection to save power (must be done as the last 
  // instruction before sleeping).
  sleep_bod_disable(); 
  interrupts(); // Enable interrupts again.
  sleep_cpu(); // Enter sleep.

  // When the device wakes up, it will resume here after executing the 
  // deepSleepISR.

  rtc.clearAlarm(1);
#if ECHO_TO_SERIAL
  Serial.println(F("Exiting deep sleep"));
  Serial.flush();
#endif
}


/*
 * Interrupt Service Routine used to wake the device from deep sleep.
 */
void deepSleepISR() {
  sleep_disable();
  detachInterrupt(INTERRUPT_INTPIN);
}


/*
 * Begin using the 32KHz output of the DS3231 as an external clock source for
 * the MCU's Timer/Counter2. This timer is then set to trigger interrupts at
 * regular intervals, depending on the sampling frequency. We use the DS3231's
 * 32KHz output since the MCU's timers are not as accurate on their own.
 */
void enableTimer() {
  // Begin the clock on the DS3231, which should be connected to pin TOSC1/XTAL1.
  rtc.enable32K();

  // Switching to an asynchronous clock source as described in section 18.9 
  // (Asynchronous Operation of Timer/Counter2) of the datasheet:

  // Tell the MCU that we will be using an external clock source (EXCLK = 1) and
  // that we want Timer/Counter2 to be clocked from a clock source connected to
  // pin TOSC1/XTAL1 (AS2 = 1).
  ASSR |= _BV(EXCLK) | _BV(AS2);
  
  TCCR2A = 0; // Using normal operation mode settings here.
  // Set the prescaler value to clk/128. This will result in the timer 
  // overflowing once per second.
  TCCR2B = _BV(CS22) | _BV(CS20);
  TCNT2 = 0; // Reset the timer to 0.
  
  // Wait for the above changes to be made.
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB)));
  
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // Clear the interrupt flags.

  // Set the timer to create an interrupt whenever it overflows.
  TIMSK2 = _BV(TOIE2);

  // This will allow the main loop to execute the sampling code.
  active = true;
  sampling = true;
#if DATA_DURATION > 0
  // Set an interrupt and alarm to stop sampling after the time specified by
  // DATA_DURATION has passed.
  attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
  rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, DATA_DURATION, 0), DS3231_A1_Minute);
#endif
}


/*
 * Interrupt Service Routine used to tell the device to stop sampling.
 */
void stopSamplingISR() {
  detachInterrupt(INTERRUPT_INTPIN);
  active = false;
}


/*
 * Interrupt Service Routine used wake the device up from light sleep and tell
 * it to take a single sample.
 */
ISR(TIMER2_OVF_vect) {
  sampling = true;
}


void disableTimer() {
  TIMSK2 = 0; // Disable Timer/Counter2 interrupts.
  rtc.disable32K();
  rtc.clearAlarm(1);
}


/*
 * Puts the device to sleep until being awakened by an interrupt from 
 * Timer/Counter2. This sleep mode saves less power than deepSleep, but is
 * faster to wake up from when higher frequency sampling is needed.
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
  
  // When the device wakes up, it will resume here after executing the 
  // TIMER2_OVF_vect ISR.

  sleep_disable();

#if ECHO_TO_SERIAL
  Serial.println(F("Exiting light sleep"));
  Serial.flush();
#endif
}


/*
 * Enter an endless loop while flashing the error LED a given number of times
 * each second.
 */
void error(short flashes) {
  int duration = 1000 / flashes;
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
