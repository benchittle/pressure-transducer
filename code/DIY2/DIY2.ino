#include <Wire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "SdFat.h"
#include "RTClib.h" // https://github.com/adafruit/RTClib
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library


//** The following four config values will be used when no config file is found.
//** If a config file is found, these values will be ignored.
// The device will wait until this time to begin. Setting a value in the past 
// will cause the device to start immediately.
#define DEFAULT_START_TIME DateTime(2000, 1, 1, 0, 0)
// Length of time to sample for.
#define DEFAULT_SAMPLING_DURATION 60
// Length of time to sleep after sampling for the above duration. Set to 0 for 
// continuous sampling.
#define DEFAULT_SLEEP_DURATION 0
// Specify a name for the sensor to use when creating logging files.
#define DEFAULT_SENSOR_NAME "MS2"
#define SENSOR_NAME_SIZE 10

// The sensor name is combined with the suffix to create the output file name
// during logging.
#define OUTPUT_FILE_NAME_SUFFIX "_YYYYMMDD-hhmm.csv"
// Path to configuration file on SD card if one exists
#define CONFIG_FILE_NAME "config.txt"
// Default number of samples to take per second (NOT IMPLEMENTED)
//#define SAMPLES_PER_SECOND 1
// Number of decimal places to keep for the pressure readings.
#define PRECISION 2

// Set to 1 to have info appear on the Serial Monitor when plugged into a 
// computer. Disable during deployment, (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 0
#define ECHO_TO_PLOT 0

// This pin is used for detecting an alarm from the RTC and triggering an
// interrupt to wake the device up.
#define INTERRUPT_PIN 2 
// Equivalent to digitalPinToInterrupt(2). Used in attachInterrupt() calls.
#define INTERRUPT_INTPIN 0
// Chip Select pin for the SD card reader.
#define SD_CS_PIN 8
// BLINK DESCRIPTION
#define ERROR_LED_PIN 4

// =============================================================================

// Real Time Clock object.
RTC_DS3231 rtc;
// Used for writing data to the current day's log file on the SD card.
SdFat sd;
SdFile logFile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_LOW);

DateTime now;
// Track the current day value to determine when a new day begins and when a new
// CSV file must be started.
uint8_t oldDay = 0;

char filename[SENSOR_NAME_SIZE + sizeof(OUTPUT_FILE_NAME_SUFFIX)];;

// These values will be obtained from the config.txt file on the SD card if it
// exists. Otherwise, the corresponding default values will be used.
uint16_t samplingDuration = DEFAULT_SAMPLING_DURATION;
uint16_t sleepDuration = DEFAULT_SLEEP_DURATION;
char sensorName[SENSOR_NAME_SIZE + 1] = DEFAULT_SENSOR_NAME;

// When the device is active, it will sample at the specified frequency and go
// into a light sleep between samples. When the device is inactive, it will go
// into deep sleep until being awakened by an alarm pulse on the RTC.
volatile bool active = true;
// The device takes a sample whenver it is active and sampling. If it is active
// but not sampling, then it will go into a light sleep.
volatile bool sampling = true;

// =============================================================================

/*
 * Setup the ATMEGA328P and peripherals to make sure connections are correct.
 * We will also wait before baginning sampling until the specified start
 * minute.
 */
void setup() {
// Code between these if statements will only be run if ECHO_TO_SERIAL is not 0.
#if ECHO_TO_SERIAL + ECHO_TO_PLOT
  Serial.begin(9600);
#endif

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(ERROR_LED_PIN, OUTPUT);

  // Disable the Analog to Digital Converter to save power, as we don't use it
  // here (the MS5803 uses its own).
  ADCSRA = 0;
  ACSR = _BV(ACD); // Analog comparator.
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
  // the LED for 1 second. While it is not necessary that the RTC has the right
  // time, if multiple sensors are being deployed and are meant to start at the
  // same time, this will warn the user that the sensors are not synchronized.
  if (rtc.lostPower()) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC time not configured (lost power)"));
#endif
    warning(500, 1);
  }
  // These features should be disabled initially to save power.
  rtc.disable32K(); // 32KHz clock
  rtc.writeSqwPinMode(DS3231_OFF); // Square wave output
  rtc.clearAlarm(1); 
  rtc.disableAlarm(2);

  // Test connection with the SD. If it fails, then the error LED will blink 
  // twice per second until the device is restarted.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
#if ECHO_TO_SERIAL
    Serial.println(F("SD setup error"));
#endif
    error(2);
  }

  // Initialize the MS5803 pressure sensor.
  sensor.reset();
  sensor.begin();

  // Take a sample pressure reading and make sure it's reasonable.
  float testPressure = sensor.getPressure(ADC_4096);
  if (testPressure < 0 || testPressure > 4000) {
#if ECHO_TO_SERIAL
    Serial.println(F("MS5803 error"));
#endif
    error(4);
  } else if (testPressure > 1000) {
#if ECHO_TO_SERIAL
    Serial.println(F("Pressure > 1400"));
#endif
    warning(250, 4);
  }

  // Used for reading data from the config file, if present.
  SdFile configFile;
  // The sensor will sleep until this minute value.
  DateTime startTime = DEFAULT_START_TIME;
  // Open the config file if it exists to obtain the start minute, data
  // sampling duration, sleep duration, and info string.
  if (configFile.open(CONFIG_FILE_NAME, O_READ)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Reading from config file:"));
#endif

    // Determines whether a number was read for each config variable. If one is
    // missing, we assume the config file is wrong and use all default values.
    uint8_t validityFlag = 0;
    // Store the datetime values for the start time.
    uint8_t dateTime[5] = {0};
    // An array to store the numeric configuration values.
    uint8_t configVars[2] = {0};

    char c;
    for (uint8_t i = 0; i < 5; i++) {
      validityFlag <<= 1;
      // Reading a single character.
      c = configFile.read();
      // Build the number by reading one digit at a time until reaching a 
      // non-numeric character.
      while (c >= '0' && c <= '9') {
        validityFlag |= 1;
        dateTime[i] = (10 * dateTime[i]) + (c - '0');
        c = configFile.read();
      }
      if (c != ' ' && i < 4) {
        break;
      }
    }
    // Read until the current line ends or the file ends.
    while (c != '\n' && c != EOF) {
      c = configFile.read();
    }

    // Continue reading information if the first 5 values were read successfully.
    if (c != EOF && validityFlag == 0b00011111) {
      // Reading in a number from the 2nd and 3rd lines of the file:
      for (uint8_t i = 0; i < 2; i++) {
        validityFlag <<= 1;
        // Reading a single character.
        c = configFile.read();
        // Build the number by reading one digit at a time until reaching a 
        // non-numeric character.
        while (c >= '0' && c <= '9') {
          validityFlag |= 1;
          configVars[i] = (10 * configVars[i]) + (c - '0');
          c = configFile.read();
        }

        // Read until the current line ends or the file ends.
        while (c != '\n' && c != EOF) {
          c = configFile.read();
        }
      }
    }
    // Look for the sensor name if we still haven't reached the end of the file
    // and all other config values have been found. Otherwise, use the default.
    if (c != EOF && validityFlag == 0b01111111) {
      // Read up to 63 characters on the current line to form the info string.
      uint8_t i = 0;
      do {
        c = configFile.read();
        sensorName[i++] = c;
      } while ((c != '\n' && c != EOF) && (i < SENSOR_NAME_SIZE));
      sensorName[i - 1] = '\0';
    } else {
#if ECHO_TO_SERIAL
      Serial.println(F("No info string found. Using default info string:"));
#endif
    }

    configFile.close();

    if (validityFlag == 0b01111111) {
      // Set these values based on the corresponding config values.
      startTime = DateTime(dateTime[0], dateTime[1], dateTime[2], dateTime[3], dateTime[4]); // 1st line in file.
      samplingDuration = configVars[0]; // 2nd line in file.
      sleepDuration = configVars[1]; // 3rd line in file.
    } else {
#if ECHO_TO_SERIAL
      Serial.println(F("Invalid config file. Using default values:"));
#endif
      warning(500, 2);
    }
  } else {
#if ECHO_TO_SERIAL
    Serial.println(F("No config file found. Using default values:"));
#endif
    warning(500, 2);
  }

#if ECHO_TO_SERIAL
  Serial.print(F("startTime = "));
  Serial.print(startTime.year());
  Serial.print("-");
  Serial.print(startTime.month());
  Serial.print("-");
  Serial.print(startTime.day());
  Serial.print(F(" @ "));
  Serial.print(startTime.hour());
  Serial.print(F(":"));
  Serial.println(startTime.minute());
  Serial.print(F("samplingDuration = "));
  Serial.println(samplingDuration);
  Serial.print(F("sleepDuration = "));
  Serial.println(sleepDuration);
  Serial.print(F("sensorName = "));
  Serial.println(sensorName);
#endif
  
  now = rtc.now();

  // If a startMinute value between 0 and 59 is set, the device will go into 
  // deep sleep until the current minute matches specified minute value.
  // Otherwise, the device will start right away.
  if (startTime > now) {
#if ECHO_TO_SERIAL
    Serial.println(F("Waiting for startTime"));
    Serial.flush();
#endif
    deepSleep(startTime, DS3231_A1_Minute);
  }

#if ECHO_TO_SERIAL
  Serial.println(F("\nSetup complete, beginning sampling"));
  Serial.flush();
#endif
  // Tell the user the device was initialized properly.
  warning(3000, 1);
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
 * specified by samplingDuration. While entering deep sleep, the device disables as
 * many components as possible to save power and relies on an alarm pulse from 
 * the DS3231 in order to wake up after a duration specified by sleepDuration.
 */
void loop() {
  now = rtc.now();

  // Start a new CSV file each day.
  if (oldDay != now.day()) {
    strcpy(filename, sensorName);
    strcat(filename, OUTPUT_FILE_NAME_SUFFIX);

    if (logFile.isOpen()) {
      logFile.timestamp(T_WRITE, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      logFile.close();
    } 
    
    // If there was an error creating the new log, the device will stop and 
    // blink three times per second until being restarted.
    if (!logFile.open(now.toString(filename), O_WRITE | O_CREAT | O_AT_END)) {
#if ECHO_TO_SERIAL
      Serial.println(F("Couldn't create file."));
#endif
      error(3);
    }
    // Print a header for the file.
    logFile.write("samplingDuration,sleepDuration,sensorName\n");
    logFile.printField(samplingDuration, ',');
    logFile.printField(sleepDuration, ',');
    logFile.write(sensorName);
    logFile.write("\ndatetime,pressure,temperature\n");
    logFile.timestamp(T_CREATE | T_WRITE, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    logFile.sync();
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
      float pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
          
      logFile.printField(now.year(), '-');
      logFile.printField(now.month(), '-');
      logFile.printField(now.day(), ' ');
      logFile.printField(now.hour(), ':');
      logFile.printField(now.minute(), ':');
      logFile.printField(now.second(), ',');
      logFile.printField(pressure, ',', PRECISION);
      logFile.printField(temperature, '\n');
      logFile.sync();
      
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
      Serial.print(pressure);
      Serial.print(F(", "));
      Serial.println(temperature);
      Serial.flush();
#endif //ECHO_TO_SERIAL

#if ECHO_TO_PLOT
      Serial.println(pressure);
      Serial.flush();
#endif //ECHO_TO_PLOT
      
      sampling = false;
    }
    // Enter light sleep briefly while waiting to take the next sample.
    lightSleep();

  } else {
    disableTimer();
    logFile.close();

    // Enter deep sleep for the specified sleepDuration.
    deepSleep(rtc.now() + TimeSpan(0, 0, sleepDuration, 0), DS3231_A1_Minute);

    logFile.open(filename, O_WRITE | O_AT_END);
    // This will allow the main loop to execute the sampling code.
    active = true;
    sampling = true;
    // Prepare to resume sampling.
    enableTimer();
    // Set an interrupt and alarm to stop sampling after the time specified by
    // samplingDuration has passed. If sleepDuration is 0, no interrupt is set.
    if (sleepDuration) {
      attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
      rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, samplingDuration, 0), DS3231_A1_Minute);
   }
  }
}

// =============================================================================

/*
 * Puts the device to sleep until being awakened by the RTC's alarm, which
 * should pull pin D2 low and trigger an interrupt. Arguments will be passed to
 * (and are the same as) rtc.setAlarm1. This sleep mode saves more power than
 * lightSleep
 */
void deepSleep(const DateTime wakeTime, Ds3231Alarm1Mode alarm_mode) {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif
  // When the alarm goes off, it will cause the SQW pin of the DS3231 to go low.
  rtc.setAlarm1(wakeTime, alarm_mode);
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

  // Set an interrupt and alarm to stop sampling after the time specified by
  // samplingDuration has passed. If sleepDuration is 0, no interrupt is set.
  if (sleepDuration) {
    attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
    rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, samplingDuration, 0), DS3231_A1_Minute);
  }
}


/*
 * Interrupt Service Routine used wake the device up from light sleep and tell
 * it to take a single sample.
 */
ISR(TIMER2_OVF_vect) {
  sampling = true;
}


/*
 * Interrupt Service Routine used to tell the device to stop sampling.
 */
void stopSamplingISR() {
  detachInterrupt(INTERRUPT_INTPIN);
  active = false;
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
void error(uint8_t flashes) {
  uint16_t duration = 1000 / flashes;
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


/*
 * Flash the error LED to warn the user.
 */
void warning(uint16_t duration, uint8_t flashes) {
  for (uint8_t i = 0; i < flashes; i++) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(duration);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(duration);
  }
  delay(500);
}
