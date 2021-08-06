// =============================================================================
/* 
 * The original code is the work of the Mississippi State University Coastal 
 * Research & Extension Center and can be found in the DIY Wave Gauge project
 * found here:
 * http://coastal.msstate.edu/waves
 */
// =============================================================================

#include <Wire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "SdFat.h"
#include "RTClib.h" // https://github.com/adafruit/RTClib
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library

//** The following four config values will be used when no config file is found.
//** If a config file is found, these values will be ignored.
// The device will wait until this minute value (between 0 and 59) to begin. 
// Setting a value greater than 59 will cause the device to start immediately.
#define DEFAULT_START_MINUTE 60
// Length of time to sample for.
#define DEFAULT_SAMPLING_DURATION 60
// Length of time to sleep after sampling for the above duration. Set to 0 for 
// continuous sampling.
#define DEFAULT_SLEEP_DURATION 0
// Specify an info string to be included in each csv file.
#define DEFAULT_INFO_STRING "DIY1 Default Info String"
#define INFO_STRING_SIZE 64

#define OUTPUT_FILE_NAME "ms1-YYYYMMDD-hhmm.csv"
#define CONFIG_FILE_NAME "config.txt"
// Default number of samples to take per second (NOT IMPLEMENTED)
//#define SAMPLES_PER_SECOND 1
// Number of decimal places to keep for the pressure readings.
#define PRECISION 2

// Set to 1 to have info appear on the Serial Monitor when plugged into a 
// computer. Disable during deployment, (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 0

// This pin is used for detecting an alarm from the RTC and triggering an
// interrupt to wake the device up.
#define INTERRUPT_PIN 2 
// Equivalent to digitalPinToInterrupt(2). Used in attachInterrupt() calls.
#define INTERRUPT_INTPIN 0
// Chip Select pin for the SD card reader.
#define SD_CS_PIN 10
// BLINK DESCRIPTION
#define ERROR_LED_PIN LED_BUILTIN

// =============================================================================

// Real Time Clock object.
RTC_PCF8523 rtc;
// Used for writing data to the current day's log file on the SD card.
SdFat sd;
SdFile logFile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_HIGH);

DateTime now;
DateTime stopSampling;


// Track the current day value to determine when a new day begins and when a new
// CSV file must be started.
uint8_t oldDay = 0;

char filename[] = OUTPUT_FILE_NAME;

// These values will be obtained from the config.txt file on the SD card if it
// exists. Otherwise, they will be set to the corresponding default values 
// above.
TimeSpan samplingDuration = TimeSpan(DEFAULT_SAMPLING_DURATION * 60);
uint16_t sleepDuration = DEFAULT_SLEEP_DURATION;
char infoString[INFO_STRING_SIZE] = DEFAULT_INFO_STRING;

// When true, the device will be able to take a sample during the main loop of
// the program.
volatile bool sampling;

// =============================================================================

void setup() {  
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

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
  rtc.deconfigureAllTimers();

  // Test connection with the SD. If it fails, the LED will light up.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
#if ECHO_TO_SERIAL
    Serial.println(F("SD setup error"));
#endif
    error(2);
  }

  // Initialize the MS5803 pressure sensor.
  sensor.reset();
  sensor.begin();
  
  // Used for reading data from the config file, if present.
  SdFile configFile;
  // The sensor will sleep until this minute value.
  uint16_t startMinute = DEFAULT_START_MINUTE;
  // Open the config file if it exists to obtain the start minute, data
  // sampling duration, sleep duration, and info string.
  if (configFile.open(CONFIG_FILE_NAME, O_READ)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Reading from config file:"));
#endif
    // Determines whether a number was read for each config variable. If one is
    // missing, we assume the config file is wrong and use all default values.
    bool foundNumberFlag;
    // An array to store the numeric configuration values.
    uint16_t configVars[3] = {0};
    // Reading in a number from the first 3 lines of the file:
    char c;
    for (uint8_t i = 0; i < 3; i++) {
      foundNumberFlag = false;
      // Reading a single character.
      c = configFile.read();
      // Build the number by reading one digit at a time until reaching a 
      // non-numeric character.
      while (c >= '0' && c <= '9') {
        foundNumberFlag = true;
        configVars[i] = (10 * configVars[i]) + (uint16_t)(c - '0');
        c = configFile.read();
      }

      if (!foundNumberFlag) {
#if ECHO_TO_SERIAL
        Serial.println(F("Failed to read all config variables. Using default values"));
#endif
        warning(500, 2);
        break;
      }

      // Read until the current line ends or the file ends.
      while (c != '\n' && c != EOF) {
        c = configFile.read();
      }
    }
    // Look for the info string if we still haven't reach the end of the file
    // and all other config values have been found. Otherwise, use the default.
    if (c != EOF && foundNumberFlag) {
      // Read up to 63 characters on the current line to form the info string.
      uint8_t i = 0;
      do {
        c = configFile.read();
        infoString[i++] = c;
      } while ((c != '\n' && c != EOF) && (i < INFO_STRING_SIZE));
      infoString[i - 1] = '\0';
    } else {
#if ECHO_TO_SERIAL
      Serial.println(F("No info string found. Using default info string"));
#endif
    }

    configFile.close();

    // Set these values based on the corresponding config values.
    startMinute = configVars[0]; // 1st line in file.
    samplingDuration = TimeSpan(configVars[1] * 60); // 2nd line in file.
    sleepDuration = configVars[2]; // 3rd line in file.
  } else {
#if ECHO_TO_SERIAL
    Serial.println(F("No config file found. Using default values"));
#endif
    warning(500, 2);
  }

#if ECHO_TO_SERIAL
  Serial.print(F("startMinute = "));
  Serial.println(startMinute);
  Serial.print(F("samplingDuration = "));
  Serial.println(samplingDuration.totalseconds() / 60);
  Serial.print(F("sleepDuration = "));
  Serial.println(sleepDuration);
  Serial.print(F("infoString = "));
  Serial.println(infoString);
#endif

  now = rtc.now();

  // If a startMinute value between 0 and 59 is set, the device will go into 
  // deep sleep until the current minute matches specified minute value.
  // Otherwise, the device will start right away.
  if ((startMinute < 60) && (now.minute() != startMinute)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Waiting for startMinute"));
    Serial.flush();
#endif
    longSleep((startMinute - now.minute() + 60) % 60);
  }

#if ECHO_TO_SERIAL
  Serial.println(F("\nSetup complete, beginning sampling"));
  Serial.flush();
#endif

  sampling = true;
  stopSampling = rtc.now() + samplingDuration;
  enableTimer();
}

void loop() {  
  now = rtc.now();

  // Start a new CSV file each day.
  if (oldDay != now.day()) {
    strcpy(filename, OUTPUT_FILE_NAME);

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
    logFile.write("samplingDuration,sleepDuration,infoString\n");
    logFile.printField(samplingDuration.totalseconds() / 60, ',');
    logFile.printField(sleepDuration, ',');
    logFile.write(infoString);
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

  if (now < stopSampling || sleepDuration == 0) {
    if (sampling) {
      float pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      
      // Write to SD card:
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

      sampling = false;
    }

    shortSleep();
  } else {
    disableTimer();
    logFile.close();

    longSleep(sleepDuration);

    logFile.open(filename, O_WRITE | O_AT_END);
    sampling = true;
    stopSampling = rtc.now() + samplingDuration;
    enableTimer();
  }
}

void enableTimer() {
  rtc.enableSecondTimer();
  attachInterrupt(INTERRUPT_INTPIN, takeSampleISR, FALLING);
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

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
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
