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
SdFile logfile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_HIGH);

DateTime now;
DateTime stopSampling;
TimeSpan samplingDuration;

// Track the current day value to determine when a new day begins and when a new
// CSV file must be started.
uint8_t oldDay = 0;

// These values will be obtained from the config.txt file on the SD card if it
// exists. Otherwise, they will be set to the corresponding default values 
// above.
uint16_t sleepDuration;

// When true, the device will be able to take a sample during the main loop of
// the program.
volatile bool sampling;

// =============================================================================

void setup() {  
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  // Disable the Analog to Digital Converter to save power, as we don't use it
  // here (the MS5803 uses its own).
  ADCSRA = 0;
  ACSR = _BV(ACD); // Analog comparator.
  wdt_disable(); // Also disable the watchdog timer.

  // Test connection with RTC. If it fails, the LED will light up.
  if (!rtc.begin()) {
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif
    error();
  }
  rtc.deconfigureAllTimers();

  // Test connection with the SD. If it fails, the LED will light up.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
#if ECHO_TO_SERIAL
    Serial.println(F("SD setup error"));
#endif
    error();
  }

  sensor.reset();
  sensor.begin();
  
  // The sensor will sleep until this minute value in the current time.
  uint16_t startMinute;

  // Open the config file if it exists to obtain the start minute, data
  // sampling duration, and sleep duration.
  if (logfile.open("config.txt", O_READ)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Reading from config file:"));
#endif
    // An array to store the configuration values.
    uint16_t configVars[3] = {0};
    // Reading in a number from each line of the file:
    for (uint8_t i = 0; i < 3; i++) {
      char c = logfile.read();
      // Build the number by reading one digit at a time until reaching a 
      // non-numeric character.
      while (c >= '0' && c <= '9') {
        configVars[i] = (10 * configVars[i]) + (uint16_t)(c - '0');
        c = logfile.read();
      }
      // Read until the current line ends or the file ends.
      while (c != '\n' && c != EOF) {
        c = logfile.read();
      }
    }
    logfile.close();
    // Set these values based on the corresponding config values.
    startMinute = configVars[0]; // 1st line in file.
    samplingDuration = TimeSpan(configVars[1] * 60); // 2nd line in file.
    sleepDuration = configVars[2]; // 3rd line in file.
  } else {
#if ECHO_TO_SERIAL
    Serial.println(F("Using default config:"));
#endif ECHO_TO_SERIAL
    // Warn the user that default settings are being used by blinking for
    // 2 seconds twice.
    warning(2000, 2);
    startMinute = DEFAULT_START_MINUTE;
    samplingDuration = TimeSpan(DEFAULT_SAMPLING_DURATION * 60);
    sleepDuration = DEFAULT_SLEEP_DURATION;
  }
#if ECHO_TO_SERIAL
  Serial.print(F("startMinute = "));
  Serial.println(startMinute);
  Serial.print(F("samplingDuration = "));
  Serial.println(samplingDuration.totalseconds() / 60);
  Serial.print(F("sleepDuration = "));
  Serial.println(sleepDuration);
  Serial.flush();
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
  Serial.println(F("Setup complete, beginning sampling"));
  Serial.flush();
#endif

  sampling = true;
  enableTimer();
  stopSampling = rtc.now() + samplingDuration;
}

void loop() {  
  now = rtc.now();

  // Start a new CSV file each day.
  if (oldDay != now.day()) {
    char filename[] = "ms1-YYYYMMDD-hhmm.csv"; // The file name will follow this format.
    
    logfile.close();
    
    // If there was an error creating the new log, the device will stop and 
    // blink three times per second until being restarted.
    if (!logfile.open(now.toString(filename), O_WRITE | O_CREAT | O_AT_END)) {
#if ECHO_TO_SERIAL
      Serial.println(F("Couldn't create file."));
#endif
      error();
    }
    // Print a header for the file.
    logfile.write("date,time,pressure,temperature\n");
    logfile.sync();
#if ECHO_TO_SERIAL
    Serial.print(F("Starting new file: ")); 
    Serial.println(filename);
    Serial.flush();
#endif
    oldDay = now.day();
  }

  if (now < stopSampling) {
    if (sampling) {
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      
      // Write to SD card:
      logfile.printField(now.year(), '-');
      logfile.printField(now.month(), '-');
      logfile.printField(now.day(), ',');
      logfile.printField(now.hour(), ':');
      logfile.printField(now.minute(), ':');
      logfile.printField(now.second(), ',');
      logfile.printField(pressure, ',', PRECISION);
      logfile.printField(temperature, '\n');
      logfile.sync();

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

    longSleep(DEFAULT_SLEEP_DURATION);

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


// This void error setup is to define the error message we'll serial print and/or relay with LED lights if things aren't right
void error() {
  // red LED indicates error
  digitalWrite(LED_BUILTIN, HIGH);
  while(1);
}

/*
 * Flash the error LED to warn the user.
 */
void warning(uint16_t duration, uint8_t flashes) {
  for (uint8_t i = 0; i < flashes; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW);
    delay(duration);
  }
}
