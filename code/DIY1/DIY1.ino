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
#include <SD.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "RTClib.h" // https://github.com/adafruit/RTClib
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library

// The device will wait until this minute value to begin. Set a value less than
// 0 to sample right away.
#define START_MINUTE 60

// Default length of time to sample for.
#define SAMPLING_DURATION 1
// Default length of time to sleep after sampling for the above length of time.
// Set to 0 for continuous sampling.
#define SLEEP_DURATION 1
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

// =============================================================================

// Real Time Clock object.
RTC_PCF8523 rtc;
// Used for writing data to the current day's log file on the SD card.
File logfile;
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
  if (!SD.begin(SD_CS_PIN)) {
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
  
  if (SD.exists("config4.txt")) {
#if ECHO_TO_SERIAL
    Serial.println(F("Reading from config file:"));
#endif ECHO_TO_SERIAL
    logfile = SD.open("confign.txt", FILE_READ);
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
    startMinute = START_MINUTE;
    samplingDuration = TimeSpan(SAMPLING_DURATION * 60);
    sleepDuration = SLEEP_DURATION;
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
    char filename[] = "YYYYMMDD.csv"; // The file name will follow this format.
    
    logfile.close();
    
    // If there was an error creating the new log, the device will stop and 
    // blink three times per second until being restarted.
    logfile = SD.open(now.toString(filename), FILE_WRITE);
    if (!logfile) {
#if ECHO_TO_SERIAL
      Serial.println(F("Couldn't create file."));
#endif
      error();
    }
    // Print a header for the file.
    logfile.println("date,time,pressure,temperature\n");
    logfile.flush();
#if ECHO_TO_SERIAL
    Serial.print(F("Starting new file: ")); 
    Serial.println(filename);
    Serial.flush();
#endif
    oldDay = now.day();
  }
  digitalWrite(LED_BUILTIN, HIGH);
  if (now < stopSampling) {
    if (sampling) {
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
      
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

    longSleep(SLEEP_DURATION);

    sampling = true;
    stopSampling = rtc.now() + samplingDuration;
    enableTimer();
  }
  digitalWrite(LED_BUILTIN, LOW);
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
