#include <Wire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "SdFat.h"
// YOU NEED TO UNCOMMENT LINE 10 IN THE config.h FILE FOR THIS LIBRARY
#include "ds3231.h" // https://github.com/rodan/ds3231
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library


//** The following four config values will be used when no config file is found.
//** If a config file is found, these values will be ignored.

// The device will wait until this time to begin. Setting a value in the past 
// will cause the device to start immediately.
// TODO: Fix this
 //#define DEFAULT_START_TIME DateTime(2000, 1, 1, 0, 0)
// Length of time to sample for before sleeping. Must be <= 1440. This value is
// ignored if DEFAULT_SLEEP_DURATION is 0
#define DEFAULT_SAMPLING_DURATION 60
// Length of time to sleep after sampling for the above duration. Set to 0 for 
// continuous sampling. Must be <= 1440
#define DEFAULT_SLEEP_DURATION 0
// Specify a name for the sensor to use when creating logging files.
#define DEFAULT_SENSOR_NAME "MS2"
// Maximum number of characters for sensor name.
#define SENSOR_NAME_SIZE 10

// NAME_YYYYMMDD-hhmm.data is the format.
#define FILE_NAME_FORMAT "%s_%04d%02d%02d-%02d%02d.data"
// The format specifiers take up more room than the formatted string will, so we 
// subtract the extra space from the size. Then add room for %s part.
#define FILE_NAME_SIZE (sizeof(FILE_NAME_FORMAT) - 11 + SENSOR_NAME_SIZE)

#define CONFIG_FILE_NAME "config.txt"
// Default number of samples to take per second (NOT IMPLEMENTED)
//#define SAMPLES_PER_SECOND 1

// Set to 1 to have info appear on the Serial Monitor when plugged into a 
// computer. Disable during deployment, (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 1
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

typedef struct {
    uint32_t timestamp;
    float pressure;
    int8_t temperature;
} __attribute__((packed)) entry_t;

// Used for storing time data from RTC.
ts now;
// Flags for setting RTC alarm so the alarm will look for a match in the hours, 
// minutes, and seconds values.
const uint8_t rtcFlags[] = {0, 0, 0, 1, 0};

// Used for writing data to the current day's log file on the SD card.
SdFat sd;
SdFile logFile;
// Pressure sensor object.
MS5803 sensor(ADDRESS_LOW);

// Track the current day value to determine when a new day begins and when a new
// CSV file must be started.
uint8_t oldDay = 0;

// Name of the current log file.
char filename[FILE_NAME_SIZE];

// These values will be obtained from the config.txt file on the SD card if it
// exists. Otherwise, the corresponding default values will be used. See the
// default value definitions above for the meaning of each.
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

  // Initialize connection with RTC. 
  // TODO: If it fails, then the error LED will blink once per second until the
  // device is restarted.
  Wire.begin();
  DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
  // If the RTC lost power and therefore lost track of time, it will light up 
  // the LED for 1 second. While it is not necessary that the RTC has the right
  // time, if multiple sensors are being deployed and are meant to start at the
  // same time, this will warn the user that the sensors are not synchronized.
  if (DS3231_get_sreg() & DS3231_STATUS_OSF) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC time not configured (lost power)"));
#endif
    warning(500, 1);
  }
  // Alarms should be disabled initially.
  DS3231_clear_a1f();
  DS3231_clear_a2f();

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
      // TODO: Fix startTime
      // startTime = DateTime(dateTime[0], dateTime[1], dateTime[2], dateTime[3], dateTime[4]); // 1st line in file.
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
  Serial.print(F("startTime = \n"));
  /*Serial.print(startTime.year());
  Serial.print("-");
  Serial.print(startTime.month());
  Serial.print("-");
  Serial.print(startTime.day());
  Serial.print(F(" @ "));
  Serial.print(startTime.hour());
  Serial.print(F(":"));
  Serial.println(startTime.minute());*/
  Serial.print(F("samplingDuration = "));
  Serial.println(samplingDuration);
  Serial.print(F("sleepDuration = "));
  Serial.println(sleepDuration);
  Serial.print(F("sensorName = "));
  Serial.println(sensorName);
#endif
  
  /*
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
  */

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
  // Get the current time from the RTC and store it in the variable "now".
  DS3231_get(&now);

  // Start a new CSV file each day.
  if (oldDay != now.mday) {

    // Generate the file name based on the provided sensor name and the 
    // current time.
    snprintf(filename, FILE_NAME_SIZE, FILE_NAME_FORMAT, sensorName, now.year, now.mon, now.mday, now.hour, now.min, now.sec);

    // Set the timestamp on the old log file (if one was open).
    if (logFile.isOpen()) {
      logFile.timestamp(T_WRITE, now.year, now.mon, now.mday, now.hour, now.min, now.sec);
      logFile.close();
    } 

    // If there is an error creating the new file, the device will stop and 
    // blink three times per second until being restarted.
    if (!logFile.open(filename, O_WRITE | O_CREAT | O_AT_END)) {
#if ECHO_TO_SERIAL
      Serial.println(F("Couldn't create file."));
#endif
      error(3);
    }
#if ECHO_TO_SERIAL
    Serial.print(F("Starting new file: ")); 
    Serial.println(filename);
    Serial.flush();
#endif
    oldDay = now.mday;
  }
  

  // If active, the device will see whether it's time to take a sample.
  // Otherwise, enter deep sleep.
  if (active) {
    if (sampling) {
      entry_t data = {
          .timestamp = now.unixtime,
          .pressure = sensor.getPressure(ADC_4096),
          .temperature = (int8_t) sensor.getTemperature(CELSIUS, ADC_512)
      };
          
      logFile.write(&data, sizeof(data));
      logFile.sync();
      
#if ECHO_TO_SERIAL
      Serial.print(now.year);
      Serial.print(F("-"));
      Serial.print(now.mon);
      Serial.print(F("-"));
      Serial.print(now.mday);
      Serial.print(F(" @ "));
      Serial.print(now.hour);
      Serial.print(F(":"));
      Serial.print(now.min);
      Serial.print(F(":"));
      Serial.print(now.sec);
      Serial.print(F(", "));
      Serial.print(data.pressure);
      Serial.print(F(", "));
      Serial.println(data.temperature);
      Serial.flush();
#endif //ECHO_TO_SERIAL

#if ECHO_TO_PLOT
      Serial.println(data.pressure);
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
    deepSleep(sleepDuration);

    logFile.open(filename, O_WRITE | O_AT_END);
    // This will allow the main loop to execute the sampling code.
    active = true;
    sampling = true;
    // Prepare to resume sampling.
    enableTimer();
  }
}

// =============================================================================

/*
 * Puts the device to sleep until being awakened by the RTC's alarm, which
 * should pull pin D2 low and trigger an interrupt. "minutes" is the time in
 * minutes to sleep for. It must be less than one day.
 */
void deepSleep(uint16_t minutes) {
#if ECHO_TO_SERIAL
  Serial.println(F("Entering deep sleep"));
  Serial.flush();
#endif

  // Get the current time.
  DS3231_get(&now);
  // Calculate the time after the specified number of minutes and set the alarm.
  // When the alarm activates, it will cause the SQW pin of the DS3231 to go 
  // low.
  DS3231_set_a1(now.sec, (now.min + minutes) % 60, (now.hour + (now.min + minutes) / 60) % 24, 0, rtcFlags);
  // Enable the alarm.
  DS3231_set_creg(DS3231_get_creg() | DS3231_CONTROL_A1IE);

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
  // deepSleepISR function.

  // Clear the alarm and disable it for now.
  DS3231_set_creg(DS3231_get_creg() & ~DS3231_CONTROL_A1IE);
  DS3231_clear_a1f();
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
  DS3231_set_32kHz_output(true);

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
    // Get the current time.
    DS3231_get(&now);
    // Attach the interrupt that will trigger when the alarm activates.
    attachInterrupt(INTERRUPT_INTPIN, stopSamplingISR, LOW);
    // Set the alarm.
    DS3231_set_a1(now.sec, (now.min + samplingDuration) % 60, (now.hour + (now.min + samplingDuration) / 60) % 24, 0, rtcFlags);
    // Enable the alarm.
    DS3231_set_creg(DS3231_get_creg() | DS3231_CONTROL_A1IE);
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
  DS3231_set_32kHz_output(false); // Disable RTC 32khz
  // Disable the alarm on the RTC.
  DS3231_set_creg(DS3231_get_creg() & ~DS3231_CONTROL_A2IE);
  // Clear the alarm on the RTC
  DS3231_clear_a1f();
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
