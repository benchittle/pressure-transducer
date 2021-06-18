#include <Wire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "SdFat.h" // https://github.com/greiman/SdFat
#include "RTClib.h" // https://github.com/adafruit/RTClib
#include "SparkFun_MS5803_I2C.h" // https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library


//** The following four config values will be used when no config file is found.
//** If a config file is found, these values will be ignored.
// The device will wait until this minute value (between 0 and 59) to begin. 
// Setting a value greater than 59 will cause the device to start immediately.
#define DEFAULT_START_MINUTE 60
// Length of time to sample for.
#define DEFAULT_SAMPLING_DURATION 17
// Length of time to sleep after sampling for the above DURATION. Set to 0 for 
// continuous sampling.
#define DEFAULT_SLEEP_DURATION 0
// Specify an info string to be included in each csv file.
#define DEFAULT_INFO_STRING "BO Default Info String"
#define INFO_STRING_SIZE 64

#define OUTPUT_FILE_NAME "bo-YYYYMMDD-hhmm.csv"
#define CONFIG_FILE_NAME "config.txt"
// Default number of samples to take per second (NOT IMPLEMENTED)
//#define SAMPLES_PER_SECOND 1
// Number of decimal places to keep for the pressure readings.
#define PRECISION 2

// Set to 1 to have info appear on the Serial Monitor when plugged into a 
// computer. Disable during deployment, (set to 0) in order to save battery.
#define ECHO_TO_SERIAL 1

// Chip Select pin for the SD card reader.
#define SD_CS_PIN 10
// BLINK DESCRIPTION
#define ERROR_LED_PIN 5
#define LED_PIN 6
#define REED_PIN 3
#define BUZZER_PIN 7
#define BUZZER_PORT PORTD
#define BUZZER_PORT_PIN PORTD7
#define HEARTBEAT_INTPIN 1


// =============================================================================

// Real Time Clock object.
RTC_DS3231 rtc;
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
// exists. Otherwise, the corresponding default values will be used.
TimeSpan samplingDuration = TimeSpan(DEFAULT_SAMPLING_DURATION * 60);
TimeSpan sleepDURATION = TimeSpan(DEFAULT_SLEEP_DURATION * 60);
char infoString[INFO_STRING_SIZE] = DEFAULT_INFO_STRING;

// The device takes a sample whenver it is active and sampling. If it is active
// but not sampling, then it will go into a light sleep.
volatile bool sampling = true;

bool noSleep = false;
bool heartBeatFlag = false;
uint8_t heartBeatCount = 0;
volatile uint16_t timer1ToggleCount;

void setup() {
  // Code between these if statements will only be run if ECHO_TO_SERIAL is not 0.
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

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

  // Used for reading data from the config file, if present.
  SdFile configFile;
  // The sensor will sleep until this minute value.
  uint16_t startMinute = DEFAULT_START_MINUTE;
  // Open the config file if it exists to obtain the start minute, data
  // sampling DURATION, sleep DURATION, and info string.
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
        Serial.println(F("Failed to read all config variables. Using default values:"));
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
      Serial.println(F("No info string found. Using default info string:"));
#endif
    }

    configFile.close();

    // Set these values based on the corresponding config values.
    startMinute = configVars[0]; // 1st line in file.
    samplingDuration = TimeSpan(configVars[1] * 60); // 2nd line in file.
    sleepDURATION = TimeSpan(configVars[2] * 60); // 3rd line in file.
  } else {
#if ECHO_TO_SERIAL
    Serial.println(F("No config file found. Using default values:"));
#endif
    //warning(500, 2);
  }

#if ECHO_TO_SERIAL
  Serial.print(F("startMinute = "));
  Serial.println(startMinute);
  Serial.print(F("samplingDuration = "));
  Serial.println(samplingDuration.totalseconds() / 60);
  Serial.print(F("sleepDuration = "));
  Serial.println(sleepDURATION.totalseconds() / 60);
  Serial.print(F("infoString = "));
  Serial.println(infoString);
#endif

  if (sleepDURATION.totalseconds() == 0) {
    noSleep = true;
  }

  now = rtc.now();

  // If a startMinute value between 0 and 59 is set, the device will go into 
  // deep sleep until the current minute matches specified minute value.
  // Otherwise, the device will start right away.
  if ((startMinute < 60) && (now.minute() != startMinute)) {
#if ECHO_TO_SERIAL
    Serial.println(F("Waiting for startMinute"));
    Serial.flush();
#endif
    deepSleep(now + TimeSpan(0, 0, (startMinute - now.minute() + 60) % 60, 0));
  }

#if ECHO_TO_SERIAL
  Serial.println(F("Setup complete, beginning sampling"));
  Serial.flush();
#endif
  // Tell the user the device was initialized properly.
  //warning(3000, 1);

  // Prepare to begin sampling.
  heartBeatFlag = true;
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
    logFile.printField(sleepDURATION.totalseconds() / 60, ',');
    logFile.write(infoString);
    logFile.write("\ndate,time,pressure,temperature\n");
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
  if (noSleep || now < stopSampling) {
    if (sampling) {
      double pressure = sensor.getPressure(ADC_4096);
      int temperature = sensor.getTemperature(CELSIUS, ADC_512); 
          
      logFile.printField(now.year(), '-');
      logFile.printField(now.month(), '-');
      logFile.printField(now.day(), ',');
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
    // Enter light sleep briefly while waiting to take the next sample.
    lightSleep();

  } else {
    disableTimer();
    logFile.close();

    // Enter deep sleep for the specified sleepDURATION.
    deepSleep(rtc.now() + sleepDURATION);

    logFile.open(filename, O_WRITE | O_AT_END);
    // Prepare to resume sampling.
    sampling = true;
    stopSampling = rtc.now() + samplingDuration;
    enableTimer();
  }
  if (heartBeatFlag) {
    heartBeat(2000, 15);
  }

  
}


void deepSleep(const DateTime wakeTime) {
  DateTime nearWakeTime = DateTime(wakeTime.secondstime() - 9);
  while (rtc.now() < nearWakeTime) {
    if (heartBeatFlag) {
      heartBeat(2000, 15);
    }
    /* It seems to be necessary to zero out the Asynchronous clock status 
    * register (ASSR) before enabling the watchdog timer interrupts in this
    * process. 
    */
    ASSR = 0;  
    // Cannot re-enter sleep mode within one TOSC cycle. 
    // This provides the needed delay.
    OCR2A = 0; // write to OCR2A, we're not using it, but no matter
    while (ASSR & _BV(OCR2AUB)); // wait for OCR2A to be updated

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // specify sleep mode
    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts();
    sleep_enable();
    // Set up Watchdog timer for long term sleep

    // Clear the reset flag first
    MCUSR &= ~_BV(WDRF);

    // In order to change WDE or the prescaler, we need to
    // set WDCE (This will allow updates for 4 clock cycles).
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    // Enable watchdog interrupt (WDIE), and set 8 second delay
    WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0); 
    wdt_reset();

    // Turn off brown-out enable in software
    sleep_bod_disable();
    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts();  // one cycle, re-enables interrupts
    sleep_cpu();   // one cycle, going to sleep now, wake on interrupt

    // The AVR is now asleep. In SLEEP_MODE_PWR_DOWN it will only wake
    // when the watchdog timer counter rolls over and creates an interrupt
    // disable sleep as a precaution after waking
    sleep_disable();
  }

  while(rtc.now() < wakeTime) {
    delay(500);
  }
}

// Interrupt to run when the watchdog timer finishes a cycle. 
ISR(WDT_vect) {
  wdt_disable();
}

/*
 * Begin using the 32KHz output of the DS3231 as an external clock source for
 * the MCU's Timer/Counter2. This timer is then set to trigger interrupts at
 * regular intervals, depending on the sampling FREQUENCY. We use the DS3231's
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
  sampling = true;
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
 * faster to wake up from when higher FREQUENCY sampling is needed.
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


//-----------------------------------------------------------------------------
// Interrupt Service Routine for INT1 (should be a reed switch)
// This function sets the global heartBeatFlag true, which will trigger actions
// in the main loop to notify the user that the datalogging is still happening.
void heartBeatInterrupt() {
	// If the interrupt is triggered, set the flag to 1 (true)
	heartBeatFlag = 1;
	// Immediately detach the interrupt on INT1 so that it doesn't 
	// trigger repeatedly. 
	detachInterrupt(1);
}

//-------------------------------------------------------------------
// heartBeat function. This function plays a tone on the buzzer attached
// to AVR pin PD7 and flashes the LED to notify the user that datalogging
// is still happening, or will happen again on schedule. 

void heartBeat(uint16_t frequency, uint16_t duration){
  if (heartBeatCount < 10){
    beepBuzzer(frequency, duration); // Play tone on Arduino pin 7 (PD7)
    heartBeatCount++; // increment counter
  } else {
    // If the heartbeat has executed 10 times, shut if off,
    // reactivate the heartbeat interrupt, and reset the counter
    heartBeatFlag = 0;
    // Register the heartBeatInterrupt service routine on INT1, 
    // triggered whenever INT1 is pulled low.
    attachInterrupt(1, heartBeatInterrupt, LOW);
    heartBeatCount = 0;
  }
}

//-------------------------------------------------------------------------
// Function beepBuzzer()
// This function uses TIMER1 to toggle the BUZZER pin to drive a piezo 
// buzzer. The frequency and duration of the noise are defined as global
// variables at the top of the program. This function exists in place of
// the normal Arduino tone() function because tone() uses TIMER2, which 
// interferes with the 32.768kHz timer used to clock the data logging. 
void beepBuzzer(uint16_t frequency, uint16_t duration){
	// Reset the 16 bit TIMER1 Timer/Counter Control Register A
  TCCR1A = 0;
	// Enable Clear Timer on Compare match mode by setting the bit WGM12 to 
	// 1 in TCCR1B
  // Set the Clock Select bit 10 to 1, which sets no prescaling
  TCCR1B = _BV(WGM12) | _BV(CS10);
	
  // Set the OCR for the given timer,
  // set the toggle count,
  // then turn on the interrupts
  OCR1A = F_CPU / frequency / 2 - 1; // Store the match value that will trigger a TIMER1 interrupt
  timer1ToggleCount = 2 * frequency * duration / 1000; // Toggle count (duration > 0)
  TIMSK1 |= _BV(OCIE1A); // Set OCEIE1A bit in TIMSK1 to 1.
	// At this point, TIMER1 should now be actively counting clock pulses, and
	// throwing an interrupt every time the count equals the value of ocr stored
	// in OCR1A above. The actual toggling of the pin to make noise happens in the 
	// TIMER1_COMPA_vect interrupt service routine. 
}

//-----------------------------------------------------
// Interrupt service routine for TIMER1 counter compare match
// This should be toggling the buzzer pin to make a beep sound
ISR(TIMER1_COMPA_vect) {
  if (timer1ToggleCount > 0) {
    // toggle the pin
    BUZZER_PORT ^= _BV(BUZZER_PORT_PIN);
    timer1ToggleCount--;
  } else {
    // Set Output Compare A Match Interrupt Enable (OCIE1A) bit to zero
    // in the TIMSK1 (Timer/Counter1 Interrupt Mask Register) to disable
    // the interrupt on compare match. 
    TIMSK1 &= ~_BV(OCIE1A); 
    BUZZER_PORT &= ~_BV(BUZZER_PORT_PIN);  // keep pin low after stop
  }
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













