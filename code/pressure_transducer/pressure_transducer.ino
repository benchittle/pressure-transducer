#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "RTClib.h"
#include "SparkFun_MS5803_I2C.h"

#define WAKEPIN 2
#define SD_CS_PIN 8


RTC_DS3231 rtc;
File sdfile;
MS5803 sensor(ADDRESS_HIGH);

DateTime now;
uint8_t old_day;

void setup() {
  Serial.begin(9600);
  pinMode(WAKEPIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!rtc.begin()) {
    Serial.println(F("Failed to start RTC!"));
    error();
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, date needs to be reset"));
    error();
  }
  rtc.clearAlarm(1);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD not connected properly"));
    error();
  }

  now = rtc.now();
  old_day = now.day();
}

void loop() {
  now = rtc.now();

  if (old_day != now.day()) {
    createNewCSV();
  }
  
  // sample at a given frequency for a given amount of time, lightly sleeping between each sample
    
    // read sensor

    // write datetime and reading to SD


  // If sampling is not continuous, go into deep sleep for the remainder of the time period

  // Create a new SD file every day

  
}

void createNewCSV() {
  
}

void error() {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(3000);
  }
}
