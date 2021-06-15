#include <SPI.h>

#include "RTClib.h"

#define ECHO_TO_SERIAL 0
#define ERROR_LED_PIN 4

#define YEAR 2021
#define MONTH 06
#define DAY 14
#define HOUR 21
#define MINUTE 16
#define SECOND 0

RTC_DS3231 rtc;

void setup() {
#if ECHO_TO_SERIAL
  Serial.begin(9600);
#endif
  pinMode(ERROR_LED_PIN, OUTPUT);

  if (!rtc.begin()) {
#if ECHO_TO_SERIAL
    Serial.println(F("RTC setup error"));
#endif
    digitalWrite(ERROR_LED_PIN, HIGH);
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(YEAR, MONTH, DAY, HOUR, MINUTE, SECOND));
  } else {
    while (1);
  }

#if ECHO_TO_SERIAL
  Serial.print("Time set to ");
  Serial.print(YEAR);
  Serial.print("-");
  Serial.print(MONTH);
  Serial.print("-");
  Serial.print(DAY);
  Serial.print(" @ ");
  Serial.print(HOUR);
  Serial.print(":");
  Serial.print(MINUTE);
  Serial.print(":");
  Serial.println(SECOND);
#endif
}

void loop() {
  digitalWrite(ERROR_LED_PIN, HIGH);
  delay(500);
  digitalWrite(ERROR_LED_PIN, LOW);
  delay(500);
}