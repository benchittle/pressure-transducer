#include <avr/sleep.h>
#include "RTClib.h"

#define WAKEPIN 2

RTC_DS3231 rtc;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WAKEPIN, INPUT_PULLUP);

  rtc.begin();
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.setAlarm1(rtc.now() + TimeSpan(10), DS3231_A1_Second);
  
  sleep();
  
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);

}

void sleep() {
  byte adcsra = ADCSRA;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(WAKEPIN), sleepISR, LOW);
  interrupts();
  sleep_cpu();  
  ADCSRA = adcsra;
}

void sleepISR() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(WAKEPIN));
}
