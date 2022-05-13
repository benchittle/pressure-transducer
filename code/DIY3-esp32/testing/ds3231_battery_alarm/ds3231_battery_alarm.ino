
#include "RTClib.h"

#define ECHO_TO_SERIAL 1

#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25

volatile boolean isInt = 0;

RTC_DS3231 rtc;

void IRAM_ATTR interrupt() {
    isInt = 1;
}


void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    delay(1000);

    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(D2, INPUT_PULLUP);
    digitalWrite(RTC_POWER_PIN, HIGH);
    delay(50);
    if (!rtc.begin()) {
        Serial.println("RTC Fail");
        return;
    }
    rtc.clearAlarm(1);
    rtc.disableAlarm(2);
    DateTime now = rtc.now();
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    rtc.setAlarm1(rtc.now(), DS3231_A1_PerSecond);

    attachInterrupt(RTC_ALARM, interrupt, FALLING);
}

void loop() {
    if (isInt) {
        rtc.clearAlarm(1);
        Serial.print("high @ ");
        Serial.println(millis());
        isInt = 0;
    }
}