#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include "MS5803_05.h"

#define ECHO_TO_SERIAL 1

#define SD_CS_PIN D9
#define RTC_POWER_PIN D3

RTC_DS3231 rtc;
DateTime now;

MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

void setup() {
    #if ECHO_TO_SERIAL
        Serial.begin(115200);
        delay(1000);
    #endif

    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(RTC_POWER_PIN, OUTPUT);

    // Initialize the connection with the RTC:
    // Power to the RTC is provided by a GPIO pin. We need to wait a short 
    // duration (10ms seems to work) before connecting to the RTC or it fails.
    // This might be because it has to switch from battery power to VCC power.
    digitalWrite(RTC_POWER_PIN, HIGH);
    // According to the datasheet, the oscillator takes <1 sec to begin (on 
    // first time startup), so we wait 1 second in case this is the first time
    // startup. This accounts for the 10ms delay mentioned above, too.
    delay(1000); 
    if (!rtc.begin()) {
        #if ECHO_TO_SERIAL
            Serial.println(F("RTC setup error"));
        #endif
    }

    now = rtc.now();
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
    Serial.println(now.second());

    // Initialize the connection with the SD card.
    if (!SD.begin(SD_CS_PIN)) {
        #if ECHO_TO_SERIAL
            Serial.println(F("SD setup error"));
        #endif
    }

    // Initialize the connection with the MS5803-05 pressure sensor.
    if (!sensor.initializeMS_5803(false)) {
        #if ECHO_TO_SERIAL
            Serial.println(F("MS5803-05 sensor setup error"));
        #endif
    }
    sensor.readSensor();
    Serial.println(sensor.pressure());

    Serial.println("Done");

    delay(1000);
    digitalWrite(RTC_POWER_PIN, LOW);
    esp_deep_sleep_start();
}

void loop() {}