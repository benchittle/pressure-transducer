
#include <driver/adc.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp32/ulp.h>

// Additional includes for peripheral devices
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "ds3231.h"
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05

#include "hulp.h"

#define ECHO_TO_SERIAL 1

#define SD_CS_PIN GPIO_NUM_2
#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25


typedef struct {
    uint32_t timestamp;
    float pressure;
    int8_t temperature;
} /*__attribute__((packed))*/ entry_t;

RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

RTC_DATA_ATTR uint8_t oldDay = 0;
RTC_DATA_ATTR struct ts now;

RTC_DATA_ATTR entry_t buffer[200];

RTC_DATA_ATTR const uint8_t rtcFlags[] = {1, 1, 1, 1, 1};


void printTime(struct ts* t) {
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", t->year, t->mon, t->mday, t->hour, t->min, t->sec);
}


void setup() {
    #if ECHO_TO_SERIAL
        Serial.begin(115200);
        delay(1000);
    #else
        // ADC, WiFi, BlueTooth are disabled by default.        
        // Default CPU frequency is 240MHz, but we don't need high speed.
        setCpuFrequencyMhz(10);     
    #endif

    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(RTC_POWER_PIN, OUTPUT);

    switch (esp_reset_reason()) {

        case ESP_RST_POWERON: {
            // Initialize the connection with the RTC:
            // Power to the RTC is provided by a GPIO pin. We need to wait a short 
            // duration (10ms seems to work) before connecting to the RTC or it fails.
            // This might be because it has to switch from battery power to VCC power.
            digitalWrite(RTC_POWER_PIN, HIGH);
            Wire.begin();
            // Initialize connection with the DS3231:
            // DS3231_CONTROL_INTCN disables any active square wave
            // DS3231_CONTROL_BBSQW enables square waves even in battery backup mode
            DS3231_init(DS3231_CONTROL_INTCN | DS3231_CONTROL_BBSQW);
            // According to the datasheet, the oscillator takes <1 sec to begin (on 
            // first time startup), so we wait 1 second in case this is the first time
            // startup. This accounts for the 10ms delay mentioned above, too.
            delay(1000); 

            // Initialize the connection with the SD card.
            if (!SD.begin(SD_CS_PIN)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("SD setup error"));
                    Serial.flush();
                #endif
            }

            // Initialize the connection with the MS5803-05 pressure sensor.
            if (!sensor.initializeMS_5803(true)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("MS5803-05 sensor setup error"));
                    Serial.flush();
                #endif
            }

            DS3231_set_a1(0, 0, 0, 0, rtcFlags);
            delay(1000);

            digitalWrite(RTC_POWER_PIN, LOW);

            esp_sleep_enable_ext0_wakeup(RTC_ALARM, 1);
            esp_deep_sleep_start();

            break; // switch (we don't actually reach this because of sleep)
        }

        case ESP_RST_DEEPSLEEP: {
            digitalWrite(RTC_POWER_PIN, HIGH);
            Wire.begin();
            

            Serial.println("IM AWAAAAKE");
            Serial.flush();

            // The I2C pull-up resistors are on the DS3231 board which is pin
            // powered, so that pin must be powered for I2C to work with the 
            // MS5803.
            
            // delay(1);

            // I2C communication needs to be reinitialized after a deep sleep
            // reset. The sensor.initializeMS_5803() method could be used here
            // instead but it would add unecessary initialization steps.
             

            Serial.printf("Reg: %x\n", DS3231_get_creg());

            sensor.resetSensor();      
                  
            sensor.readSensor();
            Serial.printf("Pressure: %f\n\n", sensor.pressure());
            Serial.flush();

            digitalWrite(RTC_POWER_PIN, LOW);

            esp_sleep_enable_ext0_wakeup(RTC_ALARM, 1);
            esp_deep_sleep_start();

            break; // switch
        }

        default: 
            #if ECHO_TO_SERIAL
                Serial.println("RESET REASON CASE NOT HANDLED");
                Serial.flush();
            #endif
    }
}

void loop() {}