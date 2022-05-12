
#include <driver/adc.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp32/ulp.h>

// Additional includes for peripheral devices
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05

#include "hulp.h"

#define ECHO_TO_SERIAL 1

#define SD_CS_PIN GPIO_NUM_2
#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25

#define BUFFER_SIZE 5


typedef struct {
    uint32_t timestamp;
    float pressure;
    int8_t temperature;
} __attribute__((packed)) entry_t;

RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

RTC_DATA_ATTR uint8_t oldDay = 0;

RTC_DATA_ATTR entry_t buffer[BUFFER_SIZE];
RTC_DATA_ATTR uint16_t bufferCount = 0;


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

    RTC_DS3231 rtc;
    DateTime now;
    switch (esp_reset_reason()) {   

        case ESP_RST_POWERON: {
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
                    Serial.flush();
                #endif
            }

            // Initialize the connection with the SD card.
            if (!SD.begin(SD_CS_PIN)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("SD setup error"));
                    Serial.flush();
                #endif
            }

            // Initialize the connection with the MS5803-05 pressure sensor.
            if (!sensor.initializeMS_5803(false)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("MS5803-05 sensor setup error"));
                    Serial.flush();
                #endif
            }
            
            // Disable power to the DS3231's VCC
            digitalWrite(RTC_POWER_PIN, LOW);

            #if ECHO_TO_SERIAL
                Serial.println("Going to sleep...");
                Serial.flush();
            #endif

            // Sleep for ~1 second (internal RTC is not super accurate)
            esp_deep_sleep(1000000);

            break; // switch (we don't actually reach this because of sleep)
        }

        case ESP_RST_DEEPSLEEP: {
            uint64_t start = micros();
            #if ECHO_TO_SERIAL    
                Serial.println("IM AWAAAAKE");
                Serial.flush();
            #endif

            // The I2C pull-up resistors are on the DS3231 board which is pin
            // powered, so that pin must be powered for I2C to work with the 
            // MS5803.
            digitalWrite(RTC_POWER_PIN, HIGH);
            // delay(1);

            // I2C communication needs to be reinitialized after a deep sleep
            // reset. The sensor.initializeMS_5803() method could be used here
            // instead but it would add unecessary initialization steps.
            // Wire.begin(); <-- UNCOMMENT IF NOT CALLED ELSEWHERE i.e. by rtc.begin() 
            rtc.begin();
            now = rtc.now();

            sensor.resetSensor();
            sensor.readSensor();

            #if ECHO_TO_SERIAL
                Serial.printf("Time: %d-%02d-%02d %02d:%02d:%02d \tPressure: %f \tTemp: %d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), sensor.pressure(), (int8_t) sensor.temperature());
                Serial.flush();
            #endif

            buffer[bufferCount] = {
                .timestamp = now.unixtime(),
                .pressure = sensor.pressure(),
                .temperature = (int8_t) sensor.temperature()
            };
            bufferCount++;
            if (bufferCount == BUFFER_SIZE) {
                #if ECHO_TO_SERIAL
                    Serial.printf("Dumping to card...\n");
                    Serial.flush();
                #endif

                if (!SD.begin(SD_CS_PIN)) {
                    #if ECHO_TO_SERIAL
                        Serial.println("Failed to reestablish SD");
                        Serial.flush();
                    #endif
                    break;
                }

                #if ECHO_TO_SERIAL
                    Serial.printf("Card Size: %d\n", SD.cardSize());
                #endif

                char fName[] = "/TEST";//_YYYYMMDD-hhmm.csv";
                //now.toString(fName);
                File f = SD.open(fName, "w", true);
                if (f == NULL) {
                    #if ECHO_TO_SERIAL
                        Serial.println("Failed to open file");
                        Serial.flush();
                    #endif
                    break;
                }
                size_t written = f.write((uint8_t*) buffer, bufferCount * sizeof(entry_t));
                #if ECHO_TO_SERIAL
                    Serial.printf("Wrote %lu bytes to file\n", written);
                    Serial.flush();
                #endif

                bufferCount = 0;
            }

            digitalWrite(RTC_POWER_PIN, LOW);

            #if ECHO_TO_SERIAL
                Serial.printf("Going to sleep...\n");
                Serial.flush();
            #endif

            esp_deep_sleep(1000000 - (micros() - start));

            break; // switch
        }

        default: 
            #if ECHO_TO_SERIAL
                Serial.println("RESET REASON CASE NOT HANDLED");
                Serial.flush();
            #endif
    }
    #if ECHO_TO_SERIAL
        Serial.println("CONTROL EXIT");
    #endif
}

void loop() {}