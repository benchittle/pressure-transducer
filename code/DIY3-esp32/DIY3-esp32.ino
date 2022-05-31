
//#include <esp32/ulp.h>

// Additional includes for peripheral devices
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "ds3231.h" // https://github.com/rodan/ds3231
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05

//#include "hulp.h"

#define ECHO_TO_SERIAL 0

#define SD_CS_PIN GPIO_NUM_13 // TODO: Change this pin: FireBeetle uses it for LED
#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25

// /NAME_YYYYMMDD-hhmm.data is the format.
#define FILE_NAME_FORMAT "/%7s_%04d%02d%02d-%02d%02d.data"
// The format specifiers take up more room than the formatted string will, so we 
// subtract the extra space from the size. Then add room for DIY3-XX part.
#define FILE_NAME_SIZE (sizeof(FILE_NAME_FORMAT) - 11 + 7)

// Number of readings we can store in a buffer before we run out of RTC memory
// and need to dump to the SD card.
#define BUFFER_SIZE 400

// A custom struct to store a single data entry.
// NOTE: __attribute__((packed)) tells the compiler not to add additional 
// padding bytes that would align to the nearest 4 bytes. This can cause issues
// in some cases, but the ESP32 doesn't seem to have a problem with it. By doing
// this, we reduce the size of each entry from 12 bytes to 9 bytes.
typedef struct {
    uint32_t timestamp;
    float pressure;
    int8_t temperature;
} __attribute__((packed)) entry_t;

// Initialize a sensor object for interacting with the MS5803-05
RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

// Store the device's name (max 7 characters).
RTC_DATA_ATTR char deviceName[8] = {0};
// We need to store the name for the current data file across deep sleep 
// restarts.
RTC_DATA_ATTR char fileName[FILE_NAME_SIZE];
RTC_DATA_ATTR uint8_t oldDay = 0;

// An array to buffer data before saving it to the SD card. This reduces the
// number of writes and therefore energy consumption.
RTC_DATA_ATTR entry_t buffer[BUFFER_SIZE];
RTC_DATA_ATTR uint16_t bufferCount = 0;

void setup() {
    #if ECHO_TO_SERIAL
        Serial.begin(115200);
    #else
        // ADC, WiFi, BlueTooth are disabled by default.        
        // Default CPU frequency is 240MHz, but we don't need high speed.
        setCpuFrequencyMhz(10);     
    #endif

    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(RTC_ALARM, INPUT_PULLUP);

    switch (esp_reset_reason()) {   

        case ESP_RST_POWERON: {
            Wire.begin();
            // Initialize the connection with the RTC:
            // Power to the RTC is provided by a GPIO pin. 
            digitalWrite(RTC_POWER_PIN, HIGH);

            // According to the datasheet, the oscillator takes <1 sec to begin (on 
            // first time startup), so we wait 1 second in case this is the first time
            // startup. 
            delay(1000); 

            // Set BBSQW (battery backed square wave) bit in DS3231 control 
            // register. This allows us to generate alarm pulses while the chip
            // is powered only by battery. Also set other default values.
            // TODO: Need a way to make sure this succeeds.
            DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
            // Clear any previous alarms.
            DS3231_clear_a1f();
            DS3231_clear_a2f();

            // Set the current day value.
            struct ts timeNow;
            DS3231_get(&timeNow);
            oldDay = timeNow.mday;

            // Start the once-per-second alarm on the DS3231:
            // These flags set the alarm to be in once-per-second mode.
            const uint8_t flags[] = {1,1,1,1,0};
            // Set the alarm. The time value doesn't matter in once-per-second
            // mode.
            DS3231_set_a1(1, 1, 1, 1, flags);
            // Enable the alarm. It will now bring the RTC_ALARM GPIO low every
            // second.
            DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);


            // Initialize the connection with the SD card.
            if (!SD.begin(SD_CS_PIN)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("SD setup error"));
                    Serial.flush();
                #endif
            }

            // Get device's name from SD card or use default.
            File config = SD.open("/config.txt", FILE_READ, false);
            if (!config) {
                strcpy(deviceName, "DIY3-XX");
            } else {
                config.read((uint8_t*) deviceName, sizeof(deviceName) - 1);
                config.close();
            }
            #if ECHO_TO_SERIAL
                Serial.printf("Device Name: %s\n", deviceName);
            #endif
            
            // Generate the first file.
            snprintf(fileName, FILE_NAME_SIZE, FILE_NAME_FORMAT, deviceName, timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
            File f = SD.open(fileName, FILE_WRITE, true);
            if (!f) {
                #if ECHO_TO_SERIAL
                    Serial.println("Failed to create first file...");
                #endif
                break; // switch
            }
            f.close();
            SD.end();


            // Initialize the connection with the MS5803-05 pressure sensor.
            if (!sensor.initializeMS_5803(false)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("MS5803-05 sensor setup error"));
                    Serial.flush();
                #endif
            }
            
            // Disable power to the DS3231's VCC.
            digitalWrite(RTC_POWER_PIN, LOW);

            #if ECHO_TO_SERIAL
                Serial.println("Going to sleep...");
                Serial.flush();
            #endif

             // Make the ESP32 wake up when the alarm goes off (i.e. goes low). 
            esp_sleep_enable_ext0_wakeup(RTC_ALARM, 0);
            // Enter deep sleep.
            esp_deep_sleep_start();

            break; // switch (we don't actually reach this because of sleep)
        }

        case ESP_RST_DEEPSLEEP: {
            #if ECHO_TO_SERIAL    
                Serial.println("Awake!");
                Serial.flush();
            #endif

            // Reinitialize connection with I2C devices.
            Wire.begin();

            // Enable power to the RTC. This is done to avoid draining the coin
            // cell during I2C, but the power savings may be negligible.
            // TODO: Test battery life operating the RTC using ONLY coin cell.
            digitalWrite(RTC_POWER_PIN, HIGH);

            // Clear the RTC's alarm signal until the next second.
            DS3231_clear_a1f();

            // Get the time from the RTC.
            struct ts timeNow;
            DS3231_get(&timeNow);

            // Disable power to the RTC.
            digitalWrite(RTC_POWER_PIN, LOW);

            // Take a reading from the sensor.
            sensor.resetSensor();
            sensor.readSensor();

            #if ECHO_TO_SERIAL
                Serial.printf("Buffer: %d \tTime: %d-%02d-%02d %02d:%02d:%02d \tPressure: %f \tTemp: %d\n", bufferCount, timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec, sensor.pressure(), (int8_t) sensor.temperature());
                Serial.flush();
            #endif

            // Create a new data entry and add it to the buffer.
            buffer[bufferCount] = {
                .timestamp = timeNow.unixtime,
                .pressure = sensor.pressure(),
                .temperature = (int8_t) round(sensor.temperature())
            };
            bufferCount++;

            // If the buffer is full, dump it to the SD card.
            if (bufferCount == BUFFER_SIZE || oldDay != timeNow.mday) {
                #if ECHO_TO_SERIAL
                    Serial.println("Dumping to card...");
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
                    Serial.flush();
                #endif

                // TODO: Write buffered data before starting new day
                // Open a file for logging the data. If it's the first dump of
                // the day, start a new file.
                File f;
                if (oldDay != timeNow.mday) {
                    // Generate the file name with the current date and time.
                    snprintf(fileName, FILE_NAME_SIZE, FILE_NAME_FORMAT, deviceName, timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                    // Open a new file for the data.
                    f = SD.open(fileName, FILE_WRITE, true);
                    oldDay = timeNow.mday;
                } else {
                    // Open an existing file and append the data to it.
                    f = SD.open(fileName, FILE_APPEND, false);
                }
                if (!f) {
                    #if ECHO_TO_SERIAL
                        Serial.println("Failed to open file");
                        Serial.flush();
                    #endif
                    break; // switch
                }
                // Write the data buffer as a sequence of bytes (it's vital that
                // the entry_t struct is packed, otherwise there will be garbage
                // bytes in between each entry that will waste space). In order
                // to use this data later, we'll have to unpack it using a 
                // postprocessing script.
                size_t written = f.write((uint8_t*) buffer, bufferCount * sizeof(entry_t));
                
                f.close();
                SD.end();
                #if ECHO_TO_SERIAL
                    Serial.printf("Wrote %lu bytes to file\n", written);
                    Serial.flush();
                #endif

                // Reset the buffer count.
                bufferCount = 0;
            }

            #if ECHO_TO_SERIAL
                Serial.printf("Going to sleep...\n");
                Serial.flush();
            #endif

            // Make the ESP32 wake up when the alarm goes off (i.e. goes low). 
            esp_sleep_enable_ext0_wakeup(RTC_ALARM, 0);
            // Enter deep sleep.
            esp_deep_sleep_start();

            break; // switch (we don't actually reach this because of sleep)
        }

        default: 
            #if ECHO_TO_SERIAL
                Serial.println("RESET REASON CASE NOT HANDLED");
                Serial.flush();
            #endif
            break; // switch
    }
    #if ECHO_TO_SERIAL
        Serial.println("CONTROL EXIT");
    #endif
}

void loop() {}