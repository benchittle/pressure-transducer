#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS_PIN D7 
#define SD_SWITCH_PIN A4

#define ERROR_LED_PIN GPIO_NUM_2

// Duration in ms for each flash of the LED when displaying an error / warning.
#define LED_FLASH_DURATION 500

// Error codes for the program. The value associated with each enum is also the
// number of times the error LED will flash if an error is encountered.
enum diy4_error_t {
    ds3231Error = 1,
    sdInitError,
    sdConfigWarning,
    sdFileError,
    ms5803Error,
    resetError
};

/*
 * Enter an endless loop while flashing the error LED a given number of times
 * each second.
 */
void error(diy4_error_t flashes) {
    while (1) {
        for (uint8_t i = 0; i < flashes; i++) {
            digitalWrite(ERROR_LED_PIN, HIGH);
            delay(LED_FLASH_DURATION);
            digitalWrite(ERROR_LED_PIN, LOW);
            delay(LED_FLASH_DURATION);
        }
        delay(1000);
    }
}

void setup() {
    delay(1000);

    Serial.begin(115200);

    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(SD_SWITCH_PIN, OUTPUT);
    pinMode(GPIO_NUM_27, INPUT_PULLUP);

    /*
    for (int i = 0; i < 5; i++) {
        digitalWrite(SD_SWITCH_PIN, LOW);
        delay(4000);
        digitalWrite(SD_SWITCH_PIN, HIGH);
        delay(4000);
    }
    */
    /*
    // Initialize the connection with the SD card.
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println(F("SD setup error"));
        Serial.flush();
        error(sdInitError);
    }
    Serial.println("Successfully connected to SD");
    
    char deviceName[8];
    File config = SD.open("/config.txt", FILE_READ, false);
    // Get device's name from SD card or use default. Also get the start
    // time, if present.
    if (!config) {
        strcpy(deviceName, "DIY3-XX");
        Serial.println("Failed to open config file");
    } else {
        config.read((uint8_t*) deviceName, sizeof(deviceName) - 1);
        config.close();
    }
    Serial.printf("Device Name: %s\n", deviceName);
    */
}
int sd_mode = 0;
void loop() {
    if (!digitalRead(GPIO_NUM_27)) {
        sd_mode = !sd_mode;
        Serial.printf("Changing to %d\n", sd_mode);
        digitalWrite(SD_SWITCH_PIN, sd_mode);
        delay(1000);
    }
}


