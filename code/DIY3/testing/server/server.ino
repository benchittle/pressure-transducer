#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <WiFi.h>

// #define ARDUINO_HTTP_SERVER_DEBUG
#include "ArduinoHttpServer.h" // https://github.com/QuickSander/ArduinoHttpServer
#include "ds3231.h"            // https://github.com/rodan/ds3231
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05

#include "index_html.h"

#define SD_CS_PIN GPIO_NUM_13 // TODO: Change this pin: FireBeetle uses it for LED
#define RTC_POWER_PIN GPIO_NUM_26 // D3
#define RTC_ALARM_PIN GPIO_NUM_25 // D2

#define CONFIG_FILE "/config.txt"
#define DEFAULT_DEVICE_NAME "DIY3-XX"
#define DEVICE_NAME_SIZE 7

char deviceName[DEVICE_NAME_SIZE + 1] = {0};

// Initialize a sensor object for interacting with the MS5803-05
RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

// Replace with your network credentials
const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

void setup() {
    Serial.begin(115200);

    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(RTC_ALARM_PIN, INPUT_PULLUP);

    // RTC setup
    Wire.begin();
    digitalWrite(RTC_POWER_PIN, HIGH);
    delay(1000);
    DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);

    // Initialize the connection with the SD card.
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD ERROR");
    }
    // Get device's name from SD card or use default.
    File config = SD.open(CONFIG_FILE, FILE_READ, false);
    if (!config) {
        strcpy(deviceName, DEFAULT_DEVICE_NAME);
        Serial.printf("WARNING: Unable to find config file. Using default device name: %s\n", deviceName);
    } else {
        config.read((uint8_t*) deviceName, DEVICE_NAME_SIZE);
        config.close();

        Serial.printf("Done\n\tDevice Name: %s\n", deviceName);
    }

    // Initialize the connection with the MS5803-05 pressure sensor.
    if (!sensor.initializeMS_5803(false)) {
        Serial.println("Sensor error");
    }

    // WiFi setup
    Serial.print("Setting AP (Access Point)...");
    WiFi.softAP(ssid, password); // CHECK SUCCESS

    server.begin();
}

void loop() {
    WiFiClient client = server.available(); 

    if (client) {
        ArduinoHttpServer::StreamHttpRequest<512> httpRequest(client);
        if (httpRequest.readRequest()) {
            const String &resource = httpRequest.getResource().toString();
            switch (httpRequest.getMethod()) {
            case ArduinoHttpServer::Method::Get:
            {
                if (resource == "/") {
                    ArduinoHttpServer::StreamHttpReply(client, "text/html").send(reinterpret_cast<char *>(index_html));
                }
                else if (resource == "/api/clock") {
                    Serial.println("Getting time");

                    struct ts timeNow;
                    DS3231_get(&timeNow);
                    Serial.printf("Clock time: %ld\n", timeNow.unixtime);

                    char json[64];
                    snprintf(json, sizeof(json), "{\"clock_time\": %d}\n", timeNow.unixtime);

                    ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
                } else if (resource == "/api/sensor") {
                    Serial.println("Getting sensor reading");
                    sensor.readSensor();

                    char json[64];
                    snprintf(
                        json, 
                        sizeof(json), 
                        "{\"sensor_pressure\": %.2f, \"sensor_temperature\": %.2f}",
                        sensor.pressure(), sensor.temperature()
                    );

                    ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
                } else if (resource == "/api/all") {
                    Serial.println("Getting all data");

                    struct ts timeNow;
                    DS3231_get(&timeNow);
                    sensor.readSensor();

                    char json[512];
                    snprintf(
                        json, 
                        sizeof(json), 
                        "{" 
                            "\"device_name\": \"%s\","
                            "\"storage_capacity\": %llu,"
                            "\"storage_used\": %llu,"
                            "\"clock_time\": %ld,"
                            "\"sensor_pressure\": %.2f," 
                            "\"sensor_temperature\": %.2f"
                        "}",
                        deviceName,
                        SD.cardSize(), 
                        SD.usedBytes(), 
                        timeNow.unixtime,
                        sensor.pressure(), 
                        sensor.temperature()
                    );

                    ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
                }
                break;
            }
            case ArduinoHttpServer::Method::Post: {
                if (resource == "/api/clock") {
                    Serial.println("Setting clock time");

                    const char *const body = httpRequest.getBody();
                    uintmax_t epoch_time = strtoumax(body, NULL, 10);
                    if (epoch_time == 0) {
                        Serial.println("Error: Invalid time string in http body");
                        ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "422").send("");
                    } else if (epoch_time > std::numeric_limits<time_t>::max()) {
                        Serial.println("Error: Time given is too large to set onboard clock");
                        ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "422").send("");
                    }
                    time_t converted_epoch_time = static_cast<time_t>(epoch_time);
                    struct tm *time = localtime(&converted_epoch_time);

                    struct ts rtc_time = {
                        .sec = time->tm_sec,
                        .min = time->tm_min,
                        .hour = time->tm_hour,
                        .mday = time->tm_mday,
                        .mon = time->tm_mon + 1,      // tm uses 0 based month while ts is 1 based
                        .year = time->tm_year + 1900, // tm uses years since 1900 while ts requires the actual year
                        .wday = time->tm_wday,
                        .yday = time->tm_yday,
                    };
                    DS3231_set(rtc_time);
                    Serial.println("Done setting time");

                    ArduinoHttpServer::StreamHttpReply(client, "text/plain").send("");
                }
                break;
            }
            default:
                // unsupported method
                Serial.println("Error: Unsupported HTTP method");
            }
        }
        client.stop();
    }
}