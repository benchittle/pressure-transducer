/* This script sets the time of the DS3231 from the internet using NTP. It is 
 * intended to be uploaded to DIY3 and DIY4 models.
 *
 * To use it, set the SSID (name) and password of a WiFi network that the ESP32
 * can connect to in the #define statements below. Ensure your components are
 * connected in the same way that they would be for standard DIY3 or DIY4 
 * deployment.
 * 
 * Also specify your timezone offset below in seconds from GMT. For example, 
 * EDT is -14400.
 * 
 * This script was intended to allow hot-plugging the DS3231 module so that
 * one could easily time sync several modules by plugging one in, waiting for 
 * it to sync the time, and then swapping it to a new module while the script 
 * was still running to repeat the process. However, hot-plugging is not stable
 * (at least off of USB power), as it sometimes causes a brown-out.
 */

// DEFINE THESE FOR YOURSELF //
#define WIFI_SSID "my wifi ssid"
#define WIFI_PASSWORD "my wifi password"

#define GMT_OFFSET_SECONDS -14400
//////////////////////////////

// ALTERNATIVELY, PUT THE ABOVE INFORMATION IN A LOCAL-ONLY secrets.h HEADER //
//                FILE TO KEEP IT OUT OF A PUBLIC REPOSITORY                 //
#include "secrets.h"
///////////////////////////////////////////////////////////////////////////////

#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include "ds3231.h"

#define ERROR_LED_PIN GPIO_NUM_2
#define LED_FLASH_DURATION 500
#define RTC_POWER_PIN GPIO_NUM_26
#define ERROR_LED_PIN GPIO_NUM_2

#define NTP_SERVER "pool.ntp.org"


/*
 * Enter an endless loop while flashing the error LED a given number of times
 * each second.
 */
void error(uint8_t flashes) {
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
    Serial.begin(115200);
    delay(3000);

    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);
    
    digitalWrite(RTC_POWER_PIN, HIGH);

    Serial.printf("Connecting to WiFi (SSID=%s)", WIFI_SSID);
    wl_status_t result = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf(" Done\n");

    Serial.printf("\nPreparing to set time via NTP\n");

    struct tm now;
    Serial.println("Configuring...\n");
    // This specifies the time offset info and NTP server we will use when
    // getLocalTime() is called below.
    configTime(GMT_OFFSET_SECONDS, 0, NTP_SERVER);
    
    Serial.printf("Getting time from NTP server (%s)... ", NTP_SERVER);
    // Try to get the time from NTP servers, or time out after 10 seconds.
    if (!getLocalTime(&now, 10000)) {
        Serial.printf("Failed to get local time. Stopping\n");
        error(1);
    }
    Serial.printf("Done\n");

    // Convert time to epoch / unix time.
    time_t const now_epoch = mktime(&now);

    // Convert time to a string for printing.
    char const fmt[] = "%Y-%m-%d %H:%M:%S";
    char now_string[sizeof(fmt) + 2] = "";
    strftime(now_string, sizeof(now_string), fmt, &now);
    Serial.printf("NTP Time: %s\nEpoch: %llu\n", now_string, (long long unsigned) now_epoch);

    Serial.printf("Setting system time... ");
    struct timeval sys_now = {
        .tv_sec = now_epoch,
        .tv_usec = 0,
    };
    // Set the "system" time on the ESP32. This is be needed for tracking time
    // between hotplugs in the loop() below. 
    settimeofday(&sys_now, NULL);
    Serial.printf("Done\n");

    WiFi.disconnect(true);

    Wire.begin();
}

void loop() {
    Serial.printf("\nGetting system time... ");
    struct timeval sys_now;
    gettimeofday(&sys_now, NULL);
    Serial.printf("Done\nEpoch time is %llu\n", (long long unsigned) sys_now.tv_sec);

    struct tm const * const now = localtime(&sys_now.tv_sec);

    // The DS3231 library uses its own time struct "ts" which is slightly 
    // different from tm. We convert the tm above to a ts here.
    struct ts rtc_time = {
        .sec = now->tm_sec,
        .min = now->tm_min,
        .hour = now->tm_hour,
        .mday = now->tm_mday,
        .mon = now->tm_mon + 1, // tm uses 0 based month while ts is 1 based
        .year = now->tm_year + 1900, // tm uses years since 1900 while ts requires the actual year
        .wday = now->tm_wday,
        .yday = now->tm_yday,
    };
    Serial.printf("Connecting to RTC... ");
    delay(1000);
    // Test to see if RTC is connected
    Wire.beginTransmission(DS3231_I2C_ADDR);
    if (Wire.endTransmission(true) != 0) {
       Serial.printf("No RTC detected. Waiting 15 seconds.\n");
       delay(15000);
       return;
    }

    DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    Serial.printf("Done\n");

    Serial.printf("Setting RTC time... ");
    DS3231_set(rtc_time);
    Serial.printf("Done\n");

    Serial.printf("Getting RTC time...");
    rtc_time.year = 0;
    DS3231_get(&rtc_time);
    printf("Done\nRTC Time: %d-%02d-%02d %02d:%02d:%02d\n", rtc_time.year, rtc_time.mon, rtc_time.mday, rtc_time.hour, rtc_time.min, rtc_time.sec);

    Serial.printf("Waiting 15 seconds...\n");
    delay(15000);
}
