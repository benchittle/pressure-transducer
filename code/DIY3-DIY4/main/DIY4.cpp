#include "Arduino.h"
// == Base Function == //
// #include <FS.h>
// #include <SD.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
// #include "driver/sdspi_host.h"

#include <SPI.h>
#include <Wire.h>
#include <esp32/ulp.h>

// ULP macro programming utility functions.
// https://github.com/boarchuz/HULP
#include "hulp.h" 
#include "hulp_i2cbb.h"
// Additional includes for peripheral devices
#include "ds3231.h" // https://github.com/rodan/ds3231
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05
// =================== //

// == WiFi Dashboard == //
// #include <WiFi.h>
// #include <DNSServer.h>

// #include "ArduinoHttpServer.h" // https://github.com/QuickSander/ArduinoHttpServer

// HTML for the dashboard page encoded as a byte array.
#include "index_html.h"
// ==================== //


// Wraps the error function to include line number information when called.
#define ERROR(error_num, attemptLogging) error(error_num, __LINE__, attemptLogging)

// Set to 1 to enable output of diagnostic information to the serial monitor, or
// set to 0 to disable.
#define ECHO_TO_SERIAL 1

// Some functions require gpio_num_t types so we use those pin number 
// definitions here.
#define SD_CS_PIN GPIO_NUM_13 // D7 
#define SD_SWITCH_PIN GPIO_NUM_15 // A4
#define SD_MOSI_PIN GPIO_NUM_23     
#define SD_MISO_PIN GPIO_NUM_19
#define SD_SCK_PIN GPIO_NUM_18
#define RTC_POWER_PIN GPIO_NUM_26 // D3
#define RTC_ALARM_PIN GPIO_NUM_25 //D2
#define ULP_SCL_PIN GPIO_NUM_0 // D5
#define ULP_SDA_PIN GPIO_NUM_14 // D6
#define ERROR_LED_PIN GPIO_NUM_2 // D9
#define BUTTON_PIN GPIO_NUM_27 // D4

// Duration in ms for each flash of the LED when displaying an error / warning.
#define LED_FLASH_DURATION 500

// Maximum number of times the device will attempt to restart on encountering an
// error before entering an endless LED flashing loop.
#define MAX_RESTART_COUNT 3

// Mount point for the SD card
#define SD_MOUNT_POINT "/sd"

// Name to use for naming files if a config file cannot be found on the SD card.
#define DEFAULT_DEVICE_NAME "DIY4-XX"
// Maximum length of the device name in the config file.
#define DEVICE_NAME_SIZE sizeof(DEFAULT_DEVICE_NAME)

// /NAME_YYYYMMDD-hhmm.data is the format.
#define FILE_NAME_FORMAT SD_MOUNT_POINT "/%7s_%04d%02d%02d-%02d%02d.data"
// The format specifiers take up more room than the formatted string will, so we 
// subtract the extra space from the size. Then add room for DIY4-XX part.
#define FILE_NAME_SIZE (sizeof(FILE_NAME_FORMAT) - 11 + 8)

// Path to config file on the SD card
#define CONFIG_FILE SD_MOUNT_POINT "/config.txt"

// A log file will be created on the SD card for capturing diagnostics and 
// errors. If the SD card has a config.txt file with a device name, the log
// will be at /devicename.log. Otherwise, the log file will be based on the
// default device name /DIY3-XX.log.
#define LOG_FILE_NAME_PREFIX SD_MOUNT_POINT "/"
#define LOG_FILE_NAME_SUFFIX ".log"
#define DEFAULT_LOG_FILE_NAME LOG_FILE_NAME_PREFIX DEFAULT_DEVICE_NAME LOG_FILE_NAME_SUFFIX
#define LOG_FILE_NAME_SIZE ((sizeof(DEFAULT_LOG_FILE_NAME) >= sizeof(LOG_FILE_NAME_PREFIX LOG_FILE_NAME_SUFFIX) + DEVICE_NAME_SIZE - 1) ? \
    sizeof(DEFAULT_LOG_FILE_NAME): \
    sizeof(LOG_FILE_NAME_PREFIX LOG_FILE_NAME_SUFFIX) + DEVICE_NAME_SIZE - 1)

// Number of readings (pressure and temperature) to store in a buffer before we 
// dump to the SD card.
#define BUFFER_SIZE 15
// Number of ulp_var_t's needed for one raw pressure and temperature reading.
// (24 bits for raw pressure, 24 bits for raw temperature; could be packed 
// in only 3 spaces with more ULP processing).
#define ULP_SAMPLE_SIZE 4
// Size of the buffer that the ULP will use to store raw data from the MS5803.
#define ULP_BUFFER_SIZE (BUFFER_SIZE * ULP_SAMPLE_SIZE)

// After dumping data to the SD card, we'll wait this long before shutting off 
// power to the SD card.
#define SD_OFF_DELAY_MS 2000

// While the ULP is active, it will repeatedly run at this interval 
// (microseconds).
#define ULP_RUN_PERIOD 1000

#define WIFI_SERVER_PORT 80
#define DNS_SERVER_PORT 53
// Domain at which the WiFi dashboard can be reached after connecting to the 
// ESP's WiFi. 
#define DASHBOARD_DOMAIN "dashboard.lan"
#define HTTP_REQUEST_MAX_BODY_SIZE 512

#define SERVER_MODE_LED_FLASH_PERIOD_MS 50

static const char* TAG = "DIY4";


// A custom struct to store a single data entry.
// NOTE: __attribute__((packed)) tells the compiler not to add additional 
// padding bytes that would align to the nearest 4 bytes. This can cause issues
// in some cases, but the ESP32 doesn't seem to have a problem with it. By doing
// this, we reduce the size of each entry from 12 bytes to 9 bytes.
struct entry_t {
    float pressure;
    int8_t temperature;
} __attribute__((packed));

// Globals for SD card
sdmmc_card_t* card;
sdspi_device_config_t sdspi_device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
char mountPoint[] = SD_MOUNT_POINT;

// Initialize a sensor object for interacting with the MS5803-05
RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

// Store the device's name.
RTC_DATA_ATTR char deviceName[DEVICE_NAME_SIZE] = {0};
// Store the name for the current data file across deep sleep restarts.
RTC_DATA_ATTR char outputFileName[FILE_NAME_SIZE] = {0};
// Store the name for the log file.
RTC_DATA_ATTR char logFileName[LOG_FILE_NAME_SIZE] = DEFAULT_LOG_FILE_NAME;

// Track the number of times a custom error has been encountered causing the
// device to restart.
RTC_NOINIT_ATTR uint8_t restartCount;

// Store the day of the month when the current output file was created. This is
// used to determine when a new day has started (and thus when to start a new 
// file).
RTC_DATA_ATTR uint8_t oldDay = 0;

// Store the epoch time for the first sensor reading in the buffer. This will be
// stored in the output file along with the sensor data. This is updated every 
// time the buffer is dumped.
RTC_DATA_ATTR uint32_t firstSampleTimestamp;

// An array for the ULP to buffer raw sensor data while the main processor is in
// deep sleep. Once full, the data will be processed into meaningful pressure
// and temperature values and written to flash storage.
RTC_DATA_ATTR ulp_var_t ulpBuffer[ULP_BUFFER_SIZE];
// Used to index the ULP's buffer in the ULP program.
RTC_DATA_ATTR ulp_var_t ulpBufOffset;
// Flag used in the ULP program to track whether the MS5803's raw D2 value has
// been read.
RTC_DATA_ATTR ulp_var_t ulpD2Flag;

RTC_DATA_ATTR bool buttonPressed = false;

// The following variables are used by HULP macros to communicate via bitbanged
// I2C.

// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to 
// take a reading and produce a raw value (D1) that we can read with subsequent 
// instructions. (OSR = 4096, see datasheet for details).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convertD1[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x48, 0),
};
// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to
// take a reading and produce a raw value (D2) that we can read with subsequent 
// instructions. (OSR = 4096, see datasheet for details).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convertD2[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x58, 0),
};
// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to
// prepare to send the last raw reading (24 bits long).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_readADC[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x0, 0),
};
// HULP bitbanged I2C instruction. Reads a raw 24 bit value from the MS5803.
RTC_DATA_ATTR ulp_var_t ulp_i2c_read_sensor[HULP_I2C_CMD_BUF_SIZE(3)] = {
    HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 3)
};
// HULP bitbanged I2C instruction. Writes to the DS3231's status register, 
// clearing it. This is done to clear an active alarm. If the state of the 
// status register needs to be maintained, the ULP program must be modified.
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_clearAlarm[] = {
    HULP_I2C_CMD_HDR(DS3231_I2C_ADDR, DS3231_STATUS_ADDR, 1),
    HULP_I2C_CMD_1B(0x0)
};

// WiFiServer wifiServer;
// DNSServer dnsServer;

bool toggleLed = false;


// Error codes for the program. The value associated with each enum is also the
// number of times the error LED will flash if an error is encountered.
enum diy4_error_t {
    ds3231Error = 1,
    sdInitError,
    sdConfigWarning,
    sdFileError,
    ms5803Error,
    resetError,
    exitedSetupError,

    wifiApSetupError,
    dnsSetupError
};


/*
 * Flash the error LED a given number of times to warn the user.
 */
void flash(uint8_t flash_count) {
    for (uint8_t i = 0; i < flash_count; i++) {
        digitalWrite(ERROR_LED_PIN, HIGH);
        delay(LED_FLASH_DURATION);
        digitalWrite(ERROR_LED_PIN, LOW);
        delay(LED_FLASH_DURATION);
    }
}


/*
 * NOTE: Don't call this function directly; use the ERROR macro.
 * Try to restart the device or enter an endless error loop otherwise, flashing
 * the LED in a given sequence to indicate the error. If attemptLogging == true,
 * we attempt to write diagnostic info to a log file on the SD card. If another
 * error is generated during this process, we ignore it and skip logging, 
 * indicating the original error on the LED.
 */
[[noreturn]]
void error(diy4_error_t error_num, size_t line, bool attemptLogging) {
    // Disable ULP wakeups
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    // Stop ULP after it finishes next cycle.
    hulp_ulp_end();
    #if ECHO_TO_SERIAL
        Serial.printf(
            "\nERROR\n"
            "Error Code: %d\n"
            "Line #: %d\n"
            // "SD capacity: %llu B\n"
            // "SD used: %llu B\n"
            "ECHO_TO_SERIAL=%d\n"
            "MAX_RESTART_COUNT=%d\n"
            "BUFFER_SIZE=%d\n"
            "deviceName=%s\n"
            "outputFileName=%s\n"
            "restartCount=%d\n"
            "firstSampleTimestamp=%ld\n",
            //error_num, line, SD.cardSize(), SD.usedBytes(), ECHO_TO_SERIAL, 
            error_num, line, ECHO_TO_SERIAL, 
            MAX_RESTART_COUNT, BUFFER_SIZE, deviceName, outputFileName,
            restartCount, firstSampleTimestamp
        );
        Serial.flush();
    #endif // ECHO_TO_SERIAL

    // Flash the error sequence 3 times on the LED.
    for (uint8_t i = 0; i < 3; ++i) {
        flash(error_num);
        delay(1000);
    }

    // Attempt to log data to SD card (this won't always be possible e.g. if
    // there was an error connecting to the SD card).
    if (attemptLogging) {
        // First we'll try to get the time.
        char timeString[16] = "unknown time";
        // TODO: Fix this approach with better DS3231 lib.
        ts timeNow = {
            .sec = 61
        };
        Wire.begin();
        digitalWrite(RTC_POWER_PIN, HIGH);
        DS3231_get(&timeNow);

        // If we successfully got the time from the DS3231, overwrite timeString
        // with the actual time.
        if (timeNow.sec != 61) {
            snprintf(timeString, sizeof(timeString), "%04d%02d%02d %02d%02d%02d", timeNow.year % 10000u, timeNow.mon % 100u, timeNow.mday % 100u, timeNow.hour % 100u, timeNow.min % 100u, timeNow.sec % 100u);
        }

        // Then we'll try to open and write to a log file on the SD card
        // TODO: Perhaps save to ESP32 flash if we can't save to SD card?
        digitalWrite(SD_SWITCH_PIN, LOW); // Turn on SD power
        // if (SD.begin(SD_CS_PIN)) {
        //     File logFile = SD.open(logFileName, FILE_APPEND, true);
        //     if (logFile) {
        //         #if ECHO_TO_SERIAL
        //             Serial.printf(
        //                 "Logging error to file: %s\n"
        //                 "\tTime: %s\n",
        //                 logFileName, timeString
        //             );
        //             Serial.flush();
        //         #endif // ECHO_TO_SERIAL

        //         logFile.printf(
        //             "\nERROR\n"
        //             "Time: %s\n"
        //             "Error Code: %d\n"
        //             "Line #: %d\n"
        //             "SD capacity: %llu B\n"
        //             "SD used: %llu B\n"
        //             "ECHO_TO_SERIAL=%d\n"
        //             "MAX_RESTART_COUNT=%d\n"
        //             "BUFFER_SIZE=%d\n"
        //             "deviceName=%s\n"
        //             "outputFileName=%s\n"
        //             "restartCount=%d\n"
        //             "firstSampleTimestamp=%ld\n",
        //             timeString, error_num, line, SD.cardSize(), SD.usedBytes(),
        //             ECHO_TO_SERIAL, MAX_RESTART_COUNT, BUFFER_SIZE, deviceName,
        //             outputFileName, restartCount, firstSampleTimestamp
        //         );

        //         if (restartCount < MAX_RESTART_COUNT) {
        //             logFile.println("\nDEVICE WILL RESTART");
        //         } else {
        //             logFile.println("\nDEVICE WILL NOT RESTART");
        //         }
        //         logFile.close();
        //         SD.end();
        //     }
        // }
    }

    // Try to restart the device.
    if (restartCount < MAX_RESTART_COUNT) {
        for (uint8_t i = 0; i < 5; ++i) {
            digitalWrite(ERROR_LED_PIN, HIGH);
            delay(LED_FLASH_DURATION / 4);
            digitalWrite(ERROR_LED_PIN, LOW);
            delay(LED_FLASH_DURATION / 4);
        }
        #if ECHO_TO_SERIAL
            Serial.println("Restarting");
            Serial.flush();
        #endif
        ++restartCount;
        ESP.restart();
    } else {
        #if ECHO_TO_SERIAL
            Serial.println("Max restart count reached. Going into endless deep sleep");
            Serial.flush();
        #endif
        // Otherwise, go into endless deep sleep.
        esp_sleep_enable_ulp_wakeup();
        esp_deep_sleep_start();
    }
}


/* 
 * Define the ULP program, configure the ULP appropriately, and upload the 
 * program to the ULP. 
 */ 
void initUlp()
{
    // Define labels used in the ULP program.
    enum {
        L_LOOP_FOR_D2,

        L_READ,
        L_WRITE,
        L_W_RETURN0,
        L_W_RETURN1,
        L_W_RETURN2,
        L_R_RETURN,
        L_DONE,
    };

    // Notes: 
    // - The following array contains instructions for programming the ULP 
    //   (ultra low power coprocessor) of the ESP32, which runs in parallel to
    //   the main processor and can run while the ESP32 is in deep sleep.
    // - In this case, programming of the ULP is done using the legacy macros.
    // - Bitbanged I2C is achieved using the HULP library. See the examples and
    //   code documentation of that project to understand the "recipe" to
    //   use bitbanged I2C on the ULP.
    // ULP Program Flow:
    //   if DS3231's once-per-second alarm not triggered:
    //     put ULP back to sleep until next cycle
    //   else:
    //     clear alarm on DS3231
    //     read D1 from MS5803 and store in ULP buffer
    //     loop back, read D2 from MS5803 and store in ULP buffer
    //     
    //     if ULP buffer full:
    //       wake up main processor
    //       end ULP program
    //     else:
    //       put ULP back to sleep until next cycle
    //     
    const ulp_insn_t program[] = {
        // Check to see if the RTC alarm was triggered. If so, continue. 
        // Otherwise, halt and put the ULP back to sleep until the next cycle.
        I_GPIO_READ(RTC_ALARM_PIN),
        M_BGE(L_DONE, 1),
        
        // Bitbanged I2C instruction: clear the DS3231's status register to 
        // disable the wakeup alarm.
        I_MOVO(R1, ulp_i2c_write_clearAlarm),
        M_MOVL(R3, L_W_RETURN2),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN2),

        // Clear D2 read flag.
        I_MOVI(R0, 0),
        I_MOVI(R0, 0),
        I_MOVI(R0, 0),
        I_MOVI(R0, 0),
        I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        // I_MOVI(R0, 0),
        
        I_PUT(R0, R0, ulpD2Flag),
        
        // Issue I2C command to MS5803 to start ADC conversion
        I_MOVO(R1, ulp_i2c_write_convertD1),
        M_LABEL(L_LOOP_FOR_D2),     // Loop back here for D2 after D1
        M_MOVL(R3, L_W_RETURN0),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN0),
        
        // Wait 10ms for conversion
        M_DELAY_US_5000_20000(10000),

        // Issue I2C command to prepare to read the ADC value
        I_MOVO(R1, ulp_i2c_write_readADC),
        M_MOVL(R3, L_W_RETURN1),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN1),

        // Read from sensor using I2C.
        I_MOVO(R1, ulp_i2c_read_sensor),
        M_MOVL(R3, L_R_RETURN),
        M_BX(L_READ),
        M_LABEL(L_R_RETURN),

        // Store value in buffer and increase buffer offset.
        I_MOVI(R2, 0),
        I_GET(R3, R2, ulpBufOffset),
        I_GET(R0, R2, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET]),
        I_GET(R1, R2, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET + 1]),
        I_PUTO(R0, R3, 0, ulpBuffer),
        I_PUTO(R1, R3, -1, ulpBuffer), // stores to ulpBuffer[offset + 1]
        I_ADDI(R3, R3, 2),
        I_PUT(R3, R2, ulpBufOffset),

        // Loop back and repeat for D2 if it hasn't been read yet. If D2 was 
        // just read, jump past next block.
        I_GET(R0, R2, ulpD2Flag),
        I_BGE(4, 1),

        // Set D2 flag before branching to get D2. (Since we know R3 > 0, we 
        // don't need to MOVI(R3, 1) first, which saves an instruction).
        I_PUT(R3, R2, ulpD2Flag),
        I_MOVO(R1, ulp_i2c_write_convertD2),
        M_BX(L_LOOP_FOR_D2),

        // Branch here if we've already read D2.

        // Check if the buffer is full and respond accordingly (i.e. wake up
        // processor and end ULP program until it is restarted by the main 
        // processor).
        I_MOVR(R0, R3),
        M_BL(L_DONE, ULP_BUFFER_SIZE),
        I_WAKE(),
        I_END(),

        // Halt ULP program and go back to sleep.
        M_LABEL(L_DONE),
        I_HALT(),

        // Include HULP "subroutines" for bitbanged I2C.
        M_INCLUDE_I2CBB_CMD(L_READ, L_WRITE, ULP_SCL_PIN, ULP_SDA_PIN),
    };
    
    // Configure pins for use by the ULP.
    ESP_ERROR_CHECK(hulp_configure_pin(ULP_SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(ULP_SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));   
    ESP_ERROR_CHECK(hulp_configure_pin(RTC_ALARM_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));
    //ESP_ERROR_CHECK(hulp_configure_pin(SD_SWITCH_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1));

    hulp_peripherals_on();

    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Load the program and start the ULP.
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), ULP_RUN_PERIOD, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void IRAM_ATTR buttonInterrupt() {
    buttonPressed = true;
}

void IRAM_ATTR serverModeLedToggleInterrupt() {
    toggleLed = true;
}

/* 
 * Deinitialize peripherals and put the device into deep sleep. The LED will 
 * remain on once it is safe to disconnect power.
 * Note: This is not a general purpose shutdown function, and it does not shut
 * down peripherals that are already expected to be disabled / not in use (e.g. 
 * the SD card) in the intended use case. It will:
 * - stop the ULP from running again
 * - reconfigure the RTC to stop generating alarms and disable its pin power
 * - put the ESP32 to deepsleep
 */ 
[[noreturn]]
void shutdown() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    #if ECHO_TO_SERIAL
        Serial.println("Shutting down...\n\tStopping ULP");
        Serial.flush();
    #endif
    // Stop ULP after it finishes next cycle.
    hulp_ulp_end();

    #if ECHO_TO_SERIAL
        Serial.println("\tDeinitializing RTC");
        Serial.flush();
    #endif
    // Enable power to RTC
    digitalWrite(RTC_POWER_PIN, HIGH);
    // Wait to make sure RTC is on and ULP is done using I2C.
    delay(1000); 

    // Disable the RTC
    Wire.begin();
    // Set default values in control register and disable 32khz output. This
    // will also stop the alarm from pulsing when battery powered. 
    DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    // Clear any previous alarms.
    DS3231_clear_a1f();
    DS3231_clear_a2f();
    Wire.end();
    digitalWrite(RTC_POWER_PIN, LOW);

    // Turn on the LED to signal power can be removed. This function is used to
    // keep the LED on during deep sleep.
    hulp_configure_pin(ERROR_LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, HIGH);

    #if ECHO_TO_SERIAL
        Serial.println("Done\nPower can be turned off safely");
        Serial.flush();
    #endif

    // Keep LED on during sleep.
    hulp_peripherals_on();
    // Sleep forever (the ULP is disabled so it won't wake us up).
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}

bool sdInit() {
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize((spi_host_device_t) host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = (spi_host_device_t) host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mountPoint, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return false;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    // sdmmc_card_print_info(stdout, card);
    return true;
}

void sdDeinit() {
    esp_vfs_fat_sdcard_unmount(mountPoint, card);
    spi_bus_free(sdspi_device_config.host_id);
}

void setup() {
    esp_err_t ret;
     bool buttonPressedAtStartup = false;
    #if ECHO_TO_SERIAL
        Serial.begin(115200);
    #else
        // ADC, WiFi, BlueTooth are disabled by default.        
        // Default CPU frequency is 240MHz, but we don't need high speed. Note
        // that the serial monitor baud rate changes with lower frequencies, so
        // it is not changed when debugging.
        setCpuFrequencyMhz(10);     
    #endif

    // pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power
    pinMode(SD_SWITCH_PIN, OUTPUT);
    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(RTC_ALARM_PIN, INPUT_PULLUP);
    pinMode(ERROR_LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // After setup, the button can be used to safely "shut down" the ESP32 and 
    // disable active peripherals.
    attachInterrupt(BUTTON_PIN, buttonInterrupt, FALLING);
    // Using ext1 instead of ext0 because of a bug that prevents ext0 and ULP
    // interrupts from being used together.
    esp_sleep_enable_ext1_wakeup(((uint64_t) 0b1) << BUTTON_PIN, ESP_EXT1_WAKEUP_ALL_LOW);

    // Jump to the appropriate code depending on whether the ESP32 was just 
    // powered or just woke up from deep sleep. 
    switch (esp_reset_reason()) {   
        case ESP_RST_POWERON:
            restartCount = 0;
            if (digitalRead(BUTTON_PIN) == LOW) {
                buttonPressedAtStartup = true;
                #if ECHO_TO_SERIAL
                    ESP_LOGI(TAG, "Button press detected at startup. Device will enter server mode after initializing");
                    // Serial.flush();
                #endif
            }
            [[fallthrough]];
        case ESP_RST_SW: {
            ulpBufOffset.val = 0;
            ulpD2Flag.val = 0;

            #if ECHO_TO_SERIAL
                // delay(1000); // Give time for Serial Monitor to start listening
                ESP_LOGI(TAG, "STARTING SETUP\nCode uploaded on " __DATE__ " @ " __TIME__ "\nInitializing RTC... ");
                // Serial.flush();
            #endif

            Wire.begin();
            // Initialize the connection with the RTC:
            // Power to the RTC is provided by a GPIO pin. Providing main power
            // to the DS3231 causes it to drain more current, but will preserve
            // the backup battery which is used most of the time. We provide 
            // main power here in case the device has not yet been initialized.
            digitalWrite(RTC_POWER_PIN, HIGH);

            // According to the datasheet, the oscillator takes <1 sec to begin (on 
            // first time startup), so we wait 1 second in case this is the first time
            // startup. 
            delay(1000); 

            // Set default values in control register and disable 32khz output.
            // TODO: Need a way to make sure this succeeds.
            DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
            // Clear any previous alarms.
            DS3231_clear_a1f();
            DS3231_clear_a2f();

            // Set the current day value.
            struct ts timeNow;
            DS3231_get(&timeNow);
            oldDay = timeNow.mday;

            #if ECHO_TO_SERIAL
                ESP_LOGI(TAG, "Done\n\tRTC Time: %d-%02d-%02d %02d:%02d:%02d\n", timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                // Serial.flush();
            #endif

            // TODO: Check Status register as well, as it indicates if time was 
            // set since powered on (See #1)
            if (timeNow.year < 2023) {
                #if ECHO_TO_SERIAL
                    ESP_LOGW(TAG, "WARNING: Time is out of date. Setting year to 2000 for UNIX time compatibility...\n");
                    // Serial.flush();
                #endif
                timeNow.year = 2000;
                DS3231_set(timeNow);
                #if ECHO_TO_SERIAL
                    ESP_LOGI(TAG, "Done\n\tNew RTC Time: %d-%02d-%02d %02d:%02d:%02d\n", timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                    // Serial.flush();
                #endif
            }

            // Only start the alarm if we aren't going into server mode.
            if (!buttonPressedAtStartup) {
                #if ECHO_TO_SERIAL
                    ESP_LOGI(TAG, "Starting RTC once-per-second alarm... ");
                    // Serial.flush();
                #endif

                // Start the once-per-second alarm on the DS3231:
                // These flags set the alarm to be in once-per-second mode.
                const uint8_t flags[] = {1,1,1,1,0};
                // Set the alarm. The time value doesn't matter in once-per-second
                // mode.
                DS3231_set_a1(1, 1, 1, 1, flags);
                // Enable the alarm. It will now bring the RTC_ALARM_PIN GPIO low every
                // second.
                DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);

                #if ECHO_TO_SERIAL
                    ESP_LOGI(TAG, "Done\n");
                #endif
            }


            #if ECHO_TO_SERIAL
                ESP_LOGI(TAG, "Initializing SD card... ");
                // Serial.flush();
            #endif

            sdInit();

            // Card has been initialized, print its properties
            sdmmc_card_print_info(stdout, card);

            // Initialize the connection with the SD card.
            // digitalWrite(SD_SWITCH_PIN, LOW);
            // if (!SD.begin(SD_CS_PIN)) {
            //     ERROR(sdInitError, false);
            // }

            #if ECHO_TO_SERIAL
                ESP_LOGI(TAG, "Done\nLooking for config file at " CONFIG_FILE "... ");
                // Serial.flush();
            #endif


            // Get device's name from SD card or use default.
            FILE* config = fopen(CONFIG_FILE, "r");
            if (config == NULL) {
                strcpy(deviceName, DEFAULT_DEVICE_NAME);
                strcpy(logFileName, DEFAULT_LOG_FILE_NAME);
                #if ECHO_TO_SERIAL
                    ESP_LOGI(TAG, "Failed\nWARNING: Unable to find config file. Using default device name: %s\n", deviceName);
                    // Serial.flush();
                #endif
                flash(sdConfigWarning);
            } else {
                // CHECK RETURN VALUE
                fgets(deviceName, DEVICE_NAME_SIZE, config);
                // config.read((uint8_t*) deviceName, DEVICE_NAME_SIZE - 1);
                // config.close();
                fclose(config);
                config = NULL;

                snprintf(logFileName, LOG_FILE_NAME_SIZE, LOG_FILE_NAME_PREFIX "%s" LOG_FILE_NAME_SUFFIX, deviceName);

                #if ECHO_TO_SERIAL
                    Serial.printf("Done\n\tDevice Name: %s\tLog File Name: %s\n", deviceName, logFileName);
                    Serial.flush();
                #endif
            }

            #if ECHO_TO_SERIAL
                Serial.print("Creating first output file... ");
                Serial.flush();
            #endif
            
            // Generate the first file.
            snprintf(outputFileName, FILE_NAME_SIZE, FILE_NAME_FORMAT, deviceName, timeNow.year % 10000, timeNow.mon % 100, timeNow.mday % 100, timeNow.hour % 100, timeNow.min % 100);
            
            ESP_LOGI(TAG, "Attempting to open file: %s", outputFileName);
            FILE* f = fopen(outputFileName, "w");
            if (f == NULL) {
                ERROR(sdFileError, true);
            }
            fclose(f);
            f = NULL;

            #if ECHO_TO_SERIAL
                Serial.printf("Done\n\tFile is %s\nInitializing MS5803 pressure sensor... ", outputFileName);
                Serial.flush();
            #endif


            // Initialize the connection with the MS5803-05 pressure sensor.
            if (!sensor.initializeMS_5803(false)) {
                ERROR(ms5803Error, true);
            }

            #if ECHO_TO_SERIAL
                Serial.print("Done\nTaking a sample sensor reading... ");
                Serial.flush();
            #endif

            sensor.readSensor();

            #if ECHO_TO_SERIAL
                Serial.printf("Done\n\tPressure: %f mbar\tTemperature: %f deg C\n", sensor.pressure(), sensor.temperature());
                Serial.flush();
            #endif


            // Write an entry to the log file with diagnostic and setup info.
            #if ECHO_TO_SERIAL
                Serial.print("Writing to log file... ");
                Serial.flush();
            #endif

            FILE* logFile = fopen(logFileName, "a");
            if (!logFile) {
                ERROR(sdFileError, false);
            }
            fprintf(logFile,
                "\nSETUP COMPLETE\n"
                "RTC Time: %d-%02d-%02d %02d:%02d:%02d\n"
                "Sample Pressure: %f mbar\n"
                "Sample Temp: %f deg C\n"
                "Code uploaded on: " __DATE__ " @ " __TIME__ "\n"
                // "SD capacity: %llu B\n"
                // "SD used: %llu B\n"
                "ECHO_TO_SERIAL=%d\n"
                "MAX_RESTART_COUNT=%d\n"
                "BUFFER_SIZE=%d\n"
                "CONFIG_FILE=" CONFIG_FILE "\n"
                "logFileName=%s\n"
                "deviceName=%s\n"
                "outputFileName=%s\n"
                "restartCount=%d\n"
                "firstSampleTimestamp=%ld\n",
                timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, 
                timeNow.min, timeNow.sec, sensor.pressure(), 
                // sensor.temperature(), SD.cardSize(), SD.usedBytes(),
                sensor.temperature(),
                ECHO_TO_SERIAL, MAX_RESTART_COUNT, BUFFER_SIZE, logFileName,
                deviceName, outputFileName, restartCount, firstSampleTimestamp
            );

            fclose(logFile);
            logFile = NULL;

            if (!buttonPressedAtStartup) {
                sdDeinit();
            }

            #if ECHO_TO_SERIAL
                Serial.print("Done\nFlashing LED... ");
                Serial.flush();
            #endif


            // Flash LED's to signal successful startup.
            digitalWrite(ERROR_LED_PIN, HIGH);
            delay(3000);
            digitalWrite(ERROR_LED_PIN, LOW);

            if (buttonPressedAtStartup) {
                #if ECHO_TO_SERIAL
                    Serial.println("Done\nEntering server mode");
                    Serial.flush();
                #endif
                // runServer();          
            }

            #if ECHO_TO_SERIAL
                Serial.print("Done\nDisabling SD card power and getting updated time... ");
                Serial.flush();
            #endif

            digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power

            DS3231_get(&timeNow);
            // Save a timestamp for the first sample. We'll add 1 second
            // (alarmStatus) to the time since the first sample will actually be
            // taken during the next second.
            firstSampleTimestamp = timeNow.unixtime + 1;

            #if ECHO_TO_SERIAL
                Serial.printf(
                    "Done\n"
                    "\tTime is %d-%02d-%02d %02d:%02d:%02d\n"
                    "Waiting for next second... ", 
                    timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec
                );
                Serial.flush();
            #endif

            // Wait the rest of this second to start.
            while (timeNow.unixtime < firstSampleTimestamp) {
                DS3231_get(&timeNow);
            }

            #if ECHO_TO_SERIAL
                Serial.print("Done\nDisabling DS3231 pin power and starting ULP... ");
                Serial.flush();
            #endif

            // Disable power to the DS3231's VCC.
            digitalWrite(RTC_POWER_PIN, LOW);
           
            restartCount = 0;

            // Start the ULP program.
            initUlp();

            #if ECHO_TO_SERIAL
                Serial.println("Done\nSETUP COMPLETE\nGoing to sleep...");
                Serial.flush();
            #endif

            // Allow the ULP to trigger the ESP32 to wake up.
            ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
            // Enter deep sleep.
            esp_deep_sleep_start();
        }

        // TODO: Check if DS3231 backup battery failed and switch to main power
        // TODO: Cleanup Serial logging / debug messages
        case ESP_RST_DEEPSLEEP: {

            #if ECHO_TO_SERIAL    
                Serial.println("Awake!");
                Serial.flush();
            #endif

            // If the shutdown button was pushed while the device was in deep 
            // sleep, stop sampling and shut down.
            switch(esp_sleep_get_wakeup_cause()) {
                case ESP_SLEEP_WAKEUP_EXT1: shutdown();
                default: break;
            }

            // Reinitialize connection with DS3231 before reactivating the ULP
            // to avoid interfering I2C commands.
            Wire.begin();
            // Enable power to the RTC. This is done to avoid draining the coin
            // cell during I2C, but the power savings may be negligible.
            // TODO: Test battery life operating the RTC using ONLY coin cell.
            digitalWrite(RTC_POWER_PIN, HIGH);
            // Get the time from the RTC.
            struct ts timeNow;
            DS3231_get(&timeNow);

            // Record the status of the RTC's alarm for later.
            uint8_t alarmStatus = digitalRead(RTC_ALARM_PIN);

            // Disable power to the RTC.
            Wire.end();
            digitalWrite(RTC_POWER_PIN, LOW);


            // Make a copy of the raw data in the ULP's buffer.
            ulp_var_t raw[ULP_BUFFER_SIZE];
            memcpy(raw, ulpBuffer, ULP_BUFFER_SIZE * sizeof(ulpBuffer[0]));
            // Restart the ULP as soon as we've made a copy of the data, since 
            // it can run in parallel with the main program as we process the
            // raw data.
            ulpBufOffset.val = 0;
            initUlp();


            // Buffer to store the processed data that will be written to flash.
            entry_t writeBuffer[BUFFER_SIZE];

            // Process all the raw data using the conversion sequence specified
            // in the MS5803 datasheet. 
            for (uint16_t i = 0, j = 0; i < BUFFER_SIZE; i++, j += ULP_SAMPLE_SIZE) {
                // Read the next D1 value from the raw data.
                uint32_t varD1 = (raw[j].val << 8) | (raw[j + 1].val >> 8);
                // Read the next D2 value from the raw data.
                uint32_t varD2 = (raw[j + 2].val << 8) | (raw[j + 3].val >> 8);

                // Convert raw D1 and D2 to pressure and temperature.
                sensor.convertRaw(varD1, varD2);
                // Write the processed data to the buffer.
                writeBuffer[i] = {
                    .pressure = sensor.pressure(),
                    .temperature = (int8_t) sensor.temperature()
                };
            }


            // Save the buffer to flash:
            #if ECHO_TO_SERIAL
                Serial.printf("RTC Time Now: %d-%02d-%02d %02d:%02d:%02d\n", timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                Serial.printf("Dumping to card...\nFirst Timestamp=%ld\nP\tT\n", firstSampleTimestamp);
                for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
                    printf("%.2f \t%u\n", writeBuffer[i].pressure, writeBuffer[i].temperature);
                }
                Serial.flush();
            #endif

            // Reinitialize connection with SD card.
            #if ECHO_TO_SERIAL
                Serial.println("Turning on SD power");
            #endif
            digitalWrite(SD_SWITCH_PIN, LOW); // Turn on SD card power
            delay(100); // Some sensors were erroring here, so maybe delay is needed?
            if (!sdInit()) {
                ERROR(sdInitError, false);
            }

            #if ECHO_TO_SERIAL
                Serial.printf("Card Size: %lld\n", 123456789ll); //SD.cardSize());
                Serial.flush();
            #endif

            // Open a file for logging the data. If it's the first dump of
            // the day, start a new file.
            // TODO: Write buffered data before starting new day
            FILE* f = fopen(outputFileName, "a");
            if (f == NULL) {
                ERROR(sdFileError, true);
            }
            // Write timestamp of first sample.
            size_t written = fwrite(&firstSampleTimestamp, sizeof(firstSampleTimestamp), 1, f);
            // Write the data buffer as a sequence of bytes (it's vital that
            // the entry_t struct is packed, otherwise there will be garbage
            // bytes in between each entry that will waste space). In order
            // to use this data later, we'll have to unpack it using a 
            // postprocessing script.
            written += fwrite(writeBuffer, sizeof(writeBuffer[0]), BUFFER_SIZE, f);
            fclose(f);
            
            #if ECHO_TO_SERIAL
                Serial.printf("Wrote %u bytes to file\n", written);
                Serial.flush();
            #endif

            // Save a timestamp for the first sample. We'll add 1 second
            // (alarmStatus) to the time if the alarm hadn't triggered yet when
            // the timestamp was taken, since the first sample will actually be
            // taken during the next second.
            firstSampleTimestamp = timeNow.unixtime + alarmStatus;

            // If a new day has started, start a new data file for the next 
            // cycle.
            if (oldDay != timeNow.mday) {
                // Generate a new file name with the current date and time.
                snprintf(outputFileName, FILE_NAME_SIZE, FILE_NAME_FORMAT, deviceName, timeNow.year % 10000, timeNow.mon % 100, timeNow.mday % 100, timeNow.hour % 100, timeNow.min % 100);
                // Start a new file.
                f = fopen(outputFileName, "w");
                if (f == NULL) {
                    ERROR(sdFileError, true);
                }
                
                fclose(f);
                oldDay = timeNow.mday;
            } 

            sdDeinit();
            #if ECHO_TO_SERIAL
                Serial.printf("Turning off SD power in %d ms... ", SD_OFF_DELAY_MS);
            #endif
            delay(SD_OFF_DELAY_MS);
            digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power

            #if ECHO_TO_SERIAL
                Serial.println("Done\nGoing to sleep...");
                Serial.flush();
            #endif

            if (buttonPressed) {
                shutdown();
            }

            // Allow the ULP to trigger the ESP32 to wake up.
            ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

            // Enter deep sleep.
            esp_deep_sleep_start();
        }

        default: 
            #if ECHO_TO_SERIAL
                Serial.printf("Wakeup reason: %d\n", esp_reset_reason());
                Serial.flush();
            #endif
            ERROR(resetError, true);
    }
}

void loop() {
    ERROR(exitedSetupError, true);
}

// void runServer() {
//     // Reset CPU frequency to default so WiFi stuff functions properly.
//     setCpuFrequencyMhz(240);

//     #if ECHO_TO_SERIAL
//         Serial.print("Setting up WiFi access point... ");
//         Serial.flush();
//     #endif
//     if (WiFi.softAP(deviceName)) {
//         #if ECHO_TO_SERIAL
//             Serial.print("Done\n");
//             Serial.flush();
//         #endif
//     } else {
//         #if ECHO_TO_SERIAL
//             Serial.print("Failed\n");
//             Serial.flush();
//         #endif
//         ERROR(wifiApSetupError, 1);
//     }

//     #if ECHO_TO_SERIAL
//         Serial.print("Setting up WiFi server... ");
//         Serial.flush();
//     #endif
//     wifiServer = WiFiServer(WIFI_SERVER_PORT);
//     wifiServer.begin();
//     #if ECHO_TO_SERIAL
//         Serial.print("Done\nSetting up DNS server... ");
//         Serial.flush();
//     #endif
//     if (dnsServer.start(DNS_SERVER_PORT, DASHBOARD_DOMAIN, WiFi.softAPIP())) {
//         #if ECHO_TO_SERIAL
//             Serial.print("Done\nBeginning server loop\n");
//             Serial.flush();
//         #endif
//     } else {
//         #if ECHO_TO_SERIAL
//             Serial.print("Failed\n");
//             Serial.flush();
//         #endif
//         ERROR(dnsSetupError, 1);
//     }

//     hw_timer_t* timer0 = timerBegin(1000000);
//     timerAttachInterrupt(timer0, serverModeLedToggleInterrupt);
//     timerAlarm(timer0, SERVER_MODE_LED_FLASH_PERIOD_MS * 1000, true, 0);

//     buttonPressed = false;
//     while(1) {
//         serverLoop();
//         if (toggleLed) {
//             if (digitalRead(ERROR_LED_PIN) == LOW) {
//                 digitalWrite(ERROR_LED_PIN, HIGH);
//             } else {
//                 digitalWrite(ERROR_LED_PIN, LOW);
//             }
//             toggleLed = false;
//         }
//         if (buttonPressed) {
//             #if ECHO_TO_SERIAL
//                 Serial.print("Stopping DNS and WiFi server... ");
//                 Serial.flush();
//             #endif
//             dnsServer.stop();
//             wifiServer.stop();
//             #if ECHO_TO_SERIAL
//                 Serial.flush();
//             #endif
//             WiFi.mode(WIFI_OFF);
//             #if ECHO_TO_SERIAL
//                 Serial.println("Done\nStopping LED timer and shutting down");
//                 Serial.flush();
//             #endif
//             timerEnd(timer0);
//             shutdown();
//         }
//     }
// }

// void serverLoop() {
//     dnsServer.processNextRequest();

//     WiFiClient client = wifiServer.accept(); 
//     if (client) {
//         #if ECHO_TO_SERIAL
//             Serial.print("Client connected\n");
//             Serial.flush();
//         #endif
//         ArduinoHttpServer::StreamHttpRequest<HTTP_REQUEST_MAX_BODY_SIZE> httpRequest(client);
//         if (httpRequest.readRequest()) {
//             const String& resource = httpRequest.getResource().toString();
//             #if ECHO_TO_SERIAL
//                 Serial.print("\tReceived HTTP request\n");
//                 char* method;
//                 switch(httpRequest.getMethod()) {
//                     case ArduinoHttpServer::Method::Get: method = "GET"; break;
//                     case ArduinoHttpServer::Method::Put: method = "PUT"; break;
//                     case ArduinoHttpServer::Method::Post: method = "POST"; break;
//                     case ArduinoHttpServer::Method::Head: method = "HEAD"; break;
//                     case ArduinoHttpServer::Method::Delete: method = "DELETE"; break;
//                     default: method = "INVALID";
//                 }
//                 Serial.printf(
//                     "\t\tMethod: %s\n"
//                     "\t\tResource: %s\n"
//                     "\t\tBody: %s\n", 
//                     method, resource.c_str(), httpRequest.getBody()
//                 );
//                 Serial.flush();
//             #endif
//             switch (httpRequest.getMethod()) {
//                 case ArduinoHttpServer::Method::Get:
//                 {
//                     if (resource == "/") {
//                         ArduinoHttpServer::StreamHttpReply(client, "text/html").send(index_html);
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 200\n");
//                             Serial.flush();
//                         #endif
//                     }
//                     else if (resource == "/api/clock") {
//                         struct ts timeNow;
//                         DS3231_get(&timeNow);

//                         char json[64];
//                         snprintf(json, sizeof(json), "{\"clock_time\": %d}\n", timeNow.unixtime);

//                         ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 200\n");
//                             Serial.flush();
//                         #endif
//                     } else if (resource == "/api/sensor") {
//                         sensor.readSensor();

//                         char json[64];
//                         snprintf(
//                             json, 
//                             sizeof(json), 
//                             "{\"sensor_pressure\": %.1f, \"sensor_temperature\": %.1f}",
//                             sensor.pressure(), sensor.temperature()
//                         );

//                         ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 200\n");
//                             Serial.flush();
//                         #endif
//                     } else if (resource == "/api/all") {
//                         struct ts timeNow;
//                         DS3231_get(&timeNow);
//                         sensor.readSensor();

//                         char json[512];
//                         snprintf(
//                             json, 
//                             sizeof(json), 
//                             "{" 
//                                 "\"device_name\": \"%s\","
//                                 "\"storage_capacity\": %llu,"
//                                 "\"storage_used\": %llu,"
//                                 "\"clock_time\": %ld,"
//                                 "\"sensor_pressure\": %.1f," 
//                                 "\"sensor_temperature\": %.1f"
//                             "}",
//                             deviceName,
//                             SD.cardSize(), 
//                             SD.usedBytes(), 
//                             timeNow.unixtime,
//                             sensor.pressure(), 
//                             sensor.temperature()
//                         );

//                         ArduinoHttpServer::StreamHttpReply(client, "application/json").send(json);
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 200\n");
//                             Serial.flush();
//                         #endif
//                     } else {
//                         ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "404").send("");
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 404\n");
//                             Serial.flush();
//                         #endif
//                     }
//                     break;
//                 }
//                 case ArduinoHttpServer::Method::Post: {
//                     if (resource == "/api/clock") {
//                         const char *const body = httpRequest.getBody();
//                         uintmax_t epoch_time = strtoumax(body, NULL, 10);
//                         if (epoch_time == 0) {
//                             #if ECHO_TO_SERIAL
//                                 Serial.println("\tError: Invalid time string in http body");
//                                 Serial.flush();
//                             #endif
//                             ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "422").send("");
//                             #if ECHO_TO_SERIAL
//                                 Serial.print("\tSent reply with code 422\n");
//                                 Serial.flush();
//                             #endif
//                             break;
//                         } else if (epoch_time > std::numeric_limits<time_t>::max()) {
//                             #if ECHO_TO_SERIAL
//                                 Serial.println("\tError: Time given is too large to set onboard clock");
//                                 Serial.flush();
//                             #endif
//                             ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "422").send("");
//                             #if ECHO_TO_SERIAL
//                                 Serial.print("\tSent reply with code 422\n");
//                                 Serial.flush();
//                             #endif
//                             break;
//                         }
//                         time_t converted_epoch_time = static_cast<time_t>(epoch_time);
//                         struct tm *time = localtime(&converted_epoch_time);

//                         struct ts rtc_time = {
//                             .sec = time->tm_sec,
//                             .min = time->tm_min,
//                             .hour = time->tm_hour,
//                             .mday = time->tm_mday,
//                             .mon = time->tm_mon + 1,      // tm uses 0 based month while ts is 1 based
//                             .year = time->tm_year + 1900, // tm uses years since 1900 while ts requires the actual year
//                             .wday = time->tm_wday,
//                             .yday = time->tm_yday,
//                         };
//                         DS3231_set(rtc_time);
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tClock updated\n");
//                             Serial.flush();
//                         #endif

//                         ArduinoHttpServer::StreamHttpReply(client, "text/plain").send("");
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 200\n");
//                             Serial.flush();
//                         #endif
//                     } else {
//                         ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "404").send("");
//                         #if ECHO_TO_SERIAL
//                             Serial.print("\tSent reply with code 404\n");
//                             Serial.flush();
//                         #endif
//                     }
//                     break;
//                 }
//                 default:
//                     ArduinoHttpServer::StreamHttpErrorReply(client, "text/plain", "501").send("");
//                     #if ECHO_TO_SERIAL
//                         Serial.print("\tSent reply with code 501\n");
//                         Serial.flush();
//                     #endif
//             }
//         } else {
//             #if ECHO_TO_SERIAL
//                 Serial.print("\tFailed to read incoming request\n");
//                 Serial.flush();
//             #endif
//         }
//         client.stop();
//         #if ECHO_TO_SERIAL
//             Serial.print("Client disconnected\n");
//             Serial.flush();
//         #endif
//     }
// }