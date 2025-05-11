// ESP IDF libraries
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_fat.h"
#include "esp_wake_stub.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "soc/rtc.h"
#include "ulp.h"

// Arduino libraries
#include "SPI.h"
#include "Wire.h"

// JSON parsing support
#include "cJSON.h"

// ULP macro programming utility functions.
// https://github.com/boarchuz/HULP
#include "hulp.h" 
#include "hulp_i2cbb.h"
// Additional includes for peripheral devices
#include "ds3231.h" // https://github.com/rodan/ds3231
#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05
// =================== //

// HTML for the dashboard page encoded as a byte array.
#include "index_html.h"
// ==================== //

// Wraps the error function to include line number information when called.
#define TRANSDUCER_ERROR(error_num, attempt_logging) transducer_error(error_num, __LINE__, attempt_logging)

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
#define SD_MOUNT_POINT_LEN (sizeof(SD_MOUNT_POINT) - 1)

// Name to use for naming files if a config file cannot be found on the SD card.
#define DEFAULT_DEVICE_NAME "DIY-XX"
// Maximum length of the device name in the config file (including null 
// terminator).
#define DEVICE_NAME_SIZE 16

#define FILE_NAME_PREFIX_FORMAT SD_MOUNT_POINT "/%s"
#define FILE_NAME_PREFIX_MAX_LEN (SD_MOUNT_POINT_LEN + 1 + DEVICE_NAME_SIZE)
#define FILE_NAME_SUFFIX_FORMAT "_%04d%02d%02d-%02d%02d.data"
#define FILE_NAME_SUFFIX_LEN 19
// The format for data file names is: /sd/DEVICENAME_YYYYMMDD-hhmm.data
#define FILE_NAME_FORMAT FILE_NAME_PREFIX_FORMAT FILE_NAME_SUFFIX_FORMAT

// Maximum length of file name string (including null terminator)
#define FILE_NAME_SIZE (FILE_NAME_PREFIX_MAX_LEN + FILE_NAME_SUFFIX_LEN + 1)

// Path to config file on the SD card
#define CONFIG_FILE SD_MOUNT_POINT "/config.json"
#define CONFIG_FILE_BUFFER_SIZE 512

#define CONFIG_JSON_DEVICE_NAME "device_name"
#define CONFIG_JSON_SAMPLE_FREQUENCY "sample_frequency_hz"

// A log file will be created on the SD card for capturing diagnostics and 
// errors. If the SD card has a config.txt file with a device name, the log
// will be at /devicename.log. Otherwise, the log file will be based on the
// default device name /DIY-XX.log.
#define LOG_FILE_NAME_PREFIX SD_MOUNT_POINT "/"
#define LOG_FILE_NAME_SUFFIX ".log"
#define DEFAULT_LOG_FILE_NAME LOG_FILE_NAME_PREFIX DEFAULT_DEVICE_NAME LOG_FILE_NAME_SUFFIX
#define LOG_FILE_NAME_SIZE ((sizeof(DEFAULT_LOG_FILE_NAME) >= sizeof(LOG_FILE_NAME_PREFIX LOG_FILE_NAME_SUFFIX) + DEVICE_NAME_SIZE - 1) ? \
    sizeof(DEFAULT_LOG_FILE_NAME): \
    sizeof(LOG_FILE_NAME_PREFIX LOG_FILE_NAME_SUFFIX) + DEVICE_NAME_SIZE - 1)

// Data files use a header to describe their contents. Every data file starts 
// with a 4 character string of the format 'Vxxx' where x is a digit. Header 
// and body formats are documented on the GitHub repository.
#define DATA_HEADER_VERSION_STRING "V000"

#define ULP_MS5803_CONVERSION_DELAY_US 10000

// Approximate duration of a single execution of the ULP program (not including 
// the wait loop). The added number is determined empirically and can be 
// adjusted if the program gets longer or shorter.
#define ULP_PROGRAM_DURATION_US (2 * ULP_MS5803_CONVERSION_DELAY_US + 800)

// The number of ULP processor cycles per iteration of the wait loop.
#define ULP_CYCLES_PER_WAIT_LOOP 32
// The intended amount of time for the ULP to spend in the wait loop
// over the course of a second, used for initially calculating how many 
// iterations of the wait loop the ULP should perform each sample.
#define ULP_WAIT_LOOP_TOTAL_TIME_MS 50
#define ULP_SLEEP_DURATION_US(sample_frequency) (((1000 - ULP_WAIT_LOOP_TOTAL_TIME_MS) / sample_frequency) * 1000 - ULP_PROGRAM_DURATION_US)
#define ULP_WAIT_LOOP_ITERATIONS_ESTIMATE(sample_frequency) (((hulp_get_fast_clk_freq() / 1000) * (ULP_WAIT_LOOP_TOTAL_TIME_MS / sample_frequency)) / ULP_CYCLES_PER_WAIT_LOOP)

// Maximum sample frequency of the device.
// MUST BE A POWER OF 2
// Intended sampling rates are 1, 2, or 4 Hz. Could probably work with 8 Hz, any
// higher might be unstable.
#define MAX_SAMPLE_FREQUENCY 4
#define VALID_SAMPLEING_FREQUENCIES "1, 2, 4"
#define IS_VALID_SAMPLING_FREQUENCY(x) (x == 1 || x == 2 || x == 4)

// Number of readings (pressure and temperature) to store in a buffer before we 
// dump to the SD card.
#define BUFFER_SIZE 440
static_assert(BUFFER_SIZE % MAX_SAMPLE_FREQUENCY == 0, "BUFFER_SIZE must be a multiple of MAX_SAMPLE_FREQUENCY");
// Number of ulp_var_t's needed for one raw pressure and temperature reading.
// (24 bits for raw pressure, 24 bits for raw temperature; could be packed 
// in only 3 spaces with more ULP processing).
#define ULP_SAMPLE_SIZE 4
// Size of the buffer that the ULP will use to store raw data from the MS5803.
#define ULP_BUFFER_SIZE (BUFFER_SIZE * ULP_SAMPLE_SIZE)

// After dumping data to the SD card, we'll wait this long before shutting off 
// power to the SD card.
#define SD_OFF_DELAY_MS 2000

// If set to 1, whenever the device wakes up it will print all pressure readings
// to serial. This can take substantial time (a second or two) which could be
// saved to reduce power consumption. Set to 0 to disable.
#define LOG_PRESSURE_ON_WAKE 0

#define WIFI_SERVER_PORT 80
#define DNS_SERVER_PORT 53

#define DIY3

// Domain at which the WiFi dashboard can be reached after connecting to the 
// ESP's WiFi. 
#define DASHBOARD_DOMAIN "dashboard.lan"

#define SERVER_MODE_LED_FLASH_PERIOD_MS 100

static const char* TAG = "transducer";
static const char* TAG_DASHBOARD = "dashboard";
static const char* TAG_CONFIG = "transducer-config";

// TODO: Move all WiFi / dashboard functionality to its own file
void start_dashboard();
bool sd_init();
void sd_deinit();

// Globals for SD card
sdmmc_card_t* card;
sdspi_device_config_t sdspi_device_config = SDSPI_DEVICE_CONFIG_DEFAULT();

// Buffer to use for storing data to be written to the SD card. We allocate it
// as a global variable to avoid stack overflow, which would otherwise happen
// (with the default stack size of under 4K) with a buffer size a bit bigger 
// than 200.
float write_buffer[BUFFER_SIZE];

// Button press flag, set when the extra button on the FireBeetle is pressed.
RTC_FAST_ATTR volatile bool button_pressed = false;

// Initialize a sensor object for interacting with the MS5803-05
RTC_FAST_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

// Store the device's name.
RTC_FAST_ATTR char device_name[DEVICE_NAME_SIZE] = {0};
// Store the name for the current data file across deep sleep restarts.
RTC_FAST_ATTR char output_file_name[FILE_NAME_SIZE] = {0};
// Store the name for the log file.
RTC_FAST_ATTR char log_file_name[LOG_FILE_NAME_SIZE] = DEFAULT_LOG_FILE_NAME;
// Store the timezone offset string: UTC+hh:mm or UTC-hh:mm

// Track the number of times a custom error has been encountered causing the
// device to restart.
RTC_NOINIT_ATTR uint8_t restart_count;

// Store the day of the month when the current output file was created. This is
// used to determine when a new day has started (and thus when to start a new 
// file).
RTC_FAST_ATTR uint8_t day_of_month = 0;

// Controls the number of samples that will be taken per second.
RTC_FAST_ATTR uint8_t sample_frequency = MAX_SAMPLE_FREQUENCY;

// Store the epoch time for the first sensor reading in the buffer. This will be
// stored in the output file along with the sensor data. This is updated every 
// time the buffer is dumped.
RTC_FAST_ATTR uint32_t first_sample_timestamp;

// Set in the deep sleep wake stub. Controls whether the main task needs to
// resynchronize with the ULP when a sample was skipped or the ULP was already
// awake during the deep sleep wake stub.
RTC_FAST_ATTR bool ulp_sample_buffer_offset_was_reset;

// An array for the ULP to buffer raw sensor data while the main processor is in
// deep sleep. Once full, the data will be processed into meaningful pressure
// and temperature values and written to flash storage.
RTC_DATA_ATTR ulp_var_t ulp_sample_buffer[ULP_BUFFER_SIZE];
RTC_FAST_ATTR ulp_var_t ulp_sample_buffer_copy[ULP_BUFFER_SIZE];
// Used to index the ULP's buffer in the ULP program.
RTC_DATA_ATTR ulp_var_t ulp_sample_buffer_offset{};
// ulp_sample_buffer_offset will be copied into here in the deep sleep wake stub
// when we wake up. It should always be ULP_BUFFER_SIZE unless we woke up early
// from a shutdown button press.
RTC_FAST_ATTR uint16_t ulp_end_sample_buffer_offset;
// The duration the ULP will sleep between samples. This value is adjusted each
// time the main core wakes up to keep the sample frequency as accurate as 
// possible.
RTC_DATA_ATTR uint32_t ulp_sleep_duration_us;

// Flag used in the ULP program to track whether the MS5803's raw D2 value has
// been read.
RTC_DATA_ATTR ulp_var_t ulp_d2_flag{};
// Used in the ULP program to track the number of samples taken in the
// current second in order to implement >1 Hz sampling
RTC_DATA_ATTR ulp_var_t ulp_samples_this_second{};
// If the main processor takes too long to wake up, the ULP will record how 
// many samples it would have taken instead of taking samples to avoid 
// overwriting the buffer before it has been copied.
RTC_DATA_ATTR ulp_var_t ulp_skipped_readings{};
// The number of iterations of the wait loop the ULP should perform to sync 
// with the DS3231. This value is adjusted by the ULP to compensate when running
// fast or slow.
RTC_DATA_ATTR ulp_var_t ulp_wait_loop_iterations{};

// The following variables are used by HULP macros to communicate via bitbanged
// I2C.

// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to 
// take a reading and produce a raw value (D1) that we can read with subsequent 
// instructions. (OSR = 4096, see datasheet for details).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convert_d1[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x48, 0),
};
// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to
// take a reading and produce a raw value (D2) that we can read with subsequent 
// instructions. (OSR = 4096, see datasheet for details).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convert_d2[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x58, 0),
};
// HULP bitbanged I2C instruction. Writes a command to the MS5803 telling it to
// prepare to send the last raw reading (24 bits long).
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_read_adc[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x0, 0),
};
// HULP bitbanged I2C instruction. Reads a raw 24 bit value from the MS5803.
RTC_DATA_ATTR ulp_var_t ulp_i2c_read_sensor[HULP_I2C_CMD_BUF_SIZE(3)] = {
    HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 3)
};
// HULP bitbanged I2C instruction. Writes to the DS3231's status register, 
// clearing it. This is done to clear an active alarm. If the state of the 
// status register needs to be maintained, the ULP program must be modified.
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_clear_alarm[] = {
    HULP_I2C_CMD_HDR(DS3231_I2C_ADDR, DS3231_STATUS_ADDR, 1),
    HULP_I2C_CMD_1B(0x0)
};
// HULP bitbanged I2C instruction. Reads the time from the DS3231.
RTC_DATA_ATTR ulp_var_t ulp_i2c_read_time[HULP_I2C_CMD_BUF_SIZE(7)] = {
    HULP_I2C_CMD_HDR(DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 7)
};

bool toggle_led = false;
bool sd_initialized = false;


// Error codes for the program. The value associated with each enum is also the
// number of times the error LED will flash if an error is encountered.
enum transducer_error_t {
    TRANSDUCER_DS3231_ERROR = 1,
    TRANSDUCER_SD_INIT_ERROR,
    TRANSDUCER_SD_CONFIG_WARNING,
    TRANSDUCER_SD_FILE_ERROR,
    TRANSDUCER_MS5803_ERROR,
    TRANSDUCER_RESET_ERROR,
    TRANSDUCER_ULP_ERROR,
};


/* Deep sleep wake stub. This runs very early in the ESP's boot / wake sequence 
 * when waking from deep sleep. The goal is to copy the ULP's sample buffer as 
 * quickly as possible so that it doesn't miss any samples. We would almost 
 * certainly miss samples without this stub at anything greater than 1Hz 
 * sampling due to the ESP's boot time. 
 */
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    uint32_t wakeup_time = esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us();
    ESP_RTC_LOGI("Deep sleep wake stub started");

    ESP_RTC_LOGI("ULP sample buffer offset is %d, time since boot is %ld us", ulp_sample_buffer_offset.val, wakeup_time);

    memcpy(ulp_sample_buffer_copy, ulp_sample_buffer, ULP_BUFFER_SIZE * sizeof(ulp_sample_buffer[0]));

    ESP_RTC_LOGI("Finished copying data from buffer (took %ld us)", esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us() - wakeup_time);

    // This code is taken from the hulp_get_state() function and modified not to 
    // call ESP_LOGW which probably shouldn't be called from a wakeup stub.
    ulp_state_t state;
    // If a sample is missed, it is possible that the ULP is still in the middle 
    // of its program when we get to this part of the stub. To avoid an edge 
    // case, we don't want to reset the buffer offset to 0 until the ULP is 
    // asleep, so we'll wait until then. Also, if a sample was skipped, we want 
    // to wait until the start of a new second to avoid some complications with 
    // how timestamps are recorded.
    uint32_t ulp_state_bits = REG_READ(RTC_CNTL_LOW_POWER_ST_REG) & (0xF << 13);
    switch(ulp_state_bits) {
        case 0:
            state = ULP_STATE_IDLE;
            break;
        case BIT(13) |  BIT(14):
            state = ULP_STATE_RUNNING;
            break;
        case BIT(13) |  BIT(14) |             BIT(16):
            state = ULP_STATE_HALTED;
            break;
        case                        BIT(15) | BIT(16):
            state = ULP_STATE_SLEEPING;
            break;
        case            BIT(14) |             BIT(16):
        case            BIT(14) |   BIT(15) | BIT(16):
        case BIT(13) |  BIT(14) |   BIT(15) | BIT(16): //if sleep time ~0
            state = ULP_STATE_WAKING;
            break;
        case                                  BIT(16):
            state = ULP_STATE_DONE;
            break;
        default:
            state = ULP_STATE_UNKNOWN;
    }
    // Save the offset so we know how many samples to copy (the only time 
    // the buffer shouldn't be full is when the shutdown button is pressed).
    ulp_end_sample_buffer_offset = ulp_sample_buffer_offset.val;

    // If the ULP has not skipped any samples and is not already waking up / 
    // running, we can safely reset the buffer offset and allow it to run again.
    // Otherwise, we will need to handle resyncing with the ULP in the main task
    // (doing it here could cause a watchdog reset). 
    if (ulp_skipped_readings.val == 0 && state != ULP_STATE_WAKING && state != ULP_STATE_RUNNING) {
        // Have the ULP start overwriting the sample buffer
        ulp_sample_buffer_offset.val = 0;
        ulp_sample_buffer_offset_was_reset = true;
    } else {
        ulp_sample_buffer_offset_was_reset = false;
        ESP_RTC_LOGW("Samples were skipped or the ULP is already awake. Not resetting the buffer offset yet");
    }

    ESP_RTC_LOGI("Deep sleep wake stub done (took %ld us)", esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us() - wakeup_time);
}


time_t ts_to_tm(ts ds3231_time, tm* system_time) {
    tm t;
    if (system_time == nullptr) {
        system_time = &t;
    }
    system_time->tm_year = ds3231_time.year - 1900; // years since 1900
    system_time->tm_mon = ds3231_time.mon - 1; // months since January
    system_time->tm_mday = ds3231_time.mday;
    system_time->tm_hour = ds3231_time.hour;
    system_time->tm_min = ds3231_time.min;
    system_time->tm_sec = ds3231_time.sec;

    return mktime(system_time);
}


/*
 * Flash the error LED a given number of times to warn the user.
 */
void flash(uint8_t flash_count) {
    for (uint8_t i = 0; i < flash_count; i++) {
        rtc_gpio_set_level(ERROR_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_DURATION));
        rtc_gpio_set_level(ERROR_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_DURATION));
    }
}


/*
 * NOTE: Don't call this function directly; use the TRANSDUCER_ERROR macro.
 * Try to restart the device or enter an endless error loop otherwise, flashing
 * the LED in a given sequence to indicate the error. If attempt_logging == true,
 * we attempt to write diagnostic info to a log file on the SD card. If another
 * error is generated during this process, we ignore it and skip logging, 
 * indicating the original error on the LED.
 */
[[noreturn]]
void transducer_error(transducer_error_t error_num, size_t line, bool attempt_logging) {
    // Disable ULP wakeups
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    // Stop ULP after it finishes next cycle.
    // The delay while flashing the LED below is long enough to ensure the ULP
    // is stopped so it won't interfere with I2C.
    hulp_ulp_end();
    
    ESP_LOGE(TAG, "Error code %d @ line: %u\n", error_num, line);

    // Flash the error sequence 3 times on the LED.
    for (uint8_t i = 0; i < 3; ++i) {
        flash(error_num);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Attempt to log data to SD card (this won't always be possible e.g. if
    // there was an error connecting to the SD card).
    if (attempt_logging) {
        // First we'll try to get the time.
        char time_string[16] = "unknown time";
        // TODO: Fix this approach with better DS3231 lib.

        ts time_now{};
        time_now.sec = 61;
        Wire.begin();
        rtc_gpio_set_level(RTC_POWER_PIN, 1);
        DS3231_get(&time_now);

        // If we successfully got the time from the DS3231, overwrite time_string
        // with the actual time.
        if (time_now.sec != 61) {
            snprintf(time_string, sizeof(time_string), "%04d%02d%02d %02d%02d%02d", time_now.year % 10000u, time_now.mon % 100u, time_now.mday % 100u, time_now.hour % 100u, time_now.min % 100u, time_now.sec % 100u);
        }

        // Then we'll try to open and write to a log file on the SD card
        // TODO: Perhaps save to ESP32 flash if we can't save to SD card?
        rtc_gpio_set_level(SD_SWITCH_PIN, 0); // Turn on SD power
        vTaskDelay(pdMS_TO_TICKS(100));
        if (sd_initialized || sd_init()) {      
            FILE* log_file = fopen(log_file_name, "a");
            if (log_file != nullptr) {
                ESP_LOGI(TAG, "Logging error to file");
                fprintf(log_file,
                    "ERROR CODE %d @ LINE %u\n"
                    "\tDATA_HEADER_VERSION_STRING=" DATA_HEADER_VERSION_STRING "\n"
                    "\tBUFFER_SIZE=%d\n"
                    "\tCONFIG_FILE=" CONFIG_FILE "\n"
                    "\tDevice name: %s\n"
                    "\tSample frequency: %d Hz\n"
                    "\tRTC time: %s\n"
                    "\tCurrent output file: %s\n"
                    "\tCurrent restart count: %d/%d\n"
                    "\tTimestamp of first sample in next dump: %ld\n"
                    "\tULP sleep duration: %lu us\n"
                    "\tULP wait loop iterations: %u (target/estimate: %lu)\n",
                    error_num, line, BUFFER_SIZE, device_name, sample_frequency, time_string,
                    output_file_name,
                    restart_count, MAX_RESTART_COUNT, first_sample_timestamp,
                    ulp_sleep_duration_us, ulp_wait_loop_iterations.val, 
                    ULP_WAIT_LOOP_ITERATIONS_ESTIMATE(sample_frequency)
                );
                
                if (restart_count < MAX_RESTART_COUNT) {
                    fprintf(log_file, "DEVICE WILL RESTART");
                    ESP_LOGI(TAG, "DEVICE WILL RESTART");
                } else {
                    fprintf(log_file, "DEVICE WILL NOT RESTART");
                    ESP_LOGI(TAG, "DEVICE WILL NOT RESTART");
                }
                fclose(log_file);
                log_file = nullptr;
            } else {
                ESP_LOGI(TAG, "Error opening log file");
            }
            sd_deinit();
        } else {
            ESP_LOGI(TAG, "Error initializing SD card");
        }
        rtc_gpio_set_level(SD_SWITCH_PIN, 1); // Turn off SD power
    }

    // Try to restart the device.
    if (restart_count < MAX_RESTART_COUNT) {
        for (uint8_t i = 0; i < 5; ++i) {
            rtc_gpio_set_level(ERROR_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_FLASH_DURATION / 4));
            rtc_gpio_set_level(ERROR_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(LED_FLASH_DURATION / 4));
        }
        ESP_LOGW(TAG, "Restarting");
        ++restart_count;
        esp_restart();
    } else {
        ESP_LOGW(TAG, "Max restart count reached. Going into endless deep sleep");
        // Otherwise, go into endless deep sleep.
        esp_deep_sleep_start();
    }
}


// Debugging utility function for monitory the ULP program while it is running
void monitor_ulp() {
    ESP_LOGI("MONITOR", "starting");

    int old = 0;
    timeval tv_now;
    gettimeofday(&tv_now, nullptr);
    int64_t last = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t last_check = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int count = 0;

    uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
    uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
    ESP_LOGI("MONITOR", "rtc_fast_freq = %ld", rtc_fast_freq_hz);

    while(1) {
        gettimeofday(&tv_now, nullptr);
        if (ulp_sample_buffer_offset.val != old) {
            old = ulp_sample_buffer_offset.val;
            int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            
            ESP_LOGI("MONITOR", "delta_ms=%lld count=%d ulp_processor_wait_cycles=%d wait_time_ms=%lu, calibration=%lx", (time_us - last) / 1000, count, ulp_wait_loop_iterations.val, ulp_wait_loop_iterations.val * 22 / (rtc_fast_freq_hz / 1000), esp_clk_slowclk_cal_get());
            last = time_us;
            count++;
        }
        last_check = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        
        vTaskDelay(1);

        if (rtc_gpio_get_level(BUTTON_PIN) == 0) {
            ulp_sample_buffer_offset.val = 0;
        }
    }
}


/* 
 * Define the ULP program, configure the ULP appropriately, and upload the 
 * program to the ULP. 
 */ 
void ulp_init()
{
    timeval tv_now;
    gettimeofday(&tv_now, nullptr);
    int64_t start_time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    
    // Define labels used in the ULP program. They are not defined in any 
    // meaningful order.
    enum {
        L_LOOP_FOR_D2,
        L_READ,
        L_WRITE,
        L_W_RETURN0,
        L_W_RETURN1,
        L_W_RETURN2,
        L_R_RETURN1,
        L_R_RETURN2,
        L_DONE,
        L_SAMPLE_AFTER_ALARM,
        L_SAMPLE_NO_ALARM,
        L_WAIT_FOR_ALARM,
        L_LAST_SAMPLE_THIS_SECOND,
        L_SAMPLE_WAIT_LOOP,
        L_WAIT_FOR_ALARM_FAST,
        L_WASTE_CYCLE_1,
        L_WASTE_CYCLE_2,
        L_WASTE_CYCLE_3,
        L_OVERFLOW,
        L_NO_OVERFLOW,
        L_D2_ALREADY_READ,
    };

    // Notes: 
    // - The following array contains instructions for programming the ULP 
    //   (ultra low power coprocessor) of the ESP32, which runs in parallel to
    //   the main processor and can run while the ESP32 is in deep sleep.
    // - In this case, programming of the ULP is done using the legacy macros.
    // - Bitbanged I2C is achieved using the HULP library. See the examples and
    //   code documentation of that project to understand the "recipe" to
    //   use bitbanged I2C on the ULP.
    const ulp_insn_t program[] = {
        I_MOVI(R1, 0),
        I_GET(R0, R1, ulp_samples_this_second),
        I_ADDI(R0, R0, 1),
        I_PUT(R0, R1, ulp_samples_this_second),
        
        // Load the samples per second variable
        // (This variable doesn't change while in deep sleep, so we hardcode it
        // into the ULP program each time it is loaded)
        I_MOVI(R2, sample_frequency),

        I_GET(R1, R1, ulp_wait_loop_iterations),
        I_MOVR(R3, R1), // Save this for later
        
        // If this is the last sample this second, wait for the RTC alarm first
        I_SUBR(R0, R2, R0),
        M_BL(L_LAST_SAMPLE_THIS_SECOND, 1),

        // Otherwise, wait then go take a sample
        M_LABEL(L_SAMPLE_WAIT_LOOP),    // cycles to execute + cycles to fetch next instruction
        I_SUBI(R1, R1, 1),              // 2 + 2 = 4 cycles
        M_BXZ(L_SAMPLE_NO_ALARM),       // 2 + 2 = 4 cycles
        // (Waste cycles to match the other wait loop)
        I_ADDI(R1, R1, 0),              // 2 + 2 = 4 cycles                     
        I_ADDI(R1, R1, 0),              // 2 + 2 = 4 cycles
        I_ADDI(R1, R1, 0),              // 2 + 2 = 4 cycles
        I_ADDI(R1, R1, 0),              // 2 + 2 = 4 cycles
        I_ADDI(R1, R1, 0),              // 2 + 2 = 4 cycles
        M_BX(L_SAMPLE_WAIT_LOOP),       // 2 + 2 = 4 cycles, total = 32

        M_LABEL(L_LAST_SAMPLE_THIS_SECOND),
        I_MOVI(R0, 0),
        I_PUT(R0, R0, ulp_samples_this_second),
        
        // Wait until the RTC alarm is triggered and track how many 
        // times we loop. We'll use this to adjust the sleep delay if we're 
        // running fast or slow.
        M_LABEL(L_WAIT_FOR_ALARM),
        I_SUBI(R1, R1, 1),                  // 2 + 2 = 4 cycles
        M_BXZ(L_WAIT_FOR_ALARM_FAST),       // 2 + 2 = 4 cycles
        I_GPIO_READ(RTC_ALARM_PIN),         // 4 + 4 = 8 cycles
        // (Waste some cycles to increase the wait loop time)
        I_ADDI(R1, R1, 0),                  // 2 + 2 = 4 cycles
        I_ADDI(R1, R1, 0),                  // 2 + 2 = 4 cycles
        I_ADDI(R1, R1, 0),                  // 2 + 2 = 4 cycles
        M_BGE(L_WAIT_FOR_ALARM, 1),         // 2 + 2 = 4 cycles, total = 32
        
        // If we get here, either we were perfectly on time or slow
        // Compensate: divide the remaining time (cycles) by the frequency 
        // and reduce the calibration delay by that amount.
        I_RSHI(R1, R1,  static_cast<uint8_t>(hulp_log2(sample_frequency))),
        I_SUBR(R3, R3, R1), // R3 is ulp_wait_loop_iterations
        I_MOVI(R1, 0),
        I_PUT(R3, R1, ulp_wait_loop_iterations),
        M_BX(L_SAMPLE_AFTER_ALARM),

        // If we get here, the RTC alarm hasn't gone off yet so we're fast
        // Keep looping and track it.
        M_LABEL(L_WAIT_FOR_ALARM_FAST),
        I_ADDI(R1, R1, 1),
        M_BXZ(L_WASTE_CYCLE_3), // Used so that one loop = same # of cycles as earlier loop
        M_LABEL(L_WASTE_CYCLE_3),
        I_GPIO_READ(RTC_ALARM_PIN),
        M_BGE(L_WAIT_FOR_ALARM_FAST, 1),  

        // Compensate: divide the cycles waited by the frequency and increase 
        // the calibration delay by that amount
        I_RSHI(R1, R1, static_cast<uint8_t>(hulp_log2(sample_frequency))),
        I_ADDR(R3, R3, R1), // R3 is ulp_wait_loop_iterations
        I_MOVI(R1, 0),
        I_PUT(R3, R1, ulp_wait_loop_iterations),

        M_LABEL(L_SAMPLE_AFTER_ALARM),
        
        // Bitbanged I2C instruction: clear the DS3231's status register to 
        // reset the wakeup alarm.
        I_MOVO(R1, ulp_i2c_write_clear_alarm),
        M_MOVL(R3, L_W_RETURN2),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN2),

        M_LABEL(L_SAMPLE_NO_ALARM),

        // Clear D2 read flag.
        I_MOVI(R0, 0),
        I_PUT(R0, R0, ulp_d2_flag),
        
        // Issue I2C command to MS5803 to start ADC conversion
        I_MOVO(R1, ulp_i2c_write_convert_d1),
        M_LABEL(L_LOOP_FOR_D2),     // Loop back here for D2 after D1
        M_MOVL(R3, L_W_RETURN0),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN0),
        
        // Wait 10ms for conversion
        M_DELAY_US_5000_20000(ULP_MS5803_CONVERSION_DELAY_US),

        // Issue I2C command to prepare to read the ADC value
        I_MOVO(R1, ulp_i2c_write_read_adc),
        M_MOVL(R3, L_W_RETURN1),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN1),

        // Read from sensor using I2C.
        I_MOVO(R1, ulp_i2c_read_sensor),
        M_MOVL(R3, L_R_RETURN1),
        M_BX(L_READ),
        M_LABEL(L_R_RETURN1),

        // Store value in buffer and increase buffer offset.

        // Get the current buffer offset
        I_MOVI(R1, 0),
        I_GET(R0, R1, ulp_sample_buffer_offset),
        
        // Check if the buffer is full
        M_BGE(L_OVERFLOW, ULP_BUFFER_SIZE),

        // If the buffer is not full, store the reading and increment the 
        // buffer offset
        I_GET(R2, R1, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET]),
        I_GET(R3, R1, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET + 1]),
        I_PUTO(R2, R0, 0, ulp_sample_buffer),
        I_PUTO(R3, R0, -1, ulp_sample_buffer), // stores to ulp_sample_buffer[offset + 1]
        I_ADDI(R0, R0, 2),
        I_PUT(R0, R1, ulp_sample_buffer_offset),
        M_BX(L_NO_OVERFLOW),

        // Otherwise if the buffer is full, increment the missed sample counter.
        // This shouldn't normally happen, but if the main processor is very 
        // slow to wake up for some reason, we don't want to overflow the 
        // buffer.
        M_LABEL(L_OVERFLOW),
        I_GET(R0, R1, ulp_skipped_readings),
        I_ADDI(R0, R0, 1),
        I_PUT(R0, R1, ulp_skipped_readings),

        M_LABEL(L_NO_OVERFLOW),

        // Loop back and repeat for D2 if it hasn't been read yet. If D2 was 
        // just read, jump past next block.
        I_GET(R0, R1, ulp_d2_flag),
        M_BGE(L_D2_ALREADY_READ, 1),

        // Set D2 flag before branching to get D2.
        I_MOVI(R3, 1),
        I_PUT(R3, R1, ulp_d2_flag),
        I_MOVO(R1, ulp_i2c_write_convert_d2),
        M_BX(L_LOOP_FOR_D2),

        // Branch here if we've already read D2.
        M_LABEL(L_D2_ALREADY_READ),

        // Check if the buffer is full and respond accordingly (i.e. wake up
        // processor).
        I_GET(R0, R1, ulp_sample_buffer_offset),
        M_BL(L_DONE, ULP_BUFFER_SIZE),

        // Read the time from the DS3231 if the buffer is full
        I_MOVO(R1, ulp_i2c_read_time),
        M_MOVL(R3, L_R_RETURN2),
        M_BX(L_READ),
        M_LABEL(L_R_RETURN2),

        I_WAKE(), // Wake up the main processor and halt below

        // Halt ULP program and go back to sleep.
        M_LABEL(L_DONE),
        I_HALT(),  // ^^ 82 instructions ^^

        // Include HULP "subroutines" for bitbanged I2C.
        M_INCLUDE_I2CBB_CMD(L_READ, L_WRITE, ULP_SCL_PIN, ULP_SDA_PIN), // 90 instructions
        // Total: 172 instructions (might be outdated if I forget to change it :) )
    };
    
    // Configure pins for use by the ULP.
    ESP_ERROR_CHECK(hulp_configure_pin(ULP_SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(ULP_SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));   

    hulp_peripherals_on();

    ulp_wait_loop_iterations.val = ULP_WAIT_LOOP_ITERATIONS_ESTIMATE(sample_frequency);
    ESP_LOGI(TAG, "Number of ULP wait loop iterations per sample: %d", ulp_wait_loop_iterations.val);
   
    // Determine how long the ULP should sleep between samples to achieve the 
    // desired sample frequency. This isn't perfect, but the ULP will actively 
    // compensate as it starts sampling.
    ulp_sleep_duration_us = ULP_SLEEP_DURATION_US(sample_frequency);
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), ulp_sleep_duration_us, 0));

    gettimeofday(&tv_now, nullptr);
    int64_t time_now_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    vTaskDelay(pdMS_TO_TICKS((ulp_sleep_duration_us - (time_now_us - start_time_us)) / 1000));
    
    ESP_ERROR_CHECK(hulp_ulp_run(0));
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

    ESP_LOGI(TAG, "Beginning shutdown");
    ESP_LOGI(TAG, "Stopping ULP");

    // Stop ULP after it finishes next cycle.
    hulp_ulp_end();

    ESP_LOGI(TAG, "Deinitializing RTC");
    // Enable power to RTC
    rtc_gpio_set_level(RTC_POWER_PIN, 1);
    // Wait to make sure RTC is on and ULP is done using I2C.
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    // Disable the RTC
    Wire.begin();
    // Set default values in control register and disable 32khz output. This
    // will also stop the alarm from pulsing when battery powered. 
    DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    // Clear any previous alarms.
    DS3231_clear_a1f();
    DS3231_clear_a2f();
    Wire.end();
    rtc_gpio_set_level(RTC_POWER_PIN, 0);

    // Turn on the LED to signal power can be removed. This function is used to
    // keep the LED on during deep sleep.
    hulp_configure_pin(ERROR_LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, HIGH);

    ESP_LOGI(TAG, "Power can now be turned off safely");

    // Keep LED on during sleep.
    hulp_peripherals_on();
    // Sleep forever (the ULP is disabled so it won't wake us up).
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}


bool sd_init() {
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config{};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

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

    spi_bus_config_t bus_cfg{};
    bus_cfg.mosi_io_num = SD_MOSI_PIN;
    bus_cfg.miso_io_num = SD_MISO_PIN;
    bus_cfg.sclk_io_num = SD_SCK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize((spi_host_device_t) host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config.gpio_cs = SD_CS_PIN;
    sdspi_device_config.host_id = (spi_host_device_t) host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &sdspi_device_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return false;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    // sdmmc_card_print_info(stdout, card);
    sd_initialized = true;
    return true;
}


DWORD sd_get_used() {
    if (!sd_initialized) {
        return 0;
    }

    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;
 
    // Get volume information and free clusters of drive 0
    if(f_getfree("0:", &fre_clust, &fs) == FR_OK)
    {
        // Get total sectors and free sectors
        tot_sect = (fs->n_fatent - 2) * fs->csize;
        fre_sect = fre_clust * fs->csize;
 
        return (tot_sect - fre_sect) * card->csd.sector_size;
    }
    return 0;
}


void sd_deinit() {
    ESP_ERROR_CHECK(esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card));
    ESP_ERROR_CHECK(spi_bus_free(sdspi_device_config.host_id));
    sd_initialized = false;
}


bool validate_device_name(char const* device_name, char const* logging_tag) {
    if (strlen(device_name) >= DEVICE_NAME_SIZE) {
        ESP_LOGI(logging_tag, "Device name is too long. Max %d characters", DEVICE_NAME_SIZE - 1);
        return false;
    }
    for (char const* c = device_name; *c != '\0'; c++) {
        if (!isalnum(*c) && *c != '-' && *c != '_') {
            ESP_LOGI(logging_tag, "Device name contains invalid character: '%c'. Only alphanumeric characters, dashes, and underscores are allowed", *c);
            return false;
        }
    }
    return true;
}


bool validate_sample_frequency(uint16_t sample_frequency, char const* logging_tag) {
    if (!IS_VALID_SAMPLING_FREQUENCY(sample_frequency)) {
        ESP_LOGI(logging_tag, "Invalid sampling frequency: %d. Valid values are: " VALID_SAMPLEING_FREQUENCIES, sample_frequency);
        return false;
    }
    return true;
}


bool parse_config(FILE* config_file) {
    bool success = false;
    char json_buffer[CONFIG_FILE_BUFFER_SIZE] = {0};
    size_t bytes_read = fread(json_buffer, 1, sizeof(json_buffer) - 1, config_file);
    if (bytes_read == 0) {
        ESP_LOGW(TAG_CONFIG, "Failed to read anything from config file");
        return false;
    } else if (bytes_read == CONFIG_FILE_BUFFER_SIZE) {
        ESP_LOGW(TAG_CONFIG, "Config file too large: max %d characters", CONFIG_FILE_BUFFER_SIZE - 1);
        return false;
    }

    cJSON* json_root = cJSON_Parse(json_buffer);
    const cJSON* json_device_name = cJSON_GetObjectItem(json_root, CONFIG_JSON_DEVICE_NAME);
    const cJSON* json_sample_frequency = cJSON_GetObjectItem(json_root, CONFIG_JSON_SAMPLE_FREQUENCY);
    if (json_root == nullptr) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != nullptr) {
            ESP_LOGW(TAG_CONFIG, "Error parsing JSON (before: %s)", error_ptr);
        }
        goto end;
    }

    // Validate device name
    if (!cJSON_IsString(json_device_name) || json_device_name->valuestring == nullptr) {
        ESP_LOGW(TAG_CONFIG, "Failed to read JSON field '" CONFIG_JSON_DEVICE_NAME "'. Field value must be a string");
        goto end;
    }
    if (!validate_device_name(json_device_name->valuestring, TAG_CONFIG)) {
        goto end;
    }

    // Validate sample frequency
    if (!cJSON_IsNumber(json_sample_frequency)) {
        ESP_LOGW(TAG_CONFIG, "Failed to read JSON field '" CONFIG_JSON_SAMPLE_FREQUENCY "'. Field value must be numeric");
        goto end;
    }
    if (!validate_sample_frequency(json_sample_frequency->valueint, TAG_CONFIG)) {
        goto end;
    }

    // Set global config variables
    strcpy(device_name, json_device_name->valuestring);
    sample_frequency = json_sample_frequency->valueint;
    
    // Cleanup
    success = true;
end:
    cJSON_Delete(json_root);
    return success;
}


bool write_config() {
    char* json_output;
    FILE* f;

    bool success = false;
    cJSON *config = cJSON_CreateObject();
    if (config == nullptr) {
        goto end1;
    }
    if (cJSON_AddStringToObject(config, CONFIG_JSON_DEVICE_NAME, device_name) == nullptr) {
        goto end1;
    }
    if (cJSON_AddNumberToObject(config, CONFIG_JSON_SAMPLE_FREQUENCY, sample_frequency) == nullptr) {
        goto end1;
    }

    json_output = cJSON_Print(config);
    if (json_output == nullptr) {
        goto end1;
    }

    f = fopen(CONFIG_FILE, "w");
    if (f == nullptr) {
        goto end2;
    }    

    if (fwrite(json_output, sizeof(json_output[0]), strlen(json_output), f) == 0) {
        goto end3;
    }

    success = true;
end3:
    fclose(f);
end2:
    free(json_output);
end1:
    cJSON_Delete(config);
    return success;
}


/* Writes header information to the provided file pointer, which should point to
 * the top of the file. All headers start with a 4 byte version string: "Vxxx"
 * where x is a digit '0' to '9'.
 * Everything after is subject to change. Header / body formats should be
 * documented on the GitHub repo by header version. 
 */
size_t write_data_header(FILE* f) {
    size_t bytes_written = 0;
    // Write the version string first. Don't include the null terminator.
    bytes_written += fwrite(DATA_HEADER_VERSION_STRING, sizeof(char), sizeof(DATA_HEADER_VERSION_STRING) - 1, f) * sizeof(char);
    
    // The 2nd field (DEVICE_NAME_SIZE==16 bytes) is the device name. Uses a null terminator.
    bytes_written += fwrite(device_name, sizeof(char), sizeof(device_name), f) * sizeof(char);

    // The 3rd field (1 byte) is the sampling rate.
    bytes_written += fwrite(&sample_frequency, sizeof(sample_frequency), 1, f) * sizeof(sample_frequency);

    // The 4th field (2 bytes) is the number of samples per dump to the SD card.
    const uint16_t buffer_size = BUFFER_SIZE;
    bytes_written += fwrite(&buffer_size, sizeof(buffer_size), 1, f) * sizeof(buffer_size);

    return bytes_written;
}


void IRAM_ATTR button_interrupt() {
    button_pressed = true;
}


extern "C" void app_main() {
    bool button_pressed_at_startup = false;
    
    // Jump to the appropriate code depending on whether the ESP32 was just 
    // powered or just woke up from deep sleep. 
    switch (esp_reset_reason()) {   
        case ESP_RST_POWERON: {
            ESP_ERROR_CHECK(hulp_configure_pin(ERROR_LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));  
            ESP_ERROR_CHECK(hulp_configure_pin(BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));  
            ESP_ERROR_CHECK(hulp_configure_pin(RTC_POWER_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1));  
            ESP_ERROR_CHECK(hulp_configure_pin(RTC_ALARM_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));  
            #ifdef DIY4
                // Enable SD power when configuring the pin
                ESP_ERROR_CHECK(hulp_configure_pin(SD_SWITCH_PIN RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));  
            #endif
            
            restart_count = 0;
            if (rtc_gpio_get_level(BUTTON_PIN) == 0) {
                button_pressed_at_startup = true;
                ESP_LOGI(TAG, "Button press detected at startup. Device will enter server mode after initializing");
            }
        }
        [[fallthrough]];
        case ESP_RST_SW: {
            // After setup, the button can be used to safely "shut down" the ESP32 and 
            // disable active peripherals.
            attachInterrupt(BUTTON_PIN, button_interrupt, FALLING);
        
            ESP_LOGI(TAG, "Starting setup");
            ESP_LOGI(TAG, "Initializing RTC");

            Wire.begin();
            // Initialize the connection with the RTC:
            // Power to the RTC is provided by a GPIO pin. Providing main power
            // to the DS3231 causes it to drain more current, but will preserve
            // the backup battery which is used most of the time. We provide 
            // main power here in case the device has not yet been initialized.
            rtc_gpio_set_level(RTC_POWER_PIN, 1);

            // According to the datasheet, the oscillator takes <1 sec to begin (on 
            // first time startup), so we wait 1 second in case this is the first time
            // startup. 
            vTaskDelay(pdMS_TO_TICKS(1000)); 

            // Set default values in control register and disable 32khz output.
            // TODO: Need a way to make sure this succeeds.
            DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
            // Clear any previous alarms.
            DS3231_clear_a1f();
            DS3231_clear_a2f();

            // Set the current day value.
            ts time_now;
            DS3231_get(&time_now);
            day_of_month = time_now.mday;

            ESP_LOGI(TAG, "RTC initialized. Time is %d-%02d-%02d %02d:%02d:%02d", time_now.year, time_now.mon, time_now.mday, time_now.hour, time_now.min, time_now.sec);

            // TODO: Check Status register as well, as it indicates if time was 
            // set since powered on (See #1)
            // Check to see if the date is earlier than the year in which the 
            // program was compiled.
            if (time_now.year < ('0' - __DATE__[8]) * 1000 + ('0' - __DATE__[9]) * 100 + ('0' - __DATE__[10]) * 10 + ('0' - __DATE__[11])) {
                ESP_LOGW(TAG, "RTC time is out of date. Setting year to 2000");
                time_now.year = 2000;
                DS3231_set(time_now);
                ESP_LOGW(TAG, "New RTC time is %d-%02d-%02d %02d:%02d:%02d", time_now.year, time_now.mon, time_now.mday, time_now.hour, time_now.min, time_now.sec);
            }

            // Only start the alarm if we aren't going into server mode.
            if (!button_pressed_at_startup) {
                ESP_LOGI(TAG, "Starting RTC once-per-second alarm");
                // Start the once-per-second alarm on the DS3231:
                // These flags set the alarm to be in once-per-second mode.
                const uint8_t flags[] = {1,1,1,1,0};
                // Set the alarm. The time value doesn't matter in once-per-second
                // mode.
                DS3231_set_a1(1, 1, 1, 1, flags);
                // Enable the alarm. It will now bring the RTC_ALARM_PIN GPIO low every
                // second.
                DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);
            }

            // Initialize and mount the SD card
            if (!sd_init()) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_INIT_ERROR, false);
            }
            uint64_t sd_capacity_kb = ((uint64_t) card->csd.capacity) * card->csd.sector_size / 1024;
            ESP_LOGI(TAG, "Card size: %llu KiB", sd_capacity_kb);
            uint32_t sd_used_kb = sd_get_used() / 1024;
            ESP_LOGI(TAG, "Card used: %lu KiB", sd_used_kb);
            

            // Get config from SD card or use default.
            ESP_LOGI(TAG, "Looking for config file at " CONFIG_FILE);
            FILE* config = fopen(CONFIG_FILE, "r");
            bool config_error = false;
            if (config == nullptr) {
                ESP_LOGW(TAG, "Unable to find config file");
                config_error = true;
            } else if (!parse_config(config)) {
                config_error = true;
            } 
            fclose(config);
            config = nullptr;

            if (config_error) {
                strcpy(device_name, DEFAULT_DEVICE_NAME);
                strcpy(log_file_name, DEFAULT_LOG_FILE_NAME);
                sample_frequency = MAX_SAMPLE_FREQUENCY;
                ESP_LOGW(TAG, "Using default config values");
                flash(TRANSDUCER_SD_CONFIG_WARNING);
            }

            ESP_LOGI(TAG, "Device name is %s", device_name);
            ESP_LOGI(TAG, "Sample frequency is %d Hz", sample_frequency);

            snprintf(log_file_name, LOG_FILE_NAME_SIZE, LOG_FILE_NAME_PREFIX "%s" LOG_FILE_NAME_SUFFIX, device_name);
            ESP_LOGI(TAG, "Log file name will be %s", log_file_name);
            
            // Generate the first file.
            ESP_LOGI(TAG, "Creating first data output file");
            snprintf(output_file_name, FILE_NAME_SIZE, FILE_NAME_FORMAT, device_name, time_now.year % 10000, time_now.mon % 100, time_now.mday % 100, time_now.hour % 100, time_now.min % 100);
            FILE* f = fopen(output_file_name, "w");
            if (f == nullptr) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, true);
            }
            if (write_data_header(f) == 0) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, true);
            } 
            fclose(f);
            f = nullptr;
            ESP_LOGI(TAG, "First data output file is %s", output_file_name);
            
            ESP_LOGI(TAG, "Initializing MS5803 pressure sensor");
            if (!sensor.initializeMS_5803(false)) {
                TRANSDUCER_ERROR(TRANSDUCER_MS5803_ERROR, true);
            }
            
            ESP_LOGI(TAG, "Taking a sample sensor reading");
            sensor.readSensor();
            ESP_LOGI(TAG, "Pressure: %f mbar \tTemperature: %f deg C", sensor.pressure(), sensor.temperature());

            ESP_LOGI(TAG, "Writing diagnostics to log file");
            FILE* log_file = fopen(log_file_name, "a");
            if (!log_file) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, false);
            }

            const esp_app_desc_t* app_info = esp_app_get_description();

            fprintf(log_file,
                "SETUP COMPLETE\n"
                "\tDATA_HEADER_VERSION_STRING=" DATA_HEADER_VERSION_STRING "\n"
                "\tBUFFER_SIZE=%d\n"
                "\tCONFIG_FILE=" CONFIG_FILE "\n"
                "\tDevice name: %s\n"
                "\tSample frequency: %d Hz\n"
                "\tRTC time: %d-%02d-%02d %02d:%02d:%02d\n"
                "\tSample pressure: %f mbar\n"
                "\tSample temp: %f deg C\n"
                "\tCode uploaded on: " __DATE__ " @ " __TIME__ "\n"
                "\tCode version: %s\n" 
                "\tSD capacity: %llu KiB\n"
                "\tSD used: %lu KiB\n"
                "\tCurrent output file: %s\n"
                "\tCurrent restart count: %d/%d\n"
                "\tTimestamp of first sample in next dump: %ld\n",
                BUFFER_SIZE, device_name, sample_frequency, time_now.year,
                time_now.mon, time_now.mday, time_now.hour, time_now.min, 
                time_now.sec, sensor.pressure(), sensor.temperature(), 
                app_info->version, sd_capacity_kb, sd_used_kb, output_file_name,
                restart_count, MAX_RESTART_COUNT, first_sample_timestamp,
            );
            fclose(log_file);
            log_file = nullptr;

            ESP_LOGI(TAG, "Deinitializing SD card");
            sd_deinit();

            // Flash LED's to signal successful startup.
            ESP_LOGI(TAG, "Flashing LED");
            rtc_gpio_set_level(ERROR_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(3000));
            rtc_gpio_set_level(ERROR_LED_PIN, 0);

            if (button_pressed_at_startup) {
                ESP_LOGI(TAG, "Entering server mode");
                start_dashboard();          
            }

            // TODO: only if DIY4 
            ESP_LOGI(TAG, "Disabling SD card power and getting updated time");

            #ifdef DIY4
                rtc_gpio_set_level(SD_SWITCH_PIN, 1); // Turn off SD card power
            #endif

            DS3231_get(&time_now);
            // Save a timestamp for the first sample. We'll add 1 second
            // (alarmStatus) to the time since the first sample will actually be
            // taken during the next second.
            first_sample_timestamp = time_now.unixtime + 1;

            ESP_LOGI(TAG, "Time is %d-%02d-%02d %02d:%02d:%02d", time_now.year, time_now.mon, time_now.mday, time_now.hour, time_now.min, time_now.sec);
            
            ESP_LOGI(TAG, "Waiting for next second and clearing alarm"); 
            while (time_now.unixtime < first_sample_timestamp) {
                DS3231_get(&time_now);
            }
            DS3231_clear_a1f();

            ESP_LOGI(TAG, "Disabling DS3231 pin power");
            rtc_gpio_set_level(RTC_POWER_PIN, 0);
           
            // Reset the restart count if a succesful startup occurs.
            restart_count = 0;

            ESP_LOGI(TAG, "Starting ULP");
            ulp_init();

            // Allow the ULP to trigger the ESP32 to wake up.
            ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
            ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(((uint64_t) 0b1) << BUTTON_PIN, ESP_EXT1_WAKEUP_ALL_LOW));

            ESP_LOGI(TAG, "Setup complete! Going to sleep...");
            esp_deep_sleep_start();
        }

        // TODO: Check if DS3231 backup battery failed and switch to main power
        // TODO: Cleanup Serial logging / debug messages
        case ESP_RST_DEEPSLEEP: {
            ESP_LOGI(TAG, "Awake!");
            
            const int32_t estimated_ulp_wait_loop_iterations = ULP_WAIT_LOOP_ITERATIONS_ESTIMATE(sample_frequency);
            const int32_t delta_ulp_wait_loop_iterations = ulp_wait_loop_iterations.val - estimated_ulp_wait_loop_iterations;
            int32_t delta_ulp_sleep_duration_us = (delta_ulp_wait_loop_iterations * ULP_CYCLES_PER_WAIT_LOOP * 1000000ll) / (int64_t) (hulp_get_fast_clk_freq());
            ESP_LOGI(TAG, "Current ULP sleep duration: %lu us", ulp_sleep_duration_us);
            ESP_LOGI(TAG, "ULP wait loop iterations: %d iterations", ulp_wait_loop_iterations.val);
            ESP_LOGI(TAG, "New ULP wait loop iterations estimate is: %lu iterations", estimated_ulp_wait_loop_iterations);
            ESP_LOGI(TAG, "Change in ULP sleep duration: %ld us", delta_ulp_sleep_duration_us);
            
            const int32_t max_delta_ulp_sleep_duration_us = (ULP_WAIT_LOOP_TOTAL_TIME_MS * 1000 / sample_frequency) / 2;
            if (abs(delta_ulp_sleep_duration_us) > max_delta_ulp_sleep_duration_us) {
                delta_ulp_sleep_duration_us = delta_ulp_sleep_duration_us < 0 ? -max_delta_ulp_sleep_duration_us : max_delta_ulp_sleep_duration_us;
                ESP_LOGW(TAG, "Change in ULP sleep duration is too large to do all at once. Reducing change to: %ld us", delta_ulp_sleep_duration_us);
            }
            
            ulp_sleep_duration_us += delta_ulp_sleep_duration_us;
            if (ulp_set_wakeup_period(0, ulp_sleep_duration_us) != ESP_OK) {
                TRANSDUCER_ERROR(TRANSDUCER_ULP_ERROR, true);
            }

            // Shutdown button was pressed
            if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
                hulp_ulp_end();
                ESP_LOGI(TAG, "Shutdown button was pressed"); 
                button_pressed = true;
                // Wait until ULP is done running so we don't miss a sample
                ulp_state_t state = hulp_get_state();
                while (state == ULP_STATE_RUNNING || state == ULP_STATE_WAKING);
            }
            
            // If samples were skipped or the ULP was already awake during the 
            // deep sleep stub, wait until the next second then reset the sample
            // buffer offset. 
            if (!ulp_sample_buffer_offset_was_reset) {
                ulp_state_t state;
                do {
                    state = hulp_get_state();
                    vTaskDelay(1);
                } while ((state == ULP_STATE_RUNNING || state == ULP_STATE_WAKING) 
                    || ulp_skipped_readings.val % (2 * sample_frequency) != 0
                );

                // Allow the ULP to start sampling again.
                ulp_sample_buffer_offset.val = 0;

                ESP_LOGW(TAG, "Missed %d samples", ulp_skipped_readings.val / 2);
                ulp_skipped_readings.val = 0;
            } 

            // Process all the raw data using the conversion sequence specified
            // in the MS5803 datasheet. 
            // First, save the first reading's temperature.
            {
                const uint32_t var_d1 = (ulp_sample_buffer_copy[0].val << 8) | (ulp_sample_buffer_copy[1].val >> 8);
                const uint32_t var_d2 = (ulp_sample_buffer_copy[2].val << 8) | (ulp_sample_buffer_copy[3].val >> 8);
                sensor.convertRaw(var_d1, var_d2);
            }
            const float first_sample_temperature = sensor.temperature();
            // Then loop through for all the pressures.
            uint16_t num_samples = 0;
            for (uint16_t j = 0; j < ulp_end_sample_buffer_offset; num_samples++, j += ULP_SAMPLE_SIZE) {
                // Read the next D1 value from the raw data.
                const uint32_t var_d1 = (ulp_sample_buffer_copy[j].val << 8) | (ulp_sample_buffer_copy[j + 1].val >> 8);
                // Read the next D2 value from the raw data.
                const uint32_t var_d2 = (ulp_sample_buffer_copy[j + 2].val << 8) | (ulp_sample_buffer_copy[j + 3].val >> 8);

                // Convert raw D1 and D2 to pressure and temperature.
                sensor.convertRaw(var_d1, var_d2);
                // Write the processed data to the buffer.
                write_buffer[num_samples] = sensor.pressure();
            }

            ESP_LOGI(TAG, "Dumping to card");
            ESP_LOGI(TAG, "First Timestamp=%ld", first_sample_timestamp);
            ESP_LOGI(TAG, "Temperature at first sample: %.2f degrees C", first_sample_temperature);
            ESP_LOGI(TAG, "Number of samples: %d", num_samples);
            #if LOG_PRESSURE_ON_WAKE
                ESP_LOGI(TAG, "Pressure (mbar)");
                for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
                    ESP_LOGI(TAG, "%.2f", write_buffer[i]);
                }
            #else 
                ESP_LOGI(TAG, "First pressure sample: %.2f mbar", write_buffer[0]);
            #endif
            #ifdef DIY4
                // Reinitialize connection with SD card.
                ESP_LOGI(TAG, "Turning on SD power");
                rtc_gpio_set_level(SD_SWITCH_PIN, 0); // Turn on SD card power
                vTaskDelay(pdMS_TO_TICKS(100)); // Some sensors were erroring here, so maybe delay is needed?
            #endif
            if (!sd_init()) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_INIT_ERROR, false);
            }

            ESP_LOGI(TAG, "Card capacity used: %ld KiB", sd_get_used() / 1024);

            // Open the current data file.
            FILE* f = fopen(output_file_name, "a");
            if (f == nullptr) {
                TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, true);
            }
            // Write timestamp of first sample (4 bytes).
            size_t written = fwrite(&first_sample_timestamp, sizeof(first_sample_timestamp), 1, f) * sizeof(first_sample_timestamp);
            // Write the data to the file in binary format. The temperature 
            // (4 bytes) goes first, followed by BUFFER_SIZE pressure readings
            // (4 bytes each).
            written += fwrite(&first_sample_temperature, sizeof(first_sample_temperature), 1, f) * sizeof(first_sample_timestamp);
            written += fwrite(write_buffer, sizeof(write_buffer[0]), num_samples, f) * sizeof(write_buffer[0]);
            fclose(f);
            f = nullptr;
            
            ESP_LOGI(TAG, "Wrote %u bytes to file", written);

            // We can skip this chunk of code if we're about to shut down
            if (!button_pressed) {
                // Right before the ULP wakes up the main processor, it reads and 
                // stores the time from the DS3231. We decode that here (assuming 
                // some things e.g. 24 hour time).
                ts ds3231_time{};
                ds3231_time.sec = (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET].val >> 12 & 0xf) * 10 + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET].val >> 8 & 0xf);
                ds3231_time.min = (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET].val >> 4 & 0xf) * 10 + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET].val & 0xf);
                ds3231_time.hour = (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 1].val >> 12 & 0xf) * 10 + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 1].val >> 8 & 0xf);
                ds3231_time.mday = (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 2].val >> 12 & 0xf) * 10 + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 2].val >> 8 & 0xf);
                ds3231_time.mon = (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 2].val >> 4 & 1) * 10 + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 2].val & 0xf);
                ds3231_time.year = 1900 
                    + 100 * (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET + 2].val >> 7 & 1) 
                    + 10 * (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET+ 3].val >> 12 & 0xf)
                    + (ulp_i2c_read_time[HULP_I2C_CMD_DATA_OFFSET+ 3].val >> 8 & 0xf);

                time_t ds3231_time_unix_timestamp = ts_to_tm(ds3231_time, nullptr);
                
                // Save a timestamp for the first sample of the next batch.
                first_sample_timestamp = ds3231_time_unix_timestamp;

                // If a new day has started, start a new data file for the next 
                // cycle.
                if (day_of_month != ds3231_time.mday) {
                    // Generate a new file name with the current date and time.
                    snprintf(output_file_name, FILE_NAME_SIZE, FILE_NAME_FORMAT, device_name, ds3231_time.year % 10000, ds3231_time.mon % 100, ds3231_time.mday % 100, ds3231_time.hour % 100, ds3231_time.min % 100);
                    // Start a new file.
                    f = fopen(output_file_name, "w");
                    if (f == nullptr) {
                        TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, true);
                    }
                    if (write_data_header(f) == 0) {
                        TRANSDUCER_ERROR(TRANSDUCER_SD_FILE_ERROR, true);
                    } 
                    fclose(f);
                    f = nullptr;
                    day_of_month = ds3231_time.mday;
                } 
            }

            ESP_LOGI(TAG, "Deinitializing SD card");
            sd_deinit();
            #ifdef DIY4
                ESP_LOGI(TAG, "Turning off SD power in %d ms", SD_OFF_DELAY_MS);
                vTaskDelay(pdMS_TO_TICKS(SD_OFF_DELAY_MS));
                rtc_gpio_set_level(SD_SWITCH_PIN, 1); // Turn off SD card power
            #endif

            // If the shutdown button was pushed while the device was in deep 
            // sleep, stop sampling and shut down.
            if (button_pressed) {
                shutdown();
            }
            
            // Allow the ULP to trigger the ESP32 to wake up.
            hulp_peripherals_on();
            ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
            ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(((uint64_t) 0b1) << BUTTON_PIN, ESP_EXT1_WAKEUP_ALL_LOW));

            ESP_LOGI(TAG, "Going to sleep...");
            esp_deep_sleep_start();
        }

        default: 
            ESP_LOGI(TAG, "Wakeup reason: %d\n", esp_reset_reason());
            TRANSDUCER_ERROR(TRANSDUCER_RESET_ERROR, true);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG_DASHBOARD, "station " MACSTR " join, AID=%d",
        MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG_DASHBOARD, "station " MACSTR " leave, AID=%d, reason=%d",
        MAC2STR(event->mac), event->aid, event->reason);
    }
}

/* Initialize soft AP */
void wifi_init_softap()
{
    // esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            nullptr,
           nullptr 
        )
    );

    wifi_config_t wifi_ap_config = {.ap{}};
    wifi_ap_config.ap.ssid_len = 0;
    wifi_ap_config.ap.channel = 1;
    wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_ap_config.ap.max_connection = 2;
    wifi_ap_config.ap.pmf_cfg.required =true;

    strcpy((char*) wifi_ap_config.ap.ssid, device_name);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished");
}


static esp_err_t http_get_index_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: GET %s", req->uri);
    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, index_html_len);

    return ESP_OK;
}


static esp_err_t http_get_api_clock_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: GET %s", req->uri);

    ts time_now;
    DS3231_get(&time_now);

    char json[32] = {0};
    int json_len = snprintf(json, sizeof(json), "{\"clock_time\": %ld}\n", time_now.unixtime);

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, json_len);

    return ESP_OK;
}


static esp_err_t http_get_api_sensor_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: GET %s", req->uri);

    sensor.readSensor();

    char json[64] = {0};
    int json_len = snprintf(
        json, 
        sizeof(json), 
        "{\"sensor_pressure\": %.1f, \"sensor_temperature\": %.1f}",
        sensor.pressure(), sensor.temperature()
    );

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, json_len);

    return ESP_OK;
}


static esp_err_t http_get_api_sample_frequency_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: GET %s", req->uri);

    char json[64] = {0};
    int json_len = snprintf(json, sizeof(json), "{\"sample_frequency\": %d}", sample_frequency);

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, json_len);

    return ESP_OK;
}


static esp_err_t http_get_api_all_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: GET %s", req->uri);

    ts time_now;
    DS3231_get(&time_now);
    sensor.readSensor();

    char json[256] = {0};
    int json_len = snprintf(
        json, 
        sizeof(json), 
        "{" 
            "\"device_name\": \"%s\","
            "\"sample_frequency\": %d,"
            "\"storage_capacity\": %llu,"
            "\"storage_used\": %lu,"
            "\"clock_time\": %ld,"
            "\"sensor_pressure\": %.1f," 
            "\"sensor_temperature\": %.1f"
        "}",
        device_name,
        sample_frequency,
        (uint64_t) card->csd.capacity * card->csd.sector_size, 
        sd_get_used(),
        time_now.unixtime,
        sensor.pressure(), 
        sensor.temperature()
    );

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, json_len);

    return ESP_OK;
}


static esp_err_t http_post_api_sample_frequency_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: POST %s", req->uri);

    char content[2]; // Should be a single digit number
    if (req->content_len > sizeof(content)) {
        ESP_LOGI(TAG_DASHBOARD, "Content size is too big: expected max of %d. Replying with 413", sizeof(content));
        httpd_resp_send_custom_err(req, "413", "Content Too Large");
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        // Check if timeout occurred
        ESP_LOGE(TAG_DASHBOARD, "Failed to receive content");
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    unsigned long new_sample_frequency = strtoul(content, nullptr, 10);

    if (!validate_sample_frequency(new_sample_frequency, TAG_DASHBOARD)) {
        httpd_resp_send_custom_err(req, "422", "Unprocessable Content");
        return ESP_FAIL;
    }

    uint16_t old_sample_frequency = sample_frequency;
    sample_frequency = new_sample_frequency;
    if (!write_config()) {
        httpd_resp_send_500(req);
        sample_frequency = old_sample_frequency;
        return ESP_FAIL;
    }

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "", 0);

    return ESP_OK;
}


static esp_err_t http_post_api_clock_handler(httpd_req_t* req) {
    ESP_LOGI(TAG_DASHBOARD, "Received: POST %s", req->uri);

    char content[16];
    if (req->content_len > sizeof(content)) {
        ESP_LOGI(TAG_DASHBOARD, "Content size is too big: expected max of %d. Replying with 413", sizeof(content));
        httpd_resp_send_custom_err(req, "413", "Content Too Large");
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        // Check if timeout occurred
        ESP_LOGE(TAG_DASHBOARD, "Failed to receive content");
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    uintmax_t epoch_time = strtoumax(content, nullptr, 10);

    if (epoch_time == 0) {
        ESP_LOGI(TAG_DASHBOARD, "The received time string was parsed as 0. This is likely an error and time will not be set");
        httpd_resp_send_custom_err(req, "422", "Unprocessable Content");
        return ESP_FAIL;
    } else if (epoch_time > std::numeric_limits<time_t>::max()) {
        ESP_LOGI(TAG_DASHBOARD, "Time given is too large (%lld). Time will not be set", epoch_time);
        httpd_resp_send_custom_err(req, "422", "Unprocessable Content");
        return ESP_FAIL;
    }

    time_t converted_epoch_time = static_cast<time_t>(epoch_time);
    tm *time = localtime(&converted_epoch_time);

    ts rtc_time = {
        .sec = static_cast<uint8_t>(time->tm_sec),
        .min = static_cast<uint8_t>(time->tm_min),
        .hour = static_cast<uint8_t>(time->tm_hour),
        .mday = static_cast<uint8_t>(time->tm_mday),
        .mon = static_cast<uint8_t>(time->tm_mon + 1),      // tm uses 0 based month while ts is 1 based
        .year = static_cast<int16_t>(time->tm_year + 1900), // tm uses years since 1900 while ts requires the actual year
        .wday = static_cast<uint8_t>(time->tm_wday),
        .yday = static_cast<uint8_t>(time->tm_yday),
        .isdst{},
        .year_s{},
        .unixtime{},
    };
    DS3231_set(rtc_time);
    ESP_LOGI(TAG_DASHBOARD, "Clock updated");

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "", 0);

    return ESP_OK;
}


static const httpd_uri_t http_get_index = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_get_index_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_get_api_clock = {
    .uri = "/api/clock",
    .method = HTTP_GET,
    .handler = http_get_api_clock_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_get_api_sensor = {
    .uri = "/api/sensor",
    .method = HTTP_GET,
    .handler = http_get_api_sensor_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_get_api_sample_frequency = {
    .uri = "/api/sample_frequency",
    .method = HTTP_GET,
    .handler = http_get_api_sample_frequency_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_get_api_all = {
    .uri = "/api/all",
    .method = HTTP_GET,
    .handler = http_get_api_all_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_post_api_clock = {
    .uri = "/api/clock",
    .method = HTTP_POST,
    .handler = http_post_api_clock_handler,
    .user_ctx = nullptr,
};

static const httpd_uri_t http_post_api_sample_frequency = {
    .uri = "/api/sample_frequency",
    .method = HTTP_POST,
    .handler = http_post_api_sample_frequency_handler,
    .user_ctx = nullptr,
};


httpd_handle_t start_server() {
    httpd_handle_t server = nullptr;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();    

    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &http_get_index);
        httpd_register_uri_handler(server, &http_get_api_clock);
        httpd_register_uri_handler(server, &http_get_api_sensor);
        httpd_register_uri_handler(server, &http_get_api_all);
        httpd_register_uri_handler(server, &http_post_api_clock);
        httpd_register_uri_handler(server, &http_get_api_sample_frequency);
        httpd_register_uri_handler(server, &http_post_api_sample_frequency);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return nullptr;
}


void start_dashboard() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize AP */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
    
    start_server();

    bool led_on = false;
    // TODO: DNS server
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SERVER_MODE_LED_FLASH_PERIOD_MS));

        if (led_on) {
            rtc_gpio_set_level(ERROR_LED_PIN, 0);
        } else {
            rtc_gpio_set_level(ERROR_LED_PIN, 1);
        }
        led_on = !led_on;
    }
}