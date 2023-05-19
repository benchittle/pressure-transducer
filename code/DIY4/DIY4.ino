#include <FS.h>
#include <SD.h>
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


#define ECHO_TO_SERIAL 0

// Some functions require gpio_num_t types so we use those pin number 
// definitions here.
#define SD_CS_PIN GPIO_NUM_13 // D7 
#define SD_SWITCH_PIN GPIO_NUM_15 // A4
#define RTC_POWER_PIN GPIO_NUM_26 // D3
#define RTC_ALARM_PIN GPIO_NUM_25 //D2
#define ULP_SCL_PIN GPIO_NUM_0 // D5
#define ULP_SDA_PIN GPIO_NUM_14 // D6
#define ERROR_LED_PIN GPIO_NUM_2

// Duration in ms for each flash of the LED when displaying an error / warning.
#define LED_FLASH_DURATION 500

// /NAME_YYYYMMDD-hhmm.data is the format.
#define FILE_NAME_FORMAT "/%7s_%04d%02d%02d-%02d%02d.data"
// The format specifiers take up more room than the formatted string will, so we 
// subtract the extra space from the size. Then add room for DIY3-XX part.
#define FILE_NAME_SIZE (sizeof(FILE_NAME_FORMAT) - 11 + 7)

// Number of readings (pressure and temperature) to store in a buffer before we 
// dump to the SD card.
#define BUFFER_SIZE 120
// Number of ulp_var_t's needed for one raw pressure and temperature reading.
// (24 bits for raw pressure, 24 bits for raw temperature; could be packed 
// in only 3 spaces with more ULP processing).
#define ULP_SAMPLE_SIZE 4
// Size of the buffer that the ULP will use to store raw data from the MS5803.
#define ULP_BUFFER_SIZE (BUFFER_SIZE * ULP_SAMPLE_SIZE)

// While the ULP is active, it will repeatedly run at this interval 
// (microseconds).
#define ULP_RUN_PERIOD 1000


// A custom struct to store a single data entry.
// NOTE: __attribute__((packed)) tells the compiler not to add additional 
// padding bytes that would align to the nearest 4 bytes. This can cause issues
// in some cases, but the ESP32 doesn't seem to have a problem with it. By doing
// this, we reduce the size of each entry from 12 bytes to 9 bytes.

struct entry_t {
    float pressure;
    int8_t temperature;
} __attribute__((packed));

// Initialize a sensor object for interacting with the MS5803-05
RTC_DATA_ATTR MS_5803 sensor(4096); // MAYBE CAN LOWER OVERSAMPLING

// Store the device's name (max 7 characters).
RTC_DATA_ATTR char deviceName[8] = {0};
// Store the name for the current data file across deep sleep restarts.
RTC_DATA_ATTR char fileName[FILE_NAME_SIZE];

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


// Error codes for the program. The value associated with each enum is also the
// number of times the error LED will flash if an error is encountered.
enum diy3_error_t {
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
void error(diy3_error_t flashes) {
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


/*
 * Flash the error LED to warn the user.
 */
void warning(diy3_error_t flashes) {
    for (uint8_t i = 0; i < flashes; i++) {
        digitalWrite(ERROR_LED_PIN, HIGH);
        delay(LED_FLASH_DURATION);
        digitalWrite(ERROR_LED_PIN, LOW);
        delay(LED_FLASH_DURATION);
    }
    delay(500);
}


/* 
 * Define the ULP program, configure the ULP appropriately, and upload the 
 * program to the ULP. 
 */ 
void init_ulp()
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


void setup() {
    #if ECHO_TO_SERIAL
        Serial.begin(115200);
    #else
        // ADC, WiFi, BlueTooth are disabled by default.        
        // Default CPU frequency is 240MHz, but we don't need high speed. Note
        // that the serial monitor baud rate changes with lower frequencies, so
        // it is not changed when debugging.
        setCpuFrequencyMhz(10);     
    #endif

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power
    pinMode(SD_SWITCH_PIN, OUTPUT);
    pinMode(RTC_POWER_PIN, OUTPUT);
    pinMode(RTC_ALARM_PIN, INPUT_PULLUP);
    pinMode(ERROR_LED_PIN, OUTPUT);

    // Jump to the appropriate code depending on whether the ESP32 was just 
    // powered or just woke up from deep sleep. 
    switch (esp_reset_reason()) {   
        case ESP_RST_POWERON: {
            #if ECHO_TO_SERIAL
                delay(500); // Wait for serial to be ready
                Serial.printf(
                    "BUFFER_SIZE=%d\n"
                    "ULP_RUN_PERIOD=%d\n",
                    BUFFER_SIZE, ULP_RUN_PERIOD
                );
                Serial.flush();
            #endif


            ulpBufOffset.val = 0;
            ulpD2Flag.val = 0;

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

            #if ECHO_TO_SERIAL
                printf("RTC Time: %d-%02d-%02d %02d:%02d:%02d\n", timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                Serial.flush();
            #endif

            // TODO: Check Status register as well, as it indicates if time was 
            // set since powered on (See #1)
            if (timeNow.year < 2023) {
                #if ECHO_TO_SERIAL
                    Serial.println("WARNING: Time is out of date. Setting year to 2000 for UNIX time compatibility.");
                    Serial.flush();
                #endif
                timeNow.year = 2000;
                DS3231_set(timeNow);
            }

            // Start the once-per-second alarm on the DS3231:
            // These flags set the alarm to be in once-per-second mode.
            const uint8_t flags[] = {1,1,1,1,0};
            // Set the alarm. The time value doesn't matter in once-per-second
            // mode.
            DS3231_set_a1(1, 1, 1, 1, flags);
            // Enable the alarm. It will now bring the RTC_ALARM_PIN GPIO low every
            // second.
            DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);


            // Initialize the connection with the SD card.
            digitalWrite(SD_SWITCH_PIN, LOW);
            if (!SD.begin(SD_CS_PIN)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("SD setup error"));
                    Serial.flush();
                #endif
                error(sdInitError);
            }

            
            File config = SD.open("/config.txt", FILE_READ, false);
            /*
            uint16_t startYear = 0;
            uint8_t startMonth, startDay, startHour, startMinute;
            */
            // Get device's name from SD card or use default. Also get the start
            // time, if present.
            if (!config) {
                strcpy(deviceName, "DIY3-XX");
                warning(sdConfigWarning);
            } else {
                config.read((uint8_t*) deviceName, sizeof(deviceName) - 1);
                /*
                // Try to read a start time from the config file next.
                // Format: YYYY MM DD hh mm
                startYear = config.parseInt();
                if (startYear) {
                    startDay = config.parseInt();
                    startHour = config.parseInt();
                    startMinute = config.parseInt();
                }
                */
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
                error(sdFileError);
            }
            f.close();
            SD.end();


            // Initialize the connection with the MS5803-05 pressure sensor.
            if (!sensor.initializeMS_5803(false)) {
                #if ECHO_TO_SERIAL
                    Serial.println(F("MS5803-05 sensor setup error"));
                    Serial.flush();
                #endif
                error(ms5803Error);
            }

            #if ECHO_TO_SERIAL
                Serial.print("Taking a sample sensor reading... ");
                Serial.flush();

                sensor.readSensor();

                Serial.printf("Pressure: %f \tTemperature: %f\n", sensor.pressure(), sensor.temperature());
                Serial.flush();
            #endif

            // Flash LED's to signal successful startup.
            digitalWrite(ERROR_LED_PIN, HIGH);
            delay(3000);
            digitalWrite(ERROR_LED_PIN, LOW);
            
            #if ECHO_TO_SERIAL
                Serial.println("Turning off SD power");
                Serial.flush();
            #endif
            digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power

            DS3231_get(&timeNow);
            // Save a timestamp for the first sample. We'll add 1 second
            // (alarmStatus) to the time since the first sample will actually be
            // taken during the next second.
            firstSampleTimestamp = timeNow.unixtime + 1;

            #if ECHO_TO_SERIAL
                Serial.printf("Time is %d-%02d-%02d %02d:%02d:%02d. ", timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                Serial.print("Waiting for next second... ");
            #endif
            // Wait the rest of this second to start.
            while (timeNow.unixtime != firstSampleTimestamp) {
                DS3231_get(&timeNow);
            }
            #if ECHO_TO_SERIAL
                Serial.print("Done\nDisabling DS3231 pin power and starting ULP...");
                Serial.flush();
            #endif

            // Disable power to the DS3231's VCC.
            digitalWrite(RTC_POWER_PIN, LOW);

            // Start the ULP program.
            init_ulp();

            #if ECHO_TO_SERIAL
                Serial.println("Done\nSetup complete. Going to sleep...");
                Serial.flush();
            #endif

            // Allow the ULP to trigger the ESP32 to wake up.
            esp_sleep_enable_ulp_wakeup();
            // Enter deep sleep.
            esp_deep_sleep_start();

            break; // switch (we shouldn't actually reach this because of sleep)
        }

        // TODO: Check if DS3231 backup battery failed and switch to main power
        case ESP_RST_DEEPSLEEP: {
            #if ECHO_TO_SERIAL    
                Serial.println("Awake!");
                Serial.flush();
            #endif

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
            digitalWrite(RTC_POWER_PIN, LOW);
            Wire.end();


            // Make a copy of the raw data in the ULP's buffer.
            ulp_var_t raw[ULP_BUFFER_SIZE];
            memcpy(raw, ulpBuffer, ULP_BUFFER_SIZE * sizeof(ulpBuffer[0]));
            // Restart the ULP as soon as we've made a copy of the data, since 
            // it can run in parallel with the main program as we process the
            // raw data.
            ulpBufOffset.val = 0;
            init_ulp();


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
                Serial.printf("Dumping to card...\nFirst Timestamp=%d\nP\tT\n", firstSampleTimestamp);
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
            //delay(2000);
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

            // Open a file for logging the data. If it's the first dump of
            // the day, start a new file.
            // TODO: Write buffered data before starting new day
            File f = SD.open(fileName, FILE_APPEND, false);
            if (!f) {
                #if ECHO_TO_SERIAL
                    Serial.println("Failed to open file");
                    Serial.flush();
                #endif
                error(sdFileError);
            }
            // Write timestamp of first sample.
            size_t written = f.write((uint8_t*) &firstSampleTimestamp, sizeof(firstSampleTimestamp));
            // Write the data buffer as a sequence of bytes (it's vital that
            // the entry_t struct is packed, otherwise there will be garbage
            // bytes in between each entry that will waste space). In order
            // to use this data later, we'll have to unpack it using a 
            // postprocessing script.
            written += f.write((uint8_t*) writeBuffer, sizeof(writeBuffer));
            f.close();
            
            #if ECHO_TO_SERIAL
                Serial.printf("Wrote %lu bytes to file\n", written);
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
                snprintf(fileName, FILE_NAME_SIZE, FILE_NAME_FORMAT, deviceName, timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec);
                // Start a new file.
                f = SD.open(fileName, FILE_WRITE, true);
                if (!f) {
                    #if ECHO_TO_SERIAL
                        Serial.println("Failed to open file");
                        Serial.flush();
                    #endif
                    error(sdFileError);
                }
                
                f.close();
                oldDay = timeNow.mday;
            } 

            SD.end();
            #if ECHO_TO_SERIAL
                Serial.println("Turning off SD power in 1 sec");
            #endif
            delay(1000);
            digitalWrite(SD_SWITCH_PIN, HIGH); // Turn off SD card power

            /*
            #if ECHO_TO_SERIAL
                Serial.printf("Buffer: %d \tTime: %d-%02d-%02d %02d:%02d:%02d \tPressure: %f \tTemp: %d\n", bufferCount, timeNow.year, timeNow.mon, timeNow.mday, timeNow.hour, timeNow.min, timeNow.sec, sensor.pressure(), (int8_t) sensor.temperature());
                Serial.flush();
            #endif
            */

            #if ECHO_TO_SERIAL
                Serial.printf("Going to sleep...\n");
                Serial.flush();
            #endif

            // Allow the ULP to trigger the ESP32 to wake up.
            esp_sleep_enable_ulp_wakeup();
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
        Serial.println("CONTROL LOST");
    #endif
    error(resetError);
}

void loop() {}