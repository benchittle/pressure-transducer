#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"
#include "MS5803_05.h"

#define POW_2_33 8589934592ULL;

#define RTC_ALARM GPIO_NUM_25


// ESP32 I2C monitoring connections
#define ESP_SDA GPIO_NUM_21
#define ESP_SCL GPIO_NUM_22


// ULP I2C connections
#define SCL_PIN GPIO_NUM_0
#define SDA_PIN GPIO_NUM_14

// The number of samples to take before buffer needs to be emptied
#define ULP_BUFFER_SAMPLES 100
#define ULP_SAMPLE_SIZE 4
#define ULP_BUFFER_SIZE (ULP_BUFFER_SAMPLES * ULP_SAMPLE_SIZE)

RTC_DATA_ATTR ulp_var_t ulp_buffer[ULP_BUFFER_SIZE];
RTC_DATA_ATTR ulp_var_t ulp_buf_offset;
RTC_DATA_ATTR ulp_var_t ulp_D2_flag;

RTC_DATA_ATTR ulp_var_t ulp_full_flag;

RTC_DATA_ATTR ulp_var_t ulp_i2c_read_sensor[HULP_I2C_CMD_BUF_SIZE(3)] = {
    //HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 2)
    HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 3)
};

RTC_DATA_ATTR ulp_var_t ulp_i2c_write_clearAlarm[] = {
    HULP_I2C_CMD_HDR(DS3231_I2C_ADDR, DS3231_STATUS_ADDR, 1),
    HULP_I2C_CMD_1B(0x0)
};

// TODO: Why does NO_PTR cause the command to become a read?
RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convertD1[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x48, 0),
};

RTC_DATA_ATTR ulp_var_t ulp_i2c_write_convertD2[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x58, 0),
};

RTC_DATA_ATTR ulp_var_t ulp_i2c_write_readADC[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0x0, 0),
};

typedef struct {
    //uint32_t timestamp;
    float pressure;
    int8_t temperature;
} __attribute__((packed)) entry_t;

RTC_DATA_ATTR MS_5803 sensor(4096);

void init_ulp()
{
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


    const ulp_insn_t program[] = {

        
        // Check to see if the RTC alarm was triggered. If so, continue. 
        // Otherwise, halt and put the ULP back to sleep until the next cycle.
        I_GPIO_READ(RTC_ALARM),
        M_BGE(L_DONE, 1),
        
        // Clear the interrupt that was triggered on DS3231
        I_MOVO(R1, ulp_i2c_write_clearAlarm),
        M_MOVL(R3, L_W_RETURN2),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN2),

        // Clear D2 read flag
        I_MOVI(R0, 0),
        I_PUT(R0, R0, ulp_D2_flag),
        
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

        // Store value in buffer
        I_MOVI(R2, 0),
        I_GET(R3, R2, ulp_buf_offset),
        I_GET(R0, R2, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET]),
        I_GET(R1, R2, ulp_i2c_read_sensor[HULP_I2C_CMD_DATA_OFFSET + 1]),
        I_PUTO(R0, R3, 0, ulp_buffer),
        I_PUTO(R1, R3, -1, ulp_buffer),
        I_ADDI(R3, R3, 2),
        I_PUT(R3, R2, ulp_buf_offset),

        // Loop back and repeat for D2. If D2 was just read, jump past next block.
        I_GET(R0, R2, ulp_D2_flag),
        I_BGE(4, 1),

        // Set D2 flag before branching to get D2. (R3 is used to save an 
        // instruction, since we know R3 > 0, we don't need to MOVI(R3, 1) 
        // first).
        I_PUT(R3, R2, ulp_D2_flag),
        I_MOVO(R1, ulp_i2c_write_convertD2),
        M_BX(L_LOOP_FOR_D2),

        // Branch here if we've already read D2.

        // Check if the buffer is full and respond accordingly (i.e. wake up
        // processor and write to flash).
        I_MOVR(R0, R3),
        M_BL(L_DONE, ULP_BUFFER_SIZE),
        I_WAKE(),
        //I_PUT(R3, R2, ulp_full_flag),
        I_END(),

        M_LABEL(L_DONE),
        I_HALT(),

        M_INCLUDE_I2CBB_CMD(L_READ, L_WRITE, SCL_PIN, SDA_PIN),
    };
    
    //ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));
    //ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));   
    ESP_ERROR_CHECK(hulp_configure_pin(RTC_ALARM, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("START");
    
    pinMode(ESP_SDA, INPUT);
    pinMode(ESP_SCL, INPUT);

    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: {
            ulp_full_flag.val = 0;
            ulp_buf_offset.val = 0;
            ulp_D2_flag.val = 0;

            hulp_peripherals_on();

            Wire.begin();
            DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
            DS3231_clear_a1f();
            DS3231_clear_a2f();

            // Start the once-per-second alarm on the DS3231:
            // These flags set the alarm to be in once-per-second mode.
            const uint8_t flags[] = {1,1,1,1,0};
            // Set the alarm. The time value doesn't matter in once-per-second
            // mode.
            DS3231_set_a1(1, 1, 1, 1, flags);
            // Enable the alarm. It will now bring the RTC_ALARM GPIO low every
            // second.
            DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);
            
            sensor.initializeMS_5803(true);
            sensor.readSensor();
            printf("D1: %x \tD2: %x\n", sensor.D1val(), sensor.D2val());
            printf("P: %f \tT: %f\n", sensor.pressure(), sensor.temperature());

            init_ulp();

            esp_sleep_enable_ulp_wakeup();
            esp_deep_sleep_start();
            
            break; // we never reach this due to sleep
        }

        case ESP_RST_DEEPSLEEP: {
            // Make a copy of the raw data.
            ulp_var_t raw[ULP_BUFFER_SIZE];
            memcpy(raw, ulp_buffer, ULP_BUFFER_SIZE * sizeof(ulp_buffer[0]));

            // Restart the ULP as soon as we've made a copy of the data, since 
            // it can run in parallel with the main program that will process 
            // the raw data.
            ulp_buf_offset.val = 0;
            ulp_full_flag.val = 0;
            init_ulp();

            entry_t buffer[ULP_BUFFER_SAMPLES];

            for (uint16_t i = 0, j = 0; i < ULP_BUFFER_SAMPLES; i++, j += ULP_SAMPLE_SIZE) {
                uint32_t varD1 = (raw[j].val << 8) | (raw[j + 1].val >> 8);
                uint32_t varD2 = (raw[j + 2].val << 8) | (raw[j + 3].val >> 8);

                sensor.convertRaw(varD1, varD2);
                buffer[i] = {
                    .pressure = sensor.pressure(),
                    .temperature = (int8_t) sensor.temperature()
                };

            }

            printf("P  \tT\n");
            for (int i = 0; i < ULP_BUFFER_SAMPLES; i++) {
                printf("%.2f  \t%d\n", buffer[i].pressure, buffer[i].temperature);
            }

            esp_sleep_enable_ulp_wakeup();
            esp_deep_sleep_start();
        }
    }
    
    /*
    while (!digitalRead(ESP_SCL) || !digitalRead(ESP_SDA));
    
    monitor();
    for (int i = 0; i < count && i < sizeof(sdaVals); i++) {
        //printf("(%d) SDA: %d\n", i, sdaVals[i]);
    }

    delay(1000);
    printf("Offset: %d\n", ulp_buf_offset.val);
        for (int i = 0; i < 4; i++) {
        printf("D%d: %x\n", i / 2 + 1, ulp_buffer[i].val);
    }*/
    
}


// 24 bit Value is stored between two variable: read[offset + 0] = upper and mid byte, 
//                                              read[offset + 1] >> 8 = lower byte
// delayed 30ms
/*
int offset = 0;
time_t t;
ulp_var_t raw_buffer[ULP_BUFFER_SIZE];
*/
void loop() {} /*
    if (ulp_buf_offset.val - offset >= 4) {
        t = millis();
        printf("t: %d \tbuf: %x \toffset: %d\n", millis(), ulp_buffer[ulp_buf_offset.val / 4].val, ulp_buf_offset.val);   
        offset = ulp_buf_offset.val;
    }
    if (ulp_full_flag.val != 0) {
        memcpy(raw_buffer, ulp_buffer, ULP_BUFFER_SIZE * sizeof(ulp_buffer[0]));
        init_ulp();
        //printf("Write to flash!\tt: %d\n", millis() - t);
        ulp_buf_offset.val = 0;
        ulp_full_flag.val = 0;
        offset = 0;
    }
    //delay(10);
}
*/

/*
void monitor() {
    while (1) {
        uint8_t flag = 0;
        //sdaVal = digitalRead(ESP_SDA);
        //sclVal = digitalRead(ESP_SCL);
        
        //printf("SDA: %d \tSCL: %d \tulp: %x\n", sdaVal, sclVal, ulp_var1.val & 0xFF);
        sdaVals[count] = digitalRead(ESP_SDA);
        count++;
        time_t t1 = millis();
        while(digitalRead(ESP_SCL)) {
            
            if (millis() - t1 > 4000) {
                return;
            }
            /*
            if (sdaVal && !digitalRead(ESP_SDA) && !flag) {
                //printf("START COND\n\n");
                sdaVals[count] = 2;
                count++;
                flag = 1;
            }
            * /
        }
        t1 = millis();
        while(!digitalRead(ESP_SCL)) {
            if (millis() - t1 > 4000) {
                return;
            }
        }
    }
}
*/