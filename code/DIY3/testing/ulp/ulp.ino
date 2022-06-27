#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"
#include "MS5803_05.h"

#define RTC_ALARM GPIO_NUM_25


// ESP32 I2C monitoring connections
#define ESP_SDA GPIO_NUM_21
#define ESP_SCL GPIO_NUM_22


// ULP I2C connections
#define SCL_PIN GPIO_NUM_14 
#define SDA_PIN GPIO_NUM_13

RTC_DATA_ATTR ulp_var_t ulp_buffer[50];
RTC_DATA_ATTR ulp_var_t ulp_buf_offset;
RTC_DATA_ATTR ulp_var_t ulp_D2_flag;

RTC_DATA_ATTR ulp_var_t ulp_var1;
RTC_DATA_ATTR ulp_var_t ulp_var2;

RTC_DATA_ATTR ulp_var_t ulp_i2c_read_sensor[HULP_I2C_CMD_BUF_SIZE(3)] = {
    //HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 2)
    HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 3)
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
    //HULP_I2C_CMD_1B(0b10100000)
};

// addr: 1101000(0|1)
// ctrl: 00001110
// 92 = 01011100 


uint8_t sdaVals[256];
uint8_t count = 0;

uint32_t varD1, varD2;

// addr: 0x77 = 0b1110111
/*
C0 = 0
C1 = 26325
C2 = 22307
C3 = 32375
C4 = 14479
C5 = 32716
C6 = 28642
C7 = 6
*/
MS_5803 sensor(4096);

void init_ulp()
{
    enum {
        L_LOOP_FOR_D2,

        L_READ,
        L_WRITE,
        L_W_RETURN0,
        L_W_RETURN1,
        L_R_RETURN,
    };


    const ulp_insn_t program[] = {
        
        // Check to see if the RTC alarm was triggered. If so, continue. 
        // Otherwise, halt and put the ULP back to sleep until the next cycle.
        I_GPIO_INT_RD(RTC_ALARM),
        I_BGE(2, 1), // Skip halt if triggered
        I_HALT(),
        
        // Clear the interrupt that was triggered
        I_GPIO_INT_CLR(RTC_ALARM),

        // Clear D2 read flag
        I_MOVI(R2, 0),
        I_MOVI(R0, 0),
        I_PUT(R0, R2, ulp_D2_flag),
        
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
        I_BGE(5, 1),

        // Set D2 flag before branching to get D2.
        I_MOVI(R0, 1),
        I_PUT(R0, R2, ulp_D2_flag),
        I_MOVO(R1, ulp_i2c_write_convertD2),
        M_BX(L_LOOP_FOR_D2),

        // Branch here if we've already read D2.
        I_END(),
        I_HALT(),

        M_INCLUDE_I2CBB_CMD(L_READ, L_WRITE, SCL_PIN, SDA_PIN),

    };
    
    //ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));
    //ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));   
    ESP_ERROR_CHECK(hulp_configure_pin_int(RTC_ALARM, GPIO_INTR_LOW_LEVEL));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("START");

    ulp_var1.val = 0;
    ulp_buf_offset.val = 0;
    ulp_D2_flag.val = 0;

    hulp_peripherals_on();

    pinMode(ESP_SDA, INPUT);
    pinMode(ESP_SCL, INPUT);
    
    sensor.initializeMS_5803(true);
    sensor.readSensor();
    printf("D1: %x \tD2: %x\n", sensor.D1val(), sensor.D2val());
    printf("P: %f \tT: %f\n", sensor.pressure(), sensor.temperature());

    init_ulp();
    
    
    while (!digitalRead(ESP_SCL) || !digitalRead(ESP_SDA));
    
    monitor();
    for (int i = 0; i < count && i < sizeof(sdaVals); i++) {
        //printf("(%d) SDA: %d\n", i, sdaVals[i]);
    }

    delay(1000);
    printf("Offset: %d\n", ulp_buf_offset.val);
        for (int i = 0; i < 4; i++) {
        printf("D%d: %x\n", i / 2 + 1, ulp_buffer[i].val);
    }
    
}


// 24 bit Value is stored between two variable: read[offset + 0] = upper and mid byte, 
//                                              read[offset + 1] >> 8 = lower byte
void loop() {
    
    printf("buf: %x \tcount: %d\n\n", ulp_buffer[ulp_buf_offset.val / 4].val, ulp_buf_offset.val);
    delay(1000);   
    
}



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
            */
        }
        t1 = millis();
        while(!digitalRead(ESP_SCL)) {
            if (millis() - t1 > 4000) {
                return;
            }
        }
    }
}