#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"
#include "MS5803_05.h"


// ESP32 I2C monitoring connections
#define ESP_SDA GPIO_NUM_21
#define ESP_SCL GPIO_NUM_22


// first send CMD_ADC_CONV + CMD_ADC_Dx + CMD_ADC_4096
// then wait 10ms
// then 


// ULP I2C connections
#define SCL_PIN GPIO_NUM_14 
#define SDA_PIN GPIO_NUM_13

#define LED_PIN SDA_PIN


#define M_I2C_DATA_HI() I_GPIO_OUTPUT_DIS(SDA_PIN)
#define M_I2C_DATA_LO() \
    I_GPIO_OUTPUT_EN(SDA_PIN), \
    I_GPIO_SET(SDA_PIN, 0)

#define M_I2C_CLOCK_HI() I_GPIO_OUTPUT_DIS(SCL_PIN)
#define M_I2C_CLOCK_LO() \
    I_GPIO_OUTPUT_EN(SCL_PIN), \
    I_GPIO_SET(SCL_PIN, 0)


#define M_I2C_DELAY() M_DELAY_US_100_5000(5000)

#define SLAVE_ADDR MS5803_I2C_ADDRESS

// Set for 8-bit read:
#define SLAVE_READ8_SUBADDR (CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096)

// Set for 16-bit read:
// #define SLAVE_READ16_SUBADDR 0x0

// Set subaddress and value for write:
 //#define SLAVE_WRITE_SUBADDR CMD_RESET
 //#define SLAVE_WRITE_VALUE CMD_RESET

RTC_DATA_ATTR ulp_var_t ulp_var1;
RTC_DATA_ATTR ulp_var_t ulp_var2;

RTC_DATA_ATTR ulp_var_t ulp_read_cmd[HULP_I2C_CMD_BUF_SIZE(2)] = {
    HULP_I2C_CMD_HDR_NO_PTR(MS5803_I2C_ADDRESS, 2)
};

RTC_DATA_ATTR ulp_var_t ulp_write_cmd[] = {
    HULP_I2C_CMD_HDR(MS5803_I2C_ADDRESS, 0b10100100, 0),
    //HULP_I2C_CMD_1B(0b10100000)
};

// addr: 1101000(0|1)
// ctrl: 00001110
// 92 = 01011100 


uint8_t sdaVals[256];
uint8_t count = 0;



uint8_t creg = 0;
uint8_t sdaVal, sclVal;


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
        L_READ,
        L_WRITE,
        L_W_RETURN,
        L_R_RETURN,
    };


    const ulp_insn_t program[] = {
        M_I2C_DELAY(),

        
        I_MOVO(R1, ulp_write_cmd),
        M_MOVL(R3, L_W_RETURN),
        M_BX(L_WRITE),
        M_LABEL(L_W_RETURN),
        


        I_MOVO(R1, ulp_read_cmd),
        M_MOVL(R3, L_R_RETURN),
        M_BX(L_READ),
        M_LABEL(L_R_RETURN),


        I_END(),
        I_HALT(),

        M_INCLUDE_I2CBB_CMD(L_READ, L_WRITE, SCL_PIN, SDA_PIN),

    };
    
    //ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));
    //ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));   

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("START");

    ulp_var1.val = 0;

    //pinMode(LED_PIN, OUTPUT);
    

    //hulp_configure_pin(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1);
    hulp_peripherals_on();

    pinMode(ESP_SDA, INPUT);
    pinMode(ESP_SCL, INPUT);
    
    
    //Wire.begin();
    //DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    //creg = DS3231_get_creg();
    //printf("start var1: %d\nstart creg: %x\n\n", ulp_var1.reg_off, creg);

    sensor.initializeMS_5803(true);

    

    init_ulp();
    //delay(1500);

    
    while (!digitalRead(ESP_SCL) || !digitalRead(ESP_SDA));
    //delay(250);
    monitor();
    for (int i = 0; i < count && i < sizeof(sdaVals); i++) {
        printf("(%d) SDA: %d\n", i, sdaVals[i]);
    }
}



void loop() {
    
    printf("c0: %d \tc1: %d\n\n", ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET].val, ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 1].val);
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
            if (sdaVal && !digitalRead(ESP_SDA) && !flag) {
                //printf("START COND\n\n");
                sdaVals[count] = 2;
                count++;
                flag = 1;
            }
        }
        t1 = millis();
        while(!digitalRead(ESP_SCL)) {
            if (millis() - t1 > 4000) {
                return;
            }
        }
    }
}