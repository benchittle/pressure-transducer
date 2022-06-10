#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "ds3231.h"


#define LED_PIN GPIO_NUM_15

#define ULP_SDA GPIO_NUM_0
#define ULP_SCL GPIO_NUM_4


RTC_DATA_ATTR ulp_var_t ulp_creg;
RTC_DATA_ATTR ulp_var_t ulp_test;

enum ulp_labels{
    L_LED_ON, 
    L_LED_OFF
};


uint8_t creg = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("START");


    const ulp_insn_t program[] = {
        I_I2C_READ(1, DS3231_STATUS_ADDR),
        //I_I2C_READ(0, 0),
        I_MOVI(R2, 0),
        I_PUT(R0, R2, ulp_creg),
        I_MOVI(R1, 2),
        I_PUT(R1, R2, ulp_test),
        
        
        I_END(),
        I_HALT()
    };

    

    //pinMode(LED_PIN, OUTPUT);
    ulp_creg.val = 0;

    //Wire.begin();
    //DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    //creg = DS3231_get_creg();

    Serial.printf("REG: %x\n", READ_PERI_REG(SENS_SAR_SLAVE_ADDR1_REG));
    ESP_ERROR_CHECK(hulp_register_i2c_slave(1, DS3231_I2C_ADDR));
    Serial.printf("REG: %x\n", READ_PERI_REG(SENS_SAR_SLAVE_ADDR1_REG));
    ESP_ERROR_CHECK(hulp_configure_i2c_pins(ULP_SCL, ULP_SDA, false, false));

    //const hulp_i2c_controller_config_t config = HULP_I2C_CONTROLLER_CONFIG_DEFAULT();
    //ESP_ERROR_CHECK(hulp_configure_i2c_controller(&config));

    //hulp_peripherals_on();
    //hulp_configure_pin(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0);
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1000000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void loop() {
    Serial.printf("Start CREG: %x\nulp CREG: %x\nULP TEST: %x\n\n", creg, ulp_creg.val, ulp_test.val);
    delay(1000);
}