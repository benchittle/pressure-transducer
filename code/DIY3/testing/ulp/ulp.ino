#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"


#define LED_PIN GPIO_NUM_15

#define ULP_SDA GPIO_NUM_0
#define ULP_SCL GPIO_NUM_4





#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13

#define SLAVE_ADDR DS3231_I2C_ADDR

// Set for 8-bit read:
 #define SLAVE_READ8_SUBADDR DS3231_CONTROL_ADDR

// Set for 16-bit read:
// #define SLAVE_READ16_SUBADDR 0x0

// Set subaddress and value for write:
// #define SLAVE_WRITE_SUBADDR 0x0
// #define SLAVE_WRITE_VALUE 0x0

RTC_DATA_ATTR ulp_var_t ulp_data8;
RTC_DATA_ATTR ulp_var_t ulp_data16;
RTC_DATA_ATTR ulp_var_t ulp_nacks;
RTC_DATA_ATTR ulp_var_t ulp_buserrors;

uint8_t creg = 0;

void init_ulp()
{
    enum {
        LBL_READ8_RETURN,
        LBL_READ16_RETURN,
        LBL_WRITE_RETURN,

        LBL_HALT,
        
        LBL_I2C_READ_ENTRY,
        LBL_I2C_WRITE_ENTRY,
        LBL_I2C_NACK,
        LBL_I2C_ARBLOST,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

    #ifdef SLAVE_READ8_SUBADDR
        M_I2CBB_RD(LBL_READ8_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR),
        I_PUT(R0, R2, ulp_data8),
        I_WAKE(),
    #endif

    #ifdef SLAVE_READ16_SUBADDR
        M_I2CBB_RD(LBL_READ16_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ16_SUBADDR),
        I_PUT(R0, R2, ulp_data16),
        I_WAKE(),
    #endif

    #ifdef SLAVE_WRITE_SUBADDR
        M_I2CBB_WR(LBL_WRITE_RETURN, LBL_I2C_WRITE_ENTRY, SLAVE_WRITE_SUBADDR, SLAVE_WRITE_VALUE),
    #endif

        I_HALT(),

        M_LABEL(LBL_I2C_NACK),
            I_GET(R0, R2, ulp_nacks),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_nacks),
            I_WAKE(),
            I_BXR(R3),

        M_LABEL(LBL_I2C_ARBLOST),
            I_GET(R0, R2, ulp_buserrors),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_buserrors),
            I_WAKE(),
            I_BXR(R3),

        M_INCLUDE_I2CBB(LBL_I2C_READ_ENTRY, LBL_I2C_WRITE_ENTRY, LBL_I2C_ARBLOST, LBL_I2C_NACK, SCL_PIN, SDA_PIN, SLAVE_ADDR),
    };

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void setup() {
    Serial.begin(115200);
    Serial.println("START");

    Wire.begin();
    DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    creg = DS3231_get_creg();

    init_ulp();
}

void loop() {
    // Wait for interrupt
    printf("Read8: %u, Read16: %u, NACK Errors: %u, Bus Errors: %u\n", ulp_data8.val, ulp_data16.val, ulp_nacks.val, ulp_buserrors.val);
    printf("REG: %d\n", creg);
    delay(1000);
}