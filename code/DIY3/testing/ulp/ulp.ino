#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"
#include "MS5803_05.h"



#define LED_PIN GPIO_NUM_15

#define ULP_SDA GPIO_NUM_0
#define ULP_SCL GPIO_NUM_4


// first send CMD_ADC_CONV + CMD_ADC_Dx + CMD_ADC_4096
// then wait 10ms
// then 



#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13


// TODO: Hi should be hi impedence (input), low should be driving low
#define I_I2C_DATA_HI() I_GPIO_OUTPUT_EN(SDA_PIN)
#define I_I2C_DATA_LO() I_GPIO_OUTPUT_DIS(SDA_PIN)

#define I_I2C_CLOCK_HI() I_GPIO_OUTPUT_EN(SCL_PIN)
#define I_I2C_CLOCK_LO() I_GPIO_OUTPUT_DIS(SCL_PIN)

#define M_DELAY_1MS() M_DELAY_US_100_5000(1000)

#define SLAVE_ADDR MS5803_I2C_ADDRESS

// Set for 8-bit read:
 #define SLAVE_READ8_SUBADDR (CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096)

// Set for 16-bit read:
// #define SLAVE_READ16_SUBADDR 0x0

// Set subaddress and value for write:
 //#define SLAVE_WRITE_SUBADDR CMD_RESET
 //#define SLAVE_WRITE_VALUE CMD_RESET

RTC_DATA_ATTR ulp_var_t ulp_data8;
RTC_DATA_ATTR ulp_var_t ulp_data16;
RTC_DATA_ATTR ulp_var_t ulp_nacks;
RTC_DATA_ATTR ulp_var_t ulp_buserrors;

uint8_t creg = 0;

MS_5803 sensor(4096);

void init_ulp()
{
    enum {
        SUB_I2C_WRITE_BIT,
        SUB_I2C_READ_BIT,
        SUB_I2C_INIT,
        SUB_I2C_START,
        SUB_I2C_STOP,
        SUB_I2C_WRITE,
        SUB_I2C_READ,
    };

    // Note:
    // R0 is used to pass an argument to a subroutine
    // R2 is used to return values from subroutines
    // R3 is used to store the return address for a subroutine
    const ulp_insn_t program[] = {
        
        M_LABEL(SUB_I2C_WRITE_BIT),
            I_BL(3, 1),     // skip next two instructions if R0 == 0
            I_I2C_DATA_LO(),
            I_BGE(2, 0),    // skip next instruction otherwise
            I_I2C_DATA_HI(),

            I_I2C_CLOCK_HI(),
            M_DELAY_1MS(),

            I_I2C_CLOCK_LO(),
            M_DELAY_1MS(),

            I_BL(2, 1),     // skip next instruction if R0 == 0
            I_I2C_DATA_LO(),

            M_DELAY_1MS(),
        I_BXR(R3),


        M_LABEL(SUB_I2C_READ_BIT),
            I_I2C_DATA_HI(),

            I_I2C_CLOCK_HI(),
            M_DELAY_1MS(),


        I_BXR(R3),


        M_LABEL(SUB_I2C_INIT),

        I_BXR(R3),


        M_LABEL(SUB_I2C_START),

        I_BXR(R3),


        M_LABEL(SUB_I2C_STOP),

        I_BXR(R3),


        M_LABEL(SUB_I2C_WRITE),

        I_BXR(R3),


        M_LABEL(SUB_I2C_READ),

        I_BXR(R3),
        


        //main code

    //M_RETURN


       /*
        M_LABEL(1),
        I_MOVI(R2,0),
        I_GPIO_OUTPUT_RD(LED_PIN),
        I_BL(3, 1),  // jumps to enable LED
        I_GPIO_OUTPUT_DIS(LED_PIN),
        I_BGE(2, 0), // jumps out of if statement (to delay)
        //M_BX(2),
        I_GPIO_OUTPUT_EN(LED_PIN),
        M_LABEL(2),
        M_DELAY_MS_20_1000(1000),
        M_BX(1),
*/

       /*
        M_LABEL(1),
        I_GPIO_OUTPUT_RD(LED_PIN),
        M_BL(2, 1),
        I_GPIO_OUTPUT_DIS(LED_PIN),
        M_BX(3),
        M_LABEL(2),
        I_GPIO_OUTPUT_EN(LED_PIN),
        M_LABEL(3),
        M_DELAY_MS_20_1000(500),
        M_BX(1),
    */



        /*
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
        */
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

    pinMode(LED_PIN, OUTPUT);

    /*
    Wire.begin();
    DS3231_init(DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    creg = DS3231_get_creg();

    sensor.initializeMS_5803(false);
    sensor.readSensor();
    */

    hulp_configure_pin(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1);
    hulp_peripherals_on();

    init_ulp();
}

void loop() {
    printf("Read8: %u, Read16: %u, NACK Errors: %u, Bus Errors: %u\n", ulp_data8.val, ulp_data16.val, ulp_nacks.val, ulp_buserrors.val);
    printf("D1: %x\tD2: %x\n", sensor.D1val(), sensor.D2val());
    delay(1000);
}