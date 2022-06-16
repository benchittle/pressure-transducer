#include <Wire.h>
#include <esp32/ulp.h>
#include <esp32-hal-i2c.h>

#include "hulp.h"
#include "hulp_i2cbb.h"
#include "ds3231.h"
#include "MS5803_05.h"




#define ULP_SDA GPIO_NUM_0
#define ULP_SCL GPIO_NUM_4

#define ESP_SDA GPIO_NUM_21
#define ESP_SCL GPIO_NUM_22


// first send CMD_ADC_CONV + CMD_ADC_Dx + CMD_ADC_4096
// then wait 10ms
// then 



#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13

#define LED_PIN SDA_PIN

// TODO: Hi should be hi impedence (input), low should be driving low

#define M_I2C_DATA_HI() I_GPIO_OUTPUT_DIS(SDA_PIN)
#define M_I2C_DATA_LO() \
    I_GPIO_OUTPUT_EN(SDA_PIN), \
    I_GPIO_SET(SDA_PIN, 0)

#define M_I2C_CLOCK_HI() I_GPIO_OUTPUT_DIS(SCL_PIN)
#define M_I2C_CLOCK_LO() \
    I_GPIO_OUTPUT_EN(SCL_PIN), \
    I_GPIO_SET(SCL_PIN, 0)


#define M_I2C_DELAY() M_DELAY_MS_20_1000(500)

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

RTC_DATA_ATTR ulp_var_t ulp_var1;

uint8_t creg = 0;
uint8_t sdaVal, sclVal;

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

        L_RET00,
        L_RET01,
        L_RET02,
        L_RET03,
        L_RET04,
        L_RET05,
        L_RET06,
        L_RET07,
        L_RET08,
        L_RET09,

        L_LOOP00,
        L_LOOP01

    };

    // Note:
    // R0 is used to pass an argument to a subroutine
    // R2 is used to return values from subroutines
    // R3 is used to store the return address for a subroutine
    const ulp_insn_t program[] = {
        // M_I2C_DELAY(),

        /*
        I_MOVI(R0, 0),
        I_MOVI(R1, 0),
        I_MOVI(R2, 0b11111111),
        I_LSHI(R1, R2, 1),
        I_PUT(R1, R0, ulp_var1),
        */

        M_I2C_DELAY(),
        
        // Send start condition
        M_RETURN(L_RET02, R3, SUB_I2C_START),

        // Send address of DS3231 and write bit
        I_MOVI(R0, 11010000),
        M_RETURN(L_RET03, R3, SUB_I2C_WRITE),

        // Read for the acknowledge bit
        M_RETURN(L_RET04, R3, SUB_I2C_READ_BIT),
        I_MOVI(R1, 0),
        I_PUT(R2, R1, ulp_var1),
        /*
        // Send write address for DS3231
        I_MOVI(R0, DS3231_CONTROL_ADDR),
        M_RETURN(L_RET05, R3, SUB_I2C_WRITE),

        // Read for the acknowledge bit
        M_RETURN(L_RET06, R3, SUB_I2C_READ_BIT),

        // Send new value for control register for DS3231
        I_MOVI(R0, DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN),
        M_RETURN(L_RET07, R3, SUB_I2C_WRITE),

        // Read for the acknowledge bit
        M_RETURN(L_RET08, R3, SUB_I2C_READ_BIT),

        // Send stop condition
        M_RETURN(L_RET09, R3, SUB_I2C_STOP),
        */
        I_END(),
        I_HALT(),


        
        // Write a single bit to the I2C slave
        // REQUIRES: R0 contains the bit value to be written
        //           R3 contains return address
        M_LABEL(SUB_I2C_WRITE_BIT),
            I_BL(3, 1),     // skip next two instructions if R0 == 0
            M_I2C_DATA_LO(),
            I_BGE(2, 0),    // skip next instruction otherwise
            M_I2C_DATA_HI(),

            M_I2C_CLOCK_HI(),
            M_I2C_DELAY(),

            M_I2C_CLOCK_LO(),
            M_I2C_DELAY(),

            //I_BL(2, 1),     // skip next instruction if R0 == 0
            //M_I2C_DATA_LO(),

            //M_I2C_DELAY(),
        I_BXR(R3),


        // Read a single bit from the I2C slave
        // REQUIRES: R3 contains return address
        // RETURNS: Bit read from I2C in R2
        M_LABEL(SUB_I2C_READ_BIT),
            M_I2C_DATA_HI(),

            M_I2C_CLOCK_HI(),
            M_I2C_DELAY(),

            I_GPIO_READ(SCL_PIN),
            I_BL(-1, 1),

            I_GPIO_READ(SDA_PIN),

            M_I2C_CLOCK_LO(),
            M_I2C_DELAY(),

            I_MOVR(R2, R0),
        I_BXR(R3),


        M_LABEL(SUB_I2C_INIT),
            M_I2C_CLOCK_HI(),
            M_I2C_DATA_HI(),
            M_I2C_DELAY(),
        I_BXR(R3),


        // Send a start condition.
        // REQUIRES: R3 contains return address
        M_LABEL(SUB_I2C_START),
            M_I2C_DATA_HI(),
            M_I2C_CLOCK_HI(),
            M_I2C_DELAY(),

            M_I2C_DATA_LO(),
            M_I2C_DELAY(),

            M_I2C_CLOCK_LO(),
            M_I2C_DELAY(),
        I_BXR(R3),


        // Send a stop condition.
        // REQUIRES: R3 contains return address
        M_LABEL(SUB_I2C_STOP),
            M_I2C_CLOCK_HI(),
            M_I2C_DELAY(),

            M_I2C_DATA_HI(),
            M_I2C_DELAY(),
        I_BXR(R3),


        // Write a byte to I2C device.
        // REQUIRES: R0 contains byte to write
        //           R3 contains return address
        // CHANGES: R0, R1, R2
        M_LABEL(SUB_I2C_WRITE),
            // Copy argument into R2.
            I_MOVR(R2, R0),
            // Reset the stage counter (loop counter) USED BY DELAY FN.
            I_STAGE_RST(),

            // Loop start
            M_LABEL(L_LOOP00),

            
            // Move MSB of R2 to R0 and shift R2 left
            I_ANDI(R0, R2, 0b10000000),
            //I_RSHI(R0, R0, 7),
            I_LSHI(R2, R2, 1),

            I_MOVI(R1, 0),
            I_PUT(R0, R1, ulp_var1),

            // Save return address.
            I_MOVR(R1, R3),
            // Enter WRITE_BIT subroutine and return below.
            M_RETURN(L_RET00, R3, SUB_I2C_WRITE_BIT),
            // Put return address back into R3
            I_MOVR(R3, R1),

            // Repeat loop 8 times total (1 byte)
            I_STAGE_INC(1),
            //M_BX(L_LOOP00),
            M_BSLE(L_LOOP00, 251),
        I_BXR(R3),


        // Read a byte from I2C slave.
        // REQUIRES: R3 contains return address
        // CHANGES: R0, R1
        // RETURNS: Byte read in R2
        M_LABEL(SUB_I2C_READ),
            // Clear register R0 so we can use it to store the byte.
            I_MOVI(R0, 0),
            // Reset the stage counter (loop counter).
            I_STAGE_RST(),

            // Loop start
            M_LABEL(L_LOOP01),
            
            // Save return address.
            I_MOVR(R1, R3),
            // Enter READ_BIT subroutine and return below.
            M_RETURN(L_RET01, R3, SUB_I2C_READ_BIT),
            // Put return address back into R3
            I_MOVR(R3, R1),
            // Shift R0 to make room for the next bit
            I_LSHI(R0, R0, 1),
            // Place the bit we read into the LSB of R0
            I_ORR(R0, R0, R2),

            // Repeat loop 8 times total (1 byte)
            I_STAGE_INC(1),
            M_BSLT(L_LOOP01, 8),

            // Move the byte we read into R2 (return value register)
            I_MOVR(R2, R0),

            M_I2C_DELAY(),
        I_BXR(R3),
        


       /*
        M_LABEL(99),
        I_MOVI(R2,0),
        I_GPIO_OUTPUT_RD(LED_PIN),
        I_BL(3, 1),  // jumps to enable LED
        I_GPIO_OUTPUT_DIS(LED_PIN),
        I_BGE(2, 0), // jumps out of if statement (to delay)
        I_GPIO_OUTPUT_EN(LED_PIN),
        M_DELAY_MS_20_1000(1000),
        M_BX(99),
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
    
    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_OUTPUT, GPIO_FLOATING, 0));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void setup() {
    Serial.begin(115200);
    Serial.println("START");

    ulp_var1.val = 0;

    //pinMode(LED_PIN, OUTPUT);

    
    /*
    sensor.initializeMS_5803(false);
    sensor.readSensor();
    */

    //hulp_configure_pin(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1);
    hulp_peripherals_on();

    pinMode(ESP_SDA, INPUT);
    pinMode(ESP_SCL, INPUT);

    init_ulp();
    //delay(10000);
/*
    Wire.begin();
    //DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    creg = DS3231_get_creg();
    printf("start var1: %d\nstart creg: %d\n\n", ulp_var1.val, creg);
    */
    while (!digitalRead(ESP_SCL) || !digitalRead(ESP_SDA));
    //delay(250);
}



void loop() {
    //printf("var1: %d\ncreg: %d\n\n", ulp_var1.val, DS3231_get_creg());
    //delay(1000);
    uint8_t flag = 0;
    sdaVal = digitalRead(ESP_SDA);
    sclVal = digitalRead(ESP_SCL);
    
    printf("SDA: %d \tSCL: %d \tulp: %x\n", sdaVal, sclVal, ulp_var1.val & 0xFF);

    while(digitalRead(ESP_SCL)) {
        if (sdaVal && !digitalRead(ESP_SDA) && !flag) {
            printf("START COND\n");
            flag = 1;
        }
    }
    while(!digitalRead(ESP_SCL));
}