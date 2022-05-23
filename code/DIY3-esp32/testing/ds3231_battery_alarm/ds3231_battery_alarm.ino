/*
 * This file contains code for experimenting with with a DS3231 module.
 * The DS3231 module has been stripped down to just the DS3231, 2 capacitors,
 * and a coin cell holder. 
 * 
 * The program first initializes the DS3231 and gets the time. Then, the once per
 * second alarm is enabled on the DS3231 and used to trigger an interrupt on the 
 * ESP32. Finally, VCC power to the DS3231 is disabled to make sure it will 
 * continue to run on just a battery.
 */


#include <Wire.h>
#include "ds3231.h" // https://github.com/rodan/ds3231

#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25


// Flag that will be set to 1 whenever an interrupt is observed.
volatile boolean isInt = 0;

void IRAM_ATTR interruptHandler() {
    isInt = 1;
    // Detach interrupts while we handle this one, otherwise we'll get stuck in
    // a neverending stream of interrupts.
    detachInterrupt(RTC_ALARM);
}


void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    delay(1000);

    pinMode(RTC_POWER_PIN, OUTPUT);
    // The DS3231's alarm is active low, so we need to pull it high.
    pinMode(RTC_ALARM, INPUT_PULLUP);
    // The DS3231's VCC is hooked up to a GPIO pin, so we need to provide power
    // before attempting I2C.

    digitalWrite(RTC_POWER_PIN, HIGH);
    // Short delay for RTC to switch from battery to VCC.
    delay(5);

    // Set BBSQW (battery backed square wave) bit in DS3231 control register. 
    // This allows us to generate alarm pulses while the chip is powered only by
    // battery. Also set other default values.
    // TODO: Need a way to determine if this fails.
    Wire.begin();
    DS3231_init(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    // Clear any previous alarms.
    DS3231_clear_a1f();
    DS3231_clear_a2f();


    // Print out the time as a sanity check.
    struct ts now;
    DS3231_get(&now);
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", now.year, now.mon, now.mday, now.hour, now.min, now.sec);

    // Shut down DS3231's VCC and turn it back on to make sure
    // restarting the chip works.
    digitalWrite(RTC_POWER_PIN, LOW);
    delay(1000);
    digitalWrite(RTC_POWER_PIN, HIGH);
    delay(5);
    DS3231_get(&now);
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", now.year, now.mon, now.mday, now.hour, now.min, now.sec); 
    
    // Attach a simple interrupt to be triggered when the alarm activates
    // (active low).
    attachInterrupt(RTC_ALARM, interruptHandler, ONLOW); // ONLOW and ONHIGH, not LOW and HIGH

    // Set the alarm to repeat every second. The time provided doesn't matter.
    Serial.println("Starting alarm...");
    // TODO: Modify library to make this better
    const uint8_t flags[] = {1,1,1,1,0};
    DS3231_set_a1(1, 1, 1, 1, flags);
    DS3231_set_creg(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE);
}

int count = 0;
void loop() {
    // Print a message whenever there is an interrupt
    if (isInt) {
        // Clear the alarm signal so we don't instantly trigger another 
        // interrupt when we enable them again. It will be set again at the 
        // next second.
        DS3231_clear_a1f();

        Serial.print("low @ ");
        Serial.println(millis());
        isInt = 0;
        count++;

        // Attach the interrupt again.
        attachInterrupt(RTC_ALARM, interruptHandler, ONLOW);
    }

    // After 5 interrupts, disable VCC power to the DS3231. It should continue
    // to run on battery power.
    if (count == 5) {
        digitalWrite(RTC_POWER_PIN, LOW);
        Serial.println("OFF");
        count++;
    }
}