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
#include "RTClib.h"

#define ECHO_TO_SERIAL 1

#define RTC_POWER_PIN GPIO_NUM_26
#define RTC_ALARM GPIO_NUM_25

// The folloing directives are from https://github.com/rodan/ds3231
#define DS3231_I2C_ADDR 0x68
#define DS3231_CONTROL_ADDR 0x0E
// control register bits
#define DS3231_CONTROL_A1IE     0x1		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_A2IE     0x2		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_INTCN    0x4		/* Interrupt Control */
#define DS3231_CONTROL_RS1	    0x8		/* square-wave rate select 2 */
#define DS3231_CONTROL_RS2    	0x10	/* square-wave rate select 2 */
#define DS3231_CONTROL_CONV    	0x20	/* Convert Temperature */
#define DS3231_CONTROL_BBSQW    0x40	/* Battery-Backed Square-Wave Enable */
#define DS3231_CONTROL_EOSC	    0x80	/* not Enable Oscillator, 0 equal on */
//////////////////////////////////////

// Flag that will be set to 1 whenever an interrupt is observed.
volatile boolean isInt = 0;

RTC_DS3231 rtc;

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
    if (!rtc.begin()) {
        Serial.println("RTC Fail");
        return;
    }
    // Clear any previous alarms.
    rtc.disableAlarm(1);
    rtc.clearAlarm(1);
    // We won't be using alarm 2.
    rtc.disableAlarm(2); // Clear alarm enabled flag?
    rtc.clearAlarm(2); // Clear alarm fired flag?

    // Print out the time as a sanity check.
    DateTime now = rtc.now();
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

    // Shut down DS3231's VCC and turn it back on to make sure
    // restarting the chip works.
    digitalWrite(RTC_POWER_PIN, LOW);
    delay(1000);
    digitalWrite(RTC_POWER_PIN, HIGH);
    delay(5);
    now = rtc.now();
    Serial.printf("%d-%02d-%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    

    // Set BBSQW (battery backed square wave) bit in DS3231 control register. 
    // This allows us to generate alarm pulses while the chip is powered only by
    // battery. Also set other default values.
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(DS3231_CONTROL_ADDR);
    Wire.write(DS3231_CONTROL_BBSQW | DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
    Wire.endTransmission();

    // Set the alarm to repeat every second. The time provided doesn't matter.
    rtc.setAlarm1(rtc.now(), DS3231_A1_PerSecond);

    // Attach a simple interrupt to be triggered when the alarm activates
    // (active low).
    Serial.println("Attaching...");
    attachInterrupt(RTC_ALARM, interruptHandler, ONLOW); // ONLOW and ONHIGH, not LOW and HIGH
}

int count = 0;
void loop() {
    // Print a message whenever there is an interrupt
    if (isInt) {
        // Clear the alarm signal so we don't instantly trigger another 
        // interrupt when we enable them again. It will be set again at the 
        // next second.
        rtc.clearAlarm(1);

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