
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

volatile boolean isInt = 0;

RTC_DS3231 rtc;

void IRAM_ATTR interrupt() {
    isInt = 1;
}


void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    delay(1000);

    pinMode(RTC_POWER_PIN, OUTPUT);
    // The DS3231's alarm is active low, so we need to pull it high.
    pinMode(RTC_ALARM, INPUT_PULLUP);
    // The DS3231's VCC is hooked up to a GPIO pin, so we need to
    // provide power before attempting I2C.
    digitalWrite(RTC_POWER_PIN, HIGH);
    // Short delay for RTC to switch from battery to VCC.
    delay(5);
    if (!rtc.begin()) {
        Serial.println("RTC Fail");
        return;
    }
    // Clear any previous alarm
    rtc.clearAlarm(1);
    // We won't be using alarm 2, so clear it.
    rtc.disableAlarm(2);

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

    // Enable battery backed square wave / maybe alarms too?
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(DS3231_CONTROL_ADDR);
    Wire.write(DS3231_CONTROL_BBSQW);
    Wire.endTransmission();

    // Set the alarm to repeat every second. The time provided doesn't matter (?)
    //rtc.setAlarm1(rtc.now(), DS3231_A1_PerSecond);

    // TODO: Desolder the resistor array on the DS3231 and use my own 
    // pull ups somewhere else.

    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);


    // Attach a simple interrupt to be triggered when the alarm activates
    // (active low). I've noticed that the interrupt doesn't work if the 
    // trigger level is set to LOW (the voltage hovers just above 0V) but
    // a FALLING condition does work.
    attachInterrupt(RTC_ALARM, interrupt, FALLING);
    
}
int count = 0;
void loop() {

    if (isInt) {
        digitalWrite(RTC_POWER_PIN, HIGH);
        delay(5);
        rtc.clearAlarm(1);
        Serial.print("high @ ");
        Serial.println(millis());
        isInt = 0;
        count++;
    }
    if (count == 5) {
        digitalWrite(RTC_POWER_PIN, LOW);
    }
}