#include "MS5803_05.h" // https://github.com/benchittle/MS5803_05, a fork of Luke Miller's repo: https://github.com/millerlp/MS5803_05

// Create an instance of the sensor
MS_5803 sensor(4096);
int count = 0;

void setup() {
    Serial.begin(115200);

    // The I2C pull up resistors are on the DS3231 board, which is pin powered
    // (the DS3231's VCC is powered by a GPIO pin). We need to provide power so
    // they can function as pull *up* resistors for I2C to work.
    pinMode(D3, OUTPUT);
    digitalWrite(D3, HIGH);

    Serial.println("Initializing sensor...");
    if (!sensor.initializeMS_5803(true)) {
        Serial.println("INITIALIZATION FAILED");
    } else {
        Serial.println("INITIALIZATION SUCCEEDED");
    }
}

// Print a pressure and temperature reading from the sensor every second.
void loop() {
    sensor.readSensor();
    Serial.printf("Reading #%d: \tP=%f mbar \tT=%f C\n", count, sensor.pressure(), sensor.temperature());
    count++;
    delay(1000);
}