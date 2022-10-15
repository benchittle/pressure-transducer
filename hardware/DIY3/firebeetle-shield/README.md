This shield design was used to create 14 DIY3 sensors, but several modifications had to be made to it by hand to optimize and correct the design. The Kicad files here detail the original design, while the file [`modified_firebeetle_shield.kicad_pcb`](./modified_firebeetle_shield.kicad_pcb) shows the schematic for the shield after being modified by hand. These modifications involve soldering new "traces", adding some components, and cutting a pin after the shield has been assembled:

1. Cut a small length of wire and make a connection between D3 and the 3V3 pin **for the DS3231**. Also make sure that  "3V3 to RTC" solder jumper has been cut. Reason: This allows us to pin power the DS3231.

2. Cut pin D9 and solder a connection between D9 and D7. Reason: D9 is used as the CS pin for the SD card, but it also controls an LED on the FireBeetle. To avoid wasting power to the LED whenever we want to use the SD card, we modify the shield to use D7 as the CS pin instead of D9.

3. Solder a connection between D6 and SDA, and another between D5 and SCL. Reason: D5 and D6 are used by the ULP as SCL and SDA respectively, and we want to allow the ULP to communicate on the same I2C bus as the other devices.

4. Add pull up resistors for I2C (add a 10K resistor between SDA and 3V3, and another between SCL and 3V3). Reason: pull up resistors are needed for I2C.