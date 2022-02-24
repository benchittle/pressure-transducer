# Some project notes for personal use


### Why switch from the ATMEGA328p?
- Likely due to chip shortages and the age of the chip, it's difficult to find sellers who stock it. Current lead times from DigiKey were around a year. This has led me to consider two other chips: the STM32 and ESP32.


### ESP32 vs. STM32 main differences so far:
- ESP has Wi-Fi / Bluetooth capability whereas STM does not.
- STM may be more power efficient out of the box; in order to save power on the ESP, it looks like I'll need to use the Ultra Low Power coprocessor while keeping the main processor in deep sleep most of the time. The downside here is that the coprocessor might have to be programmed in assembly.
- ESP packages seem to be physically bigger than STM packages.


### Plan of action
- I think I'll design a PCB using an ESP chip first, since power savings aren't an issue for the current use case. If the power consumption of the ESP chip can't be reduced enough, an STM32 design might be fleshed out in the future.



## Choosing an ESP Chip

### There are [4 Series](https://www.espressif.com/en/products/modules) to consider 

- [ ] ESP8266 Series
    - Oldest model, this one has some security issues and lacks Bluetooth which is a feature I'd like to have as an option. It's very commonly used despite these issues, however.
- [x] ESP32 Series
    - Like the above chip, but with security issues fixed and Bluetooth capability. This is the Series I'm currently considering.
- [ ] ESP32-C Series
    - Newer and has every feature I'm looking for except for the ultra low power co-processor, which I think I'll need in order to pursue a low power model that could compete with STM32 chips.
- [ ] ESP32-S Series
    - Broken up into S2 and S3: S2 ditches the Bluetooth connectivity which makes it a no go, S3 is the newest Series and is also currently out of stock on Digikey, otherwise this might be a viable frontrunner.


### There are various "Modules" of the ESP32 Series to consider
Note: Modules are listed by ESP Series on the page linked in the previous section.

- Some modules have a "PCB antenna" as a trace built directly into the module's PCB, whereas others have an "IPEX antenna" which I believe requires you to connect an external wire / antenna piece.
    - Without having done much research, I think the PCB antenna should suffice. I would only need it to communicate with a device directly next to the sensor, maybe a phone held up against the sensor housing.
- Some modules allow you to select from varying amounts (4, 8, 16 MB) of flash (program) memory.
    - The 8 MB flash option appears to be the least expensive option in stock for some of the modules on DigiKey, so I'll choose this if the option is there. I'm not worried about program space given the ATMEGA328p's comparatively miniscule flash memory.
- There is one major difference that I have found between modules from the WROOM series and modules from the WROVER series: modules from the WROVER series come with an additional 8 MB PSRAM (pseudo static RAM) chip inside the module package which gives the module a significant amount of additional (slightly slower?) memory to use.
    - Considering I was able to comfortably use the 2KB of RAM and 32KB of flash provided by the ATMEGA328p for the DIY2 design, I don't think I need a WROVER series module which may drain more power with the extra RAM chip. The ESP's ~500 kB of RAM should be plenty.
- The package size is another thing to consider. While the mini version appears to have all the same features as the standard packages, it might be harder to handle / install by hand. 
    - Since I don't think space will be a major constraint, I will choose the standard larger package size.
- There are several revisions of the modules I've narrowed down (first iteration: ESP32-WROOM-32, ..., later: ESP32-WROOM-32D, ..., most recent: ESP32-WROOM-32E). The 32E module changes a few major things from the 32D and 32 modules: it fixes some security bugs where one could access the code on the chip and some read/write bugs, but most importantly it turns GPIO pins 17-22 into NC (no connect) pins. I *believe* this is because they were meant to be used to connect to additional external program memory in older versions, but with E they package that additional memory within the chip
    - The 32E module is slightly cheaper than the D (at least from what I saw on DigiKey). While I wouldn't consider the improvements the E offers over the other versions necessary for the project (I don't think the GPIO pins lost in the E are gamebreaking), I see no reason why I shouldn't begin modelling and testing with this version to begin.

**My module of choice: the ESP32-WROOM-32E**


