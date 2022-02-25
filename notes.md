# Project Notes

*Ongoing notes are kept at the top of this file. Dated entries with various thoughts and findings follow.*

## Overview
These notes begin with the development of a new pressure sensor, after DIY2. With this new sensor, the goal is to be able to deploy multiple sensors nearshore and maintain a wired connection from a central device(s?) on the shore to each sensor underwater. This will supply power to each sensor while also allowing them to transmit readings back to the shore in real time.


## Sensor Use Cases 

### Deployment
The sensor will be prepared and assembled within the housing before travelling to the deployment site. The sensor will be unpowered until it is ready to deploy. At this point, the sensor will be plugged into a power supply / data channel "hub" via a waterproof cable, turning it on. Readings from the sensor should begin being transmitted to the shore hub. If the sensor is functioning properly, it can be deployed underwater after an additional check to make sure the housing is properly sealed.
   
---

# 25-02-2022

## Long Range Data Transmission
Each sensor will be connected to the shore hub via a wired connection in order to receive power and transmit data. However, this distance will undoubtedly be longer than just a few meters, making communication protocols like I2C and SPI ineffective. If the sensors are to be underwater, Wi-Fi is (probably) not an option either. For this reason, a longer range wired communication protocol will be required, like [RS-485](https://www.omega.ca/en/resources/rs422-rs485-rs232), as well as a transceiever in order to use it. The cable used is also important: twirled pair cables (like in ethernet cables) reduce interference. The cable will need to be waterproof, which is supposedly best achieved by an outdoor rated gel filled cable.

    
# 24-02-2022

## New Course of Action
Although I would like to design my own PCB for a project at some point, I need to learn more about the ESP32 before I can do that for this project. After further research, the [FireBeetle](https://www.dfrobot.com/product-1590.html) ESP32 development board seems like a very good alternative to a self made board (which would essentially need the same features for this project). It boasts a deep sleep current of just 10 μA and will keep development time on track. 

  
# 23-02-2022

## Plan of action
I think I'll design a PCB using an ESP chip first, since power savings aren't an issue for the current use case. If the power consumption of the ESP chip can't be reduced enough, an STM32 design might be fleshed out in the future.


## Choosing an ESP Chip

### Why switch from the ATMEGA328p?
- Likely due to chip shortages and the age of the chip, it's difficult to find sellers who stock it. Current lead times from DigiKey were around a year. This has led me to consider two other chips: the STM32 and ESP32.

### ESP32 vs. STM32 main differences so far:
- ESP has Wi-Fi / Bluetooth capability whereas STM does not.
- STM may be more power efficient out of the box; in order to save power on the ESP, it looks like I'll need to use the Ultra Low Power coprocessor while keeping the main processor in deep sleep most of the time. The downside here is that the coprocessor might have to be programmed in assembly.
- ESP packages seem to be physically bigger than STM packages.

### There are [4 Series](https://www.espressif.com/en/products/modules) to consider 

- [ ] ESP8266 Series
    - Oldest model, this one has some security issues and lacks Bluetooth which is a feature I'd like to have as an option. It's very commonly used despite these issues, however.
- [x] ESP32 Series
    - Like the above chip, but with security issues fixed and Bluetooth capability. This is the Series I first considered.
- [ ] ESP32-C Series
    - Newer and has every feature I'm looking for except for the ultra low power co-processor, which I think I'll need in order to pursue a low power model that could compete with STM32 chips.
- [x] ESP32-S Series
    - Broken up into S2 and S3: S2 ditches the Bluetooth connectivity ~~which makes it a no go~~, S3 is the newest Series and is also currently out of stock on Digikey, otherwise this might be a viable frontrunner. 
    - **UPDATE**: Having discovered that Bluetooth is limited to ~1Mb/s transfer speed, I think using a Wi-Fi connection is the way to go. For this reason, I don't really care about Bluetooth, and instead prefer the S2 model or the ESP32 base one. 


### There are various "Modules" of the ESP32 Series to consider
*Note: Modules are listed by ESP Series on the page linked in the previous section.*

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


