# Project Notes

*Ongoing notes are kept at the top of this file. Dated entries with various thoughts and findings follow.*

## Overview
These notes begin with the development of a new pressure sensor, after DIY2. With this new sensor, the goal is to be able to deploy multiple sensors nearshore ~~and maintain a wired connection from a central device(s?) on the shore to each sensor underwater. This will supply power to each sensor while also allowing them to transmit readings back to the shore in real time.~~


## Sensor Use Cases 

### Deployment
The sensor will be prepared and assembled within the housing before travelling to the deployment site. The sensor will be unpowered until it is ready to deploy. ~~At this point, the sensor will be plugged into a power supply / data channel "hub" via a waterproof cable, turning it on. Readings from the sensor should begin being transmitted to the shore hub.~~ If the sensor is functioning properly, it can be deployed underwater after an additional check to make sure the housing is properly sealed.

---
---

# 10 May 2022

# Working with the ESP32's ULP Coprocessor
(I'm just going to call it the ULP). The ULP is one of the main reasons why I considered the ESP32 despite how power hungry it could be. Based on what I read during my research, it seemed that the ULP couldn't be used for very complex tasks, but it could still interact with certain GPIO's, interact with certain memory (that was shared with the main processor), and communicate via I2C. This was perfect, because it would allow me to read from the MS5803 without even having to wake up the main processor! However, diving into the ULP has been quite a challenge so far.

The first thing to figure out is how you're going to program it. If you're using the ESP's native framework, this isn't a problem and it's well documented by Espressif. However, if you're using the Arduino ESP framework (like me), then here's a list of what I've tried:
1. The [ulptool](https://github.com/duff2013/ulptool) project seemed promising, but I wasn't able to get it working. It might be abandoned at this point, and I don't think it's compatible with newer versions of the ESP Core framework for Arduino. I tried changing to an older version of the framework, but it broke my code and I can't be bothered to migrate backwards.
2. You can migrate your project to the native framework for the ESP. However, I'm using several libraries built for Arduino, and I don't have time to port them to a different framework, although this would likely be the cleanest option in the end.
3. It turns out you can program the the ULP at runtime using some [legacy macros](https://docs.espressif.com/projects/esp-idf/en/v3.3.5/api-guides/ulp_macros.html). It's a little janky and there aren't many examples I can find, but so far it's been working in my program. 

If you choose option 3, there's an amazing little project that makes working with the macros *slightly* higher level called [HULP](https://github.com/boarchuz/HULP). It doesn't offer much more in the way of standalone documentation, but most of the functions are commented well, and there are a number of examples. Even better, the project appears to be active still at the time of writing this.


## JST Connectors and Crimping
In DIY2's design, I created a connector for the MS5803-14 using Dupont connectors. Although they're great for prototyping, they aren't the best choice for a finished product. My biggest gripes with them are that they can be plugged in the wrong way and they don't always make a solid connection. 

So as an alternative for this project, I wanted to use JST connectors. They seemed like a good fit since the MS5803-05 wouldn't have to be unplugged from the board very often (JST connectors aren't technically meant for applications where there's a lot of plugging in and unplugging). I included a footprint for the 4-pin PH variant (there's [a lot](https://en.wikipedia.org/wiki/JST_connector) of JST connectors) and then proceeded to buy precrimped wires for the *SH* variant... which used 2.54mm pin spacing instead of the 2mm *PH* footprint on my board. 

Since the 2mm variant is less common, I couldn't find long enough precrimped wires for my design, so I had to order a kit of various JST PH connectors and a pair of Engineer Crimpers off Amazon to crimp my own (make sure you have wire strippers too, and that your wire is the right AWG for the type of JST connector). I found crimping to be much less frustrating than soldering wires to male Dupont header pins, and the end product was a clean and foolproof connector for the MS5803-05 wires.


# 6 May 2022

## Reducing the ESP32's Current Draw 
There were a number of things one could configure with the ATmega328p to save energy: disabling the ADC, reducing the CPU frequency, etc. With the ESP32, I've been looking into similar functionality. As measured previously, the device with all components attached (FireBeetle board, MS5803-05, DS3231 module, microSD card) drains ~44mA while idle (but not sleeping). The FireBeetle ESP32 is responsible for most of this current draw, given that it's running at a whopping 240MHz and doing... the same thing the ATmega328p was doing at 8MHz (and even that is probably overkill). 

So the first thing to reduce is the CPU frequency, which can easily be done at runtime with the `setCpuFrequencyMhz` function. The [docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html) only show three frequencies from 240MHz to 80MHz, but more are listed in the header file, all the way down to 10MHz (though below 80MHz you can't make use of WiFi / BlueTooth). 

The ADC seems to be disabled until it's actually needed, so there's nothing to worry about there. As for WiFi and BlueTooth, those are also disabled by default. Those seem to be the easy targets, but if any more modules can be disabled then I'll take note of them.


# 5 May 2022

## Sleep Current Tests
Today I wired up a sensor and uploaded a demo of what the setup code will be: initialize the RTC, initialize the SD card, initialize the MS5803-05, and then go to sleep. I then measured the current going to the system under different conditions. 

The ESP32 alone will drain tens of milliamps when it's not sleeping, this is why it will be crucial to make use of the ULP coprocessor (ultra low power) to take measurements if possible, or minimize the time spent awake otherwise. With just the MS5803-05 wired up to the ESP32, I observed around 40mA being drained by the system during the setup code, 0.2mA at the beginning of sleep, and 0.1mA shortly after. This was surprisingly high, since I had observed the ESP32 sleeping at only 0.013mA (or 13uA), and the MS5803-05's datasheet claims it functions at <1uA and sleeps at <0.15uA (150 nano Amps!). 

Continuing with the tests, however, I introduced the DS3231 module into the circuit with [a GPIO pin powering VCC](https://thecavepearlproject.org/2014/05/21/using-a-cheap-3-ds3231-rtc-at24c32-eeprom-from-ebay/) (i.e. the DS3231's VCC is wired to a GPIO pin so it can be easily switched off when I2C is not needed. I noticed that a short delay was needed after reenabling VCC power before you could communicate again). I expected the current consumption to be the same as before once the GPIO pin powering the DS3231 was set to low, but instead the sleep current went even lower: all the way down to 0.013mA, or 13uA, the sleep current of just the ESP32! At first I thought my circuit was either wrong now or wrong in the last setup, but after retesting both circuits, the results were the same. Although I'm not entirely sure why this is the case, I think it has to do with the fact that the design relies on the pull-up resistors found on the DS3231 module for the I2C lines, which weren't present in the previous circuit. Whatever the reason, this result was very promising.

Finally, I introduced the microSD to the circuit. Without a card inserted into the module, the setup consumed no additional current compared to the last test (as one would hope from an open circuit...). With the card, however, around 1.2mA of current was being devoured by the system. In other words, if the only function of this device was to sleep, the microSD card would consume 9000% more energy than the rest of the circuit...


# 3 May 2022

## Controlling Power to the microSD Card
Being unfamiliar with the available options for electronic switches, my first thought was to use a BJT (a basic kind of transistor) to toggle the power to the card on and off using a GPIO pin from the MCU (I can't just pin power the card because current spikes during certain operations could exceed the ESP32's max current draw from GPIO pins). The card itself would be connected to the 3V3 line of the FireBeetle board. Unfortunately, it seems BJTs see a relatively large voltage drop across the collector and emitter (I was measuring ~0.5V using an SS8050 NPN transistor) which would knock the voltage down too far for the microSD (it should be at 3.3V). 

A little extra reading introduced me to an alternative solution: MOSFETs. These transistors function similarly to BJTs, but they switch on/off based on a voltage threshold (rather than current with BJTs), and more importantly, they tend to have much lower voltage drops. As of now, my understanding is that a low-side logic-level N-channel MOSFET would be a good choice for this project. (Low-side means the MOSFET goes after the microSD in the circuit, N-channel is like how BJTs have NPN vs. PNP, and logic-level means the threshold voltage of the device is low e.g. 2.5V or lower for a 3.3V MCU.) [This one, the IPP055N03LGXKSA1](https://www.digikey.ca/en/products/detail/infineon-technologies/IPP055N03LGXKSA1/1996759), might do the job.

I might not have time to add this to the current design before the upcoming deployment, however, and this could really impact battery life. Assuming a capacity of 2000mAh in the 3 AA alkaline batteries, and a sleep current of 0.5mA without switching off the microSD, the device would naively last for around 2000/0.5 = 4000 hours. This isn't so bad, but in reality the device will be drawing more current, and sleep current *should* be <0.1mA. When the ESP32 wakes up to take a sample, it could draw several mA of current for a short period of time. Writing to the microSD, even if I can buffer several hours of data, is also capable of eating up current. I can't really estimate the impact on longevity without more tests, but I doubt it will be negligible.


# 2 May 2022

## Beginning Construction of the First DIY3 Board
The custom PCB has been designed, ordered, and received. I used JLCPCB as the boardhouse and it turned out great, but as expected, I'm already noticing design issues.

After reading up on the Cave Pearl Project's notes on [saving power with microSD cards](https://thecavepearlproject.org/2017/05/21/switching-off-sd-cards-for-low-power-data-logging/), I realized that I underestimated the power consumption of the microSD card and DS3231. Using some DeepSleep example code from the ESP32 framework for Arduino, and after cutting [a connection on the board itself](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#target_4) (see the "Low Power" pad), I was able to get the FireBeetle's current consumption down to ~10 uA as expected. However, just adding the RTC module and microSD card to the circuit bumped the current consumption up to between 0.4 mA and 1.2 mA depending on the microSD card. I'm thinking about dealing with this the same way the Cave Pearl Project did: pin power the RTC module and use a transistor (a BJT) as a switch to toggle the power to the microSD during sleep. Unfortunately, that means I'll have to do some janky soldering on the custom PCB, which I'll have to redesign in the future.


# 14 Apr 2022

## Getting ready to order the parts
There's a lot of scattered thinking involved when ordering parts for something you haven't fully thought out and tested. The main aspects of the sensor are the same as with DIY2: MS5803-05B pressure transducer (we used an MS5803-14BA for DIY2 but the hardware footprint is identical between all the models), DS3231SN (or DS3231M) RTC breakout, microSD breakout. However, since the parts won't be assembled on a breadboard, I have to think about how I'll be wiring things together. 
- Like with DIY2, I'm planning on wiring the MS5803 and DS3231 to the same I2C bus to communicate with the ESP32; the question is, what is the fastest and easiest way to merge the 4 wires from the MS5803 and the 4 wires from the DS3231 into 4 wires that feed to the ESP32 I2C bus (and power)? 
    - I could solder four junctions of three wires. I don't want to do this. I find that oldering wires together is finicky and time consuming. It would also be ideal to have a solution where I can still disconnect each components from one another.
    - Another solution I looked into was to lead male Dupont wires (or similar) from each of the MS5803 and DS3231 and have them connect to a 2x4 female Dupont connector that would somehow merge them and lead back to the ESP32. The more modular design would be nice, but I'm not quite sure how I would actually merge each pair of wires into one still. This would also likely require the use of a crimping tool and kit, which myself and others who want to build this device would need to buy.
    - [Wego lever nuts](https://www.wago.com/ca-en/lp-221) are a cool way to join wires, but they could be a bit too expensive to use in bulk. There are other solutions for joining wires like this, likely inexpensive, but also permanent. So finally, here's the solution I'm looking at.
    - I could design a custom PCB shield for the FireBeetle to mount connectors for the MS5803 cable, the DS3231 (directly or via cable), and even the microSD board. The traces on the PCB would merge the two I2C connections into the ESP32. Everything could remain modular. Soldering through-hole on a PCB is easy. And since the PCB is just for a couple signal connections and through-holes, I could utilize a relatively cheap manufacturer.

## Creating a Custom Shield for the FireBeetle
The first thought that came to my head when starting this idea was to attach a microSD card port right to the board and ditch the microSD breakout. Hey, I could do the same thing with the DS3231. Actually, I might as well just make the whole ESP32 board myself and -- no. Because then I would have to assemble it myself, and so would anyone else who uses this design. This is one issue I had with the OWHL: if you have the tools to make it, it's a very high quality DIY design. If you don't, it's going to be a daunting investment of time and money that many researchers would not make. Yes, there are board houses that will assemble your PCBs too, but I feel that is beyond the scope of a DIY project like this one. Or maybe it isn't, it's something to investigate later.

So no PCB? Well a PCB might still be useful. The microSD and DS3231 breakout boards are inexpensive (somehow I can buy 4 DS3231SN or M breakout boards on ebay for the price of one DS3231SN chip on Digikey...) and already assembled, that's why we chose them as opposed to making our own PCB with the same components. However, as I mentioned in the above section, the components must now be wired together somehow, rather than connected easily via traces in a PCB. Unless... I just create a shield with a few easy to solder through-hole connectors for the MS5803 cable, DS3231 module, and microSD module that handles all the "wiring" for me. Bingo!

# 05-03-2022

## Change of Plans
This batch of sensors no longer needs to be wired: instead it will once again be battery powered and save data (most likely) to an SD card, essentially the same as DIY2. At this point, an Arduino nano or pro mini could likely be used instead of a handmade board like DIY2 or the FireBeetle. However, I'd like to continue testing.


# 01-03-2022

## Software for Programming an ESP32
There is support for ESP32 programming in the Arduino IDE, but I'd like to have the Intellisense perks that come with an editor like VS Code. I have both the Arduino extension and PlatformIO (PIO) extension set up now. I'm thinking it will be best if I begin programming under the Arduino framework (either using the extension directly or under PIO) since I'm familiar with it, and once I have the functionality I want, I may convert to Espressif's framework for the ESP32 (still under PIO) if there are any major benefits for doing so.


## Prototyping
I'll be ordering 2 or 3 FireBeetle boards, 3 or 4 MAX3485 chips (with boards to break out the SOIC-8 package to pin holes), possibly some ethernet ports. I'll also need to pick up some ethernet cable for long range wired communication and power testing. The goal for a prototype is to have each of the 2 or 3 ESP32 boards read from a pressure sensor (MS5803-14BA) and send the data to the same "hub" computer. This will cover the main desired functionality for the finished sensors.



# 26-02-2022

## Long Range Data Transmission cont.
- Power loss is another factor that needs to be considered here: there will be a voltage drop at each sensor, which could become problematic if the cable is too long. This is because the cable I currently intend to use (ethernet cable of some kind) has very small gauge copper wire, which is less effective for power transmission.
- An additional transceiver component will be needed to use RS-485 communication. There are some very cheap breakout boards available on Amazon and other places that work at 5V, but I'm intending for this project to work at 3(.3)V (maybe this should change?) considering the ESP32 chip requires a voltage in this range. Sparkfun sells a [board](https://www.sparkfun.com/products/10124) that works at 3.3V, but based on the price and the reviews I'd be better off using a chip like this [Maxim one](https://www.maximintegrated.com/en/products/interface/transceivers/MAX3485.html) ([DigiKey link](https://www.digikey.ca/en/products/detail/analog-devices-inc-maxim-integrated/MAX3485CSA-T/1703654)) with perhaps a custom PCB or a small bare breakout PCB for whatever package type the chip comes in. I'm going to investigate the cost of a custom PCB, with room for an ethernet port or other wire connector.


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


