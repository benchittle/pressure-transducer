# Changes from DIY3 to DIY4 Shield Design
- Rotate shield 180 degrees.
- Rotate silkscreen text additional 180 degrees to match orientation in [pinout image](https://image.dfrobot.com/image/data/DFR0654-F/Pinout.jpg).
- Modify silkscreen text.
- Implement all changes from [DIY3's modified design](../../DIY3/firebeetle-shield/modified-firebeetle-shield.kicad_sch). This will save time during assembly, as the changes in DIY3's design had to be made by hand (soldering, cutting wires, etc.) whereas now they are incorporated into the PCB. Some solder bridges were removed accordingly.
- Add high-side MOSFET switch for enabling / disabling power to the SD card using a GPIO pin. The MOSFET and resistor are actually optional, and power can be directly provided to the SD card (like DIY3) depending on the configuration of a solder bridge.
- Redo trace routing to be more consistent (front traces are horizontal, back traces are vertical) 