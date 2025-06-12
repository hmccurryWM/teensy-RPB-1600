# PMBus Library for Arduino

## PMBus-ABIO
This is a heavily modified (and extremely extremely messy, barebones, probably-not-standard-compliant) implementation of a library to communicate with power supplies usign the PMBus standard. This sits on top of I2C, and in the interest of easy compatibility with the project that inspired this it uses the Adafruit I2C wrapper `Adafruit_I2C_Device' to handle communications. There really isn't any other reason to use it, so feel free to rip it out if needed.
I also focused on the parts of the standard that the power supply actually uses, as well as the parts I actually need so its nowhere near complete, nor can I actually guarantee that it actually works well lol. I also wrote this in an extremely ad-hoc manner to get it work well enough for a prototype, so there's an atrocious mishmash of coding styles and conventions involved as well as war crime worthy shortcuts, hacks and generally bad code. While I'd like to imagine I can and will go back and clean it up, refactor it to fit my compulsion for neatness I think anyone who's genuinely interested in using PMBus (especially if its not through arduino!) can relate heavily with the idea of not having the time, energy, or willpower to do so.

I've left the original `README.md` at the bottom as it does have some pertinent information: namely info about PMBus, how to use, and troubleshooting -- NOTE: The documentation for the HEP-1000-100 suggests leaving a 50ms wait in between instructions, so the issues the previous had are almost certainly related to that. HOWEVER that being said I've not run into issues with wait times, though even at a 2MBit baud rate the truly massive amount of debug printfs ive sprinkled in help with that so I'd caution for adding some wait time.

_(Note ABIO is _`Adafruit Bus IO`_, and does not include any ABO fanfiction at this time)_

### Compatibility
The original was written for the teensy and just used wire.h, whereas I'm using an esp32 and using the adafruit busio/i2c device libraries. As they say down below it should work with most if not all other hardware with the exception of at least the Atmega 238 which does not handle printfs (or at least not for me). More obscure and less well supported hardware may also have quirks that aren't accounted for, and that will be up to the individual user to worry about, naturally.

## PMBus Standard
The device I'm writing this for (Meanwell HEP-1000-100) uses the v1.1 standard, so it's built around that. I have next to no familiarity with the standard, so I do not know how it changed across the revisions (i think its v1.4 right now? 1.5?). Also, I'm not quite sure how well the power supply I'm using complies with the standard so there may be design choices I went with to fit this that are not compliant.

## Structure
The idea is to have a common PMBus framework that ideally would have a full implementation of the entire standard, and then have inherited classes for individual devices which can handle any quirks, customizations and anything else unique to the device-- for instance, the HEP-1000-100 does not let you change the output voltage directly; It's a read-only word with a value of '100' in the given linear data format. Instead, what they have you do is use `VOUT_TRIM` to modify the voltage, so the subclass will have functions to calculate and handle that, as well as recover original set points. It might also be a good idea to make wrappers for all of the registers that the device does implement, and only let the end user access them through the subclass (thereby keeping track of which registers are read only or read/write) but that requires PMBus-ABIO to be re-organized so effort.
It might be good to have a local copy of the registers, and treat communication as a syncing but for now alot of them are in structures with related paramters (like mfgr information), which can be copied in-place for perusal outside of the class.






# O.G. Readme

## Teensy-RPB-1600
This is Arduino Library for communicating with and programming the Meanwell RPB-1600 via PMBus with a Teensy 4.0. While this library was designed for use with the Teensy 4.0, it's written on top of the Wire library and should be usable other Arduino hardware. I've been using it as part of [this project](https://github.com/maland16/citicar-charger). In the absence of any real documentation that project, and the curve configurator in this repo, can serve as examples of how to use this library.  

## How to use this library
-Download a zip of this library using the green button above  
-Follow [the instructions here](https://www.arduino.cc/en/guide/libraries) under "Manual installation"  
-Use the public functions defined in "rpb-1600.h" to your heart's content  
-Don't forget to construct a RPB_1600 object and Init()!  
-See the example that's included in this library  

## Curve Configurator  
This example arduino sketch can be used to read data from and write data to the RPB-1600 over the PMBus protocol via I2C.

## Hardware
Communication with the charger is done over I2C using Pins 7 & 8 of CN500 (the smaller 8 pin connector). [See Meanwell's instructions for more details](https://www.meanwell.com/webapp/product/search.aspx?prod=RPB-1600). I didn't find it necessary to use pull up resistors in addition to the ones internal to the teensy, but you milage may vary. I purchased the connectors and crimped my own wires and I highly recommend this. At first I tried to make due with a hacky solution but buying the right connector was infinitely less frustrating. The connector is a Hirose HRS DF11-16DS I believe.   

## PMBus i2c Protocol
The PMBus protocol can be found on [the PMBus website](https://pmbus.org/specification-archives/). You have to be granted access to the latest specifications, but the older ones (like the version the RPB-1600 uses) are free under their "archives" section. **There were multiple instances where the RPB-1600 datasheet is misleading about how to write or read data from it. Included in this repo is an email exchange between myself and a Meanwell rep who helped me sort through some of the issues I was having.

## Troubleshooting  
### Write/Read speed  
I ran into an issue recently while developing the citicar charger where I would write and then immediately read, and either the data I read out was stale or the read operation interrupted the read. Either way, I'd recommend waiting a bit after a write operation before reading  

## Future work  
¯\_(ツ)_/¯
