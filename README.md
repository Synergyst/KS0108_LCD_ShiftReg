# KS0108_GLCD_ShiftReg
Arduino library for KS0108 controller based monochrome gLCD using 74LS164 shift register for data-bus control

This is a library for controlling KS0108-based monochrome Graphics Liquid Crystal Displays (gLCD) using a 74LS164 shift register.
This library works with gLCD modules with 1, 2, or 3 KS0108 chips.
A gLCD module with one-chip KS0108 modules (1 CS pin) the resolution is 64x64 pixel,
with two-chip KS0108 modules (2 CS pins) the resolution is 128x64 pixels,
and a three-chip module (3 CS pins) the resolution is 192x64 pixels.
This type of display normally uses parallel interface to communicate with the MCU,
though when using a 74LS164 shift register it only requires up to 8 GPIO pins (when using a 3-chip module)
depending on the resolution of the display (less than three KS0108 chip-select pins requires less I/O pins).

Forked from:


Website URL:
https://simple-circuit.com/

Example URL:
https://wp.me/p9n96B-2Oc

*****************************************************************************************
This library depends on Adafruit GFX library at:
https://github.com/adafruit/Adafruit-GFX-Library
being present on your system. Please make sure you have installed the latest version before using this library.
This library is not an official Adafruit Industries library, for more information visit "Simple Circuit" website on:
https://simple-circuit.com/

BSD license, all text above and splash screen header file (splash.h) must be included in any redistribution.
