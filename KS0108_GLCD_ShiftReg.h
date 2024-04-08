/***************************************************************************************
* This is a library for KS0108 monochrome Graphics Liquid Crystal Display.
* This library works with GLCD modules with 2 or 3 KS0108 chips. A GLCD module with
*   two KS0108 chips (2 CS pins) resolution is 128x64 pixel, and a three-chip module
*   (3 CS pins) resolution is 192x64 pixel.
* This type of display uses parallel interface to communicate with master device and requires
*   14 or 15 GPIO pins mainly depending on the resolution of the display (# of KS0108 chips).
*
* https://simple-circuit.com/
*
*****************************************************************************************
* This library depends on Adafruit GFX library at:
*   https://github.com/adafruit/Adafruit-GFX-Library
*   being present on your system. Please make sure you have installed the latest
*   version before using this library.
* This library is not an official Adafruit Industries library, for more information
*   visit "Simple Circuit" website on:
* https://simple-circuit.com/
*
* BSD license, all text above and splash screen header file (splash.h) must be
*   included in any redistribution.
*
*****************************************************************************************/

#ifndef _KS0108_GLCD_ShiftReg_H_
#define _KS0108_GLCD_ShiftReg_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include <Adafruit_GFX.h>


// uncomment the line below and the splash screen will not be used, frees some flash memory space
//#define KS0108_NO_SPLASH

#define KS0108_ON  1      // ON pixel
#define KS0108_OFF 0      // OFF pixel
#define KS0108_INVERSE 2  // inverse pixel

#define KS0108_CMD_DISPLAY_OFF   0x3E  // lcd off
#define KS0108_CMD_DISPLAY_ON    0x3F  // lcd on

#define KS0108_CMD_SET_DISP_START_LINE  0xC0   // start line address, 0x40
#define KS0108_CMD_SET_PAGE_ADDRESS     0xB8   // page address
#define KS0108_CMD_SET_COL_ADDRESS      0x40   // column address

#define KS0108_Chip_1     1
#define KS0108_Chip_2     2
#define KS0108_Chip_3     3
#define KS0108_Chip_All   255

#define KS0108_CS_ACTIVE_HIGH    HIGH
#define KS0108_CS_ACTIVE_LOW     LOW

#define KS0108_Data()        digitalWrite(di_pin, HIGH)
#define KS0108_Instruction() digitalWrite(di_pin, LOW)

#define ks0108_swap(a, b) { uint8_t t = a; a = b; b = t; }

class KS0108_GLCD_ShiftReg : public Adafruit_GFX {
 public:
  // three KS0108 chip, 192x64 pixel resolution ONE shift register
  KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES, uint8_t SHD, uint8_t SHC);
  // two KS0108 chip, 128x64 pixel resolution ONE shift register
  KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1, uint8_t CS2, uint8_t RES, uint8_t SHD, uint8_t SHC);
  // one KS0108 chip, 128x64 pixel resolution ONE shift register
  KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1, uint8_t RES, uint8_t SHD, uint8_t SHC);
  void init(uint8_t EN, uint8_t DI, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES, uint8_t SHD, uint8_t SHC);

  bool begin(bool cs_active);
  void writeCommand(uint8_t c, uint8_t chip);
  void writeData(uint8_t d, uint8_t chip);
  void ChipSelect(uint8_t chip);
  void ChipdeSelect(uint8_t chip);
  void EnablePulse();

  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void display();
  void clearDisplay(void);
  void fillDisplay(void);
  void scrollRight(uint8_t s);
  void scrollLeft(uint8_t s);
  void scrollDown(uint8_t s);
  void scrollUp(uint8_t s);
  void invertDisp(bool inv);

  protected:
  void write_byte(uint8_t _byte);          // write one byte to data pins
  void data_pins_config(); // data pins
  uint8_t di_pin, en_pin, cs1_pin, cs2_pin, cs3_pin, res_pin, shiftDataPin, shiftClockPin;
  uint8_t ks0108_chips;
  //uint8_t data_pins[8];
  bool ks0108_active;    // a variable to select or deselect a certain KS0108 chip, 0: not selected, 1: selected
  bool ks0108_inverted;  // display invert variable
  uint8_t *buffer;       // display buffer
  
};

#endif