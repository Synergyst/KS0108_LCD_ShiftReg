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

#include <avr/pgmspace.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifdef __AVR__
  #include <util/delay.h>
#endif

#include <stdlib.h>
#include "KS0108_GLCD_ShiftReg.h"
#include <Adafruit_GFX.h>

// three KS0108 chip, 192x64 pixel resolution ONE shift register
KS0108_GLCD_ShiftReg::KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1,
  uint8_t CS2, uint8_t CS3, uint8_t RES, uint8_t SHD, uint8_t SHC) : Adafruit_GFX(192, 64), buffer(NULL)
{
  init(EN, DI, CS1, CS2, CS3, RES, SHD, SHC);
}
// two KS0108 chip, 128x64 pixel resolution ONE shift register
KS0108_GLCD_ShiftReg::KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1,
  uint8_t CS2, uint8_t RES, uint8_t SHD, uint8_t SHC) : Adafruit_GFX(128, 64), buffer(NULL)
{
  init(EN, DI, CS1, CS2, 255, RES, SHD, SHC);
}
// one KS0108 chip, 64x64 pixel resolution ONE shift register
KS0108_GLCD_ShiftReg::KS0108_GLCD_ShiftReg(uint8_t EN, uint8_t DI, uint8_t CS1,
uint8_t RES, uint8_t SHD, uint8_t SHC) : Adafruit_GFX(64, 64), buffer(NULL)
{
  init(EN, DI, CS1, 255, 255, RES, SHD, SHC);
}

void KS0108_GLCD_ShiftReg::init(uint8_t EN, uint8_t DI, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES, uint8_t SHD, uint8_t SHC)
{
  en_pin  = EN;
  di_pin  = DI;
  cs1_pin = CS1;
  cs2_pin = CS2;
  cs3_pin = CS3;
  res_pin = RES;
  shiftDataPin = SHD;
  shiftClockPin = SHC;
}

// KS0108 gLCD initialize function
bool KS0108_GLCD_ShiftReg::begin(bool cs_active) {
  // allocate some RAM space for the display buffer, returns false if failed
  if ( (buffer = (uint8_t *)malloc((WIDTH * HEIGHT) / 8)) == NULL )
    return false;

  clearDisplay();   // clear display buffer

  // pin configuration
  pinMode(di_pin, OUTPUT);      // data/instruction pin
  pinMode(en_pin, OUTPUT);      // enable pin
  pinMode(cs1_pin, OUTPUT);     // chip select pin chip 1
  if (cs2_pin != 255)
    pinMode(cs2_pin, OUTPUT);     // chip select pin for chip 2 (if used)
  if (cs3_pin != 255)
    pinMode(cs3_pin, OUTPUT);   // chip select pin for chip 3 (if used)
  
  ks0108_chips = 1;
  if (cs2_pin != 255 && cs3_pin != 255) {
	ks0108_chips = 3;
  } else if (cs2_pin != 255 && cs3_pin == 255) {
	ks0108_chips = 2;
  } else {
	ks0108_chips = 1;
  }
  
  ks0108_active = cs_active;
  ChipdeSelect(KS0108_Chip_All);   // unselect all KS0108 gLCD chip

  // initialize data pins, configure as outputs
  //data_pins_config();
  
  // shift register configuration
  pinMode(shiftDataPin, OUTPUT);
  pinMode(shiftClockPin, OUTPUT);

  //reset device
  delay(100);
  digitalWrite(res_pin, LOW);
  delay(10);
  digitalWrite(res_pin, HIGH);
  delay(10);

  writeCommand( KS0108_CMD_SET_DISP_START_LINE | 0, KS0108_Chip_All );  // set RAM start line to 0 (top of screen)
  writeCommand( KS0108_CMD_SET_PAGE_ADDRESS | 0, KS0108_Chip_All );     // set page address to 0
  writeCommand( KS0108_CMD_SET_COL_ADDRESS | 0, KS0108_Chip_All );      // set column address to 0
  writeCommand( KS0108_CMD_DISPLAY_ON, KS0108_Chip_All );               // turn the display ON
  
  setRotation(0);
  ks0108_inverted = false;   // initial state of the display, not inverted

  return true;  // gLCD has been initialized with success
}

// the most basic function, set a single pixel
void KS0108_GLCD_ShiftReg::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      ks0108_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      ks0108_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }

    if (ks0108_inverted && color != KS0108_INVERSE) {
      color ^= 1;   // invert color
    }
    switch (color) {
      case KS0108_ON:
        buffer[x + (y / 8) * WIDTH] |= (1 << (y & 7));
        break;
      case KS0108_OFF:
        buffer[x + (y / 8) * WIDTH] &= ~(1 << (y & 7));
        break;
      case KS0108_INVERSE:
        buffer[x + (y / 8) * WIDTH] ^= (1 << (y & 7));
        break;
    }
  }
}

// write command to the display
void KS0108_GLCD_ShiftReg::writeCommand(uint8_t c, uint8_t chip) {
  KS0108_Instruction();   // instuction (command)
  ChipSelect(chip);       // select desired chip(s)

  write_byte(c);

  EnablePulse();
  ChipdeSelect(chip);     // deselect previously selected chip(s)
}

// write data to the display
void KS0108_GLCD_ShiftReg::writeData(uint8_t d, uint8_t chip) {
  KS0108_Data();        // display RAM data
  ChipSelect(chip);     // select desired chip(s)

  write_byte(d);

  EnablePulse();
  ChipdeSelect(chip);   // deselect previously selected chip(s)
}

// write display buffer in LCD hardware
void KS0108_GLCD_ShiftReg::display(void) {
  writeCommand( KS0108_CMD_SET_COL_ADDRESS | 0, KS0108_Chip_All );  // column address = 0 (automatically incremented up to 63)
  for (uint8_t ch = 1; ch < ks0108_chips+1; ch++) {
    for(uint16_t p = 0; p < 8; p++) {
      writeCommand( KS0108_CMD_SET_PAGE_ADDRESS | p, ch );   // page address
      KS0108_Data();        // write data to display RAM
      ChipSelect(ch);       // select desired chip
      for(uint8_t col = 0; col < 64; col++) {
        write_byte( buffer[ (p * WIDTH) + (ch-1)*64 + col ] );
        EnablePulse();
      }
      ChipdeSelect(ch);     // unselect desired chip
    }
  }
}

// clear everything
void KS0108_GLCD_ShiftReg::clearDisplay(void) {
  if (ks0108_inverted)
    memset(buffer, 0xFF, WIDTH*HEIGHT/8);
  else
    memset(buffer, 0, WIDTH*HEIGHT/8);
  cursor_y = cursor_x = 0;
}

// fill all the display
void KS0108_GLCD_ShiftReg::fillDisplay(void) {
  if (ks0108_inverted)
    memset(buffer, 0, WIDTH*HEIGHT/8);
  else
    memset(buffer, 0xFF, WIDTH*HEIGHT/8);
  cursor_y = cursor_x = 0;
}

// scroll display buffer to the right by 's' pixels
void KS0108_GLCD_ShiftReg::scrollRight(uint8_t s) {
  s &= 0x7F;
  while (s--) {
    for(uint8_t p = 0; p < 8; p++) {
      uint8_t msbyte = buffer[ ((p + 1) * WIDTH) - 1 ];
      for(uint8_t col = (WIDTH - 1); col > 0; col--) {
        buffer[ (p * WIDTH) + col ] = buffer[ (p * WIDTH) + col - 1 ];
      }
      buffer[ (p * WIDTH) ] = msbyte;
    }
  }
}
// scroll display buffer to the left by 's' pixels
void KS0108_GLCD_ShiftReg::scrollLeft(uint8_t s) {
  s &= 0x7F;
  while (s--) {
    for(uint8_t p = 0; p < 8; p++) {
      uint8_t lsbyte = buffer[ p * WIDTH ];
      for(uint8_t col = 0; col < (WIDTH - 1); col++) {
        buffer[ (p * WIDTH) + col ] = buffer[ (p * WIDTH) + (col + 1) ];
      }
      buffer[ ((p + 1) * WIDTH) - 1 ] = lsbyte;
    }
  }
}
// scroll display buffer down by 's' pixels
void KS0108_GLCD_ShiftReg::scrollDown(uint8_t s) {
  s &= 0x3F;
  while (s--) {
    for(uint8_t col = 0; col < WIDTH; col++) {
      uint8_t msbit = buffer[ (7 * WIDTH) + col ] & 0x80;  // save lcd buffer ms bit (very last bit in the display)
      for(uint8_t p = 7; p > 0; p--) {
        buffer[ (p * WIDTH) + col ] = (buffer[ (p * WIDTH) + col ] << 1);  // shift left by 1
        uint8_t p_msb = buffer[ ((p-1) * WIDTH) + col ] & 0x80;  // save page byte ms bit
        buffer[ (p * WIDTH) + col ] |= (p_msb >> 7);  // update page byte ls bit
      }
      // first page byte update
      buffer[col] = (buffer[col] << 1);
      buffer[col] |= (msbit >> 7);
    }
  }
}
// scroll display buffer up by 's' pixels
void KS0108_GLCD_ShiftReg::scrollUp(uint8_t s) {
  s &= 0x3F;
  while (s--) {
    for(uint8_t col = 0; col < WIDTH; col++) {
      uint8_t lsbit = buffer[col] & 0x01;  // save lcd buffer ls bit (very first bit in the display)
      for(uint8_t p = 0; p < 7; p++) {
        buffer[ (p * WIDTH) + col ] = (buffer[ (p * WIDTH) + col ] >> 1);  // shift right by 1
        uint8_t p_lsb = buffer[ ((p+1) * WIDTH) + col ] & 0x01;  // save page byte ls bit
        buffer[ (p * WIDTH) + col ] |= (p_lsb << 7);  // update page byte ls bit
      }
      // first page byte update
      buffer[ (7 * WIDTH) + col ] = (buffer[ (7 * WIDTH) + col ] >> 1);
      buffer[ (7 * WIDTH) + col ] |= (lsbit << 7);
    }
  }
}

// invert the display
void KS0108_GLCD_ShiftReg::invertDisp(bool inv) {
  if (ks0108_inverted != inv) {
    for (uint16_t i = 0; i < WIDTH*HEIGHT/8; i++)
      buffer[i] = ~buffer[i];
    ks0108_inverted = inv;
  }
}

// write byte to DB0 --> DB7 pins, MSB to DB7
void KS0108_GLCD_ShiftReg::write_byte(uint8_t _byte) {
  shiftOut(shiftDataPin, shiftClockPin, MSBFIRST, _byte);
  //for (uint8_t i = 0; i < 8; i++, _byte >>= 1) digitalWrite(data_pins[i], _byte & 0x01);
}

// enable pulse
void KS0108_GLCD_ShiftReg::EnablePulse() {
  digitalWrite(en_pin, HIGH);
  delayMicroseconds(1);         // high time must be > 200 ns (data set-up time)
  digitalWrite(en_pin, LOW);
}

// chip select: select one or all KS0108 chips
void KS0108_GLCD_ShiftReg::ChipSelect(uint8_t chip) {
  switch (chip) {
    case KS0108_Chip_1:
      digitalWrite(cs1_pin, ks0108_active);
      break;
    case KS0108_Chip_2:
      digitalWrite(cs2_pin, ks0108_active);
      break;
    case KS0108_Chip_3:
      digitalWrite(cs3_pin, ks0108_active);
      break;
    case KS0108_Chip_All:
      digitalWrite(cs1_pin, ks0108_active);
      digitalWrite(cs2_pin, ks0108_active);
      if (cs3_pin != 255)
        digitalWrite(cs3_pin, ks0108_active);
  }
}
// chip deselect: deselect one or all KS0108 chips
void KS0108_GLCD_ShiftReg::ChipdeSelect(uint8_t chip)
{
  switch (chip) {
    case KS0108_Chip_1:
      digitalWrite(cs1_pin, !ks0108_active);
      break;
    case KS0108_Chip_2:
      digitalWrite(cs2_pin, !ks0108_active);
      break;
    case KS0108_Chip_3:
      digitalWrite(cs3_pin, !ks0108_active);
      break;
    case KS0108_Chip_All:
      digitalWrite(cs1_pin, !ks0108_active);
      digitalWrite(cs2_pin, !ks0108_active);
      if (cs3_pin != 255)
        digitalWrite(cs3_pin, !ks0108_active);
  }
}

/*void KS0108_GLCD_ShiftReg::data_pins_config() {
  for (uint8_t i = 0; i < 8; i++) pinMode(data_pins[i], OUTPUT);
}*/

// end of code.