#ifndef LIB_ADAFRUIT_IL91874
#define LIB_ADAFRUIT_IL91874

#include "Adafruit_EPD.h"
#include <Arduino.h>

#define IL91874_PANEL_SETTING 0x00
#define IL91874_POWER_SETTING 0x01
#define IL91874_POWER_OFF 0x02
#define IL91874_POWER_OFF_SEQUENCE 0x03
#define IL91874_POWER_ON 0x04
#define IL91874_POWER_ON_MEASURE 0x05
#define IL91874_BOOSTER_SOFT_START 0x06
#define IL91874_DEEP_SLEEP 0x07
#define IL91874_DTM1 0x10
#define IL91874_DATA_STOP 0x11
#define IL91874_DISPLAY_REFRESH 0x12
#define IL91874_PDTM1 0x14
#define IL91874_PDTM2 0x15
#define IL91874_PDRF 0x16
#define IL91874_LUT1 0x20
#define IL91874_LUTWW 0x21
#define IL91874_LUTBW 0x22
#define IL91874_LUTWB 0x23
#define IL91874_LUTBB 0x24
#define IL91874_PLL 0x30
#define IL91874_CDI 0x50
#define IL91874_RESOLUTION 0x61
#define IL91874_VCM_DC_SETTING 0x82

const uint8_t il91874_default_init_code[] {
  IL91874_BOOSTER_SOFT_START, 3, 0x07, 0x07, 0x17,
  IL91874_POWER_ON, 0,
  0xFF, 200,          // busy wait
  IL91874_PANEL_SETTING, 1, 0x0f, // b/w/r, lut from OTP
  IL91874_PDRF, 1, 0x00,
  0xF8, 2, 0x60, 0xA5, // boost
  0xF8, 2, 0x73, 0x23, // boost
  0xF8, 2, 0x7C, 0x00, // boost
  IL91874_CDI, 1, 0x97,
  0xFE
};

const uint8_t il91874_partial_init_code[] {
  IL91874_BOOSTER_SOFT_START, 3, 0x07, 0x07, 0x17,
  IL91874_POWER_ON, 0,
  0xFF, 200,          // busy wait
  IL91874_PANEL_SETTING, 1, 0x2f, // b/w/r, LUT from reg
  IL91874_PDRF, 1, 0x00,
  0xF8, 2, 0x60, 0xA5, // boost
  0xF8, 2, 0x73, 0x23, // boost
  0xF8, 2, 0x7C, 0x00, // boost
  IL91874_CDI, 1, 0x97,
  0xFE
};

// clang-format on

const unsigned char il91874_lut_full[] = {
  IL91874_LUT1, 44,
  0x00, 0x00,
  0x00, 0x1A, 0x1A, 0x00, 0x00, 0x01,
  0x00, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x0E, 0x01, 0x0E, 0x01, 0x10,
  0x00, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTWW, 42,
  0x90, 0x1A, 0x1A, 0x00, 0x00, 0x01,
  0x40, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x84, 0x0E, 0x01, 0x0E, 0x01, 0x10,
  0x80, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTBW, 42,
  0xA0, 0x1A, 0x1A, 0x00, 0x00, 0x01,
  0x00, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x84, 0x0E, 0x01, 0x0E, 0x01, 0x10,
  0x90, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0xB0, 0x04, 0x10, 0x00, 0x00, 0x05,
  0xB0, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0xC0, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTWB, 42,
  0x90, 0x1A, 0x1A, 0x00, 0x00, 0x01,
  0x40, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x84, 0x0E, 0x01, 0x0E, 0x01, 0x10,
  0x80, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTBB, 42,
  0x90, 0x1A, 0x1A, 0x00, 0x00, 0x01,
  0x20, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x84, 0x0E, 0x01, 0x0E, 0x01, 0x10,
  0x10, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  0xFE
};

const unsigned char il91874_lut_partial[] = {
  IL91874_LUT1, 44,
  0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTWW, 42,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTBW, 42,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x90, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0xB0, 0x04, 0x10, 0x00, 0x00, 0x05,
  0xB0, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0xC0, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTWB, 42,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  IL91874_LUTBB, 42,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x10, 0x0A, 0x0A, 0x00, 0x00, 0x08,
  0x00, 0x04, 0x10, 0x00, 0x00, 0x05,
  0x00, 0x03, 0x0E, 0x00, 0x00, 0x0A,
  0x00, 0x23, 0x00, 0x00, 0x00, 0x01,

  0xFE
};

/**************************************************************************/
/*!
    @brief  Class for interfacing with IL0373 EPD drivers
*/
/**************************************************************************/
class Adafruit_IL91874 : public Adafruit_EPD {
public:
  Adafruit_IL91874(int width, int height, int16_t SID, int16_t SCLK, int16_t DC,
                   int16_t RST, int16_t CS, int16_t SRCS, int16_t MISO,
                   int16_t BUSY = -1);
  Adafruit_IL91874(int width, int height, int16_t DC, int16_t RST, int16_t CS,
                   int16_t SRCS, int16_t BUSY = -1, SPIClass *spi = &SPI);

  void begin(bool reset = true);
  void powerUp();
  void update();
  void powerDown();
  void displayPartial(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

protected:
  uint8_t writeRAMCommand(uint8_t index);
  void setRAMAddress(uint16_t x, uint16_t y);
  void busy_wait();
  void full_luts();
  void partial_luts();
};

#endif
