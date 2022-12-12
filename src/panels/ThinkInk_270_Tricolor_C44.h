#ifndef _THINKINK_270_TRICOLOR_C44_H
#define _THINKINK_270_TRICOLOR_C44_H

// This file is #included by Adafruit_ThinkInk.h and does not need to
// #include anything else to pick up the EPD header or ink mode enum.

class ThinkInk_270_Tricolor_C44 : public Adafruit_IL91874 {
public:
  ThinkInk_270_Tricolor_C44(int16_t SID, int16_t SCLK, int16_t DC, int16_t RST,
                            int16_t CS, int16_t SRCS, int16_t MISO,
                            int16_t BUSY = -1)
      : Adafruit_IL91874(264, 176, SID, SCLK, DC, RST, CS, SRCS, MISO, BUSY){};

  ThinkInk_270_Tricolor_C44(int16_t DC, int16_t RST, int16_t CS, int16_t SRCS,
                            int16_t BUSY = -1, SPIClass *spi = &SPI)
      : Adafruit_IL91874(264, 176, DC, RST, CS, SRCS, BUSY, spi){};

  void begin(thinkinkmode_t mode = THINKINK_TRICOLOR) {
    Adafruit_IL91874::begin(true);
    setBlackBuffer(0, true);
    setColorBuffer(1, false);

    _epd_init_code = il91874_default_init_code;
    _epd_partial_init_code = il91874_partial_init_code;
    _epd_partial_lut_code = il91874_lut_partial;

    inkmode = mode; // Preserve ink mode for ImageReader or others

    layer_colors[EPD_WHITE] = 0b00;
    layer_colors[EPD_BLACK] = 0b01;
    layer_colors[EPD_RED] = 0b10;
    layer_colors[EPD_GRAY] = 0b10;
    layer_colors[EPD_LIGHT] = 0b00;
    layer_colors[EPD_DARK] = 0b01;

    default_refresh_delay = 13000;
    powerDown();
  }
};

#endif // _THINKINK_270_TRICOLOR_C44_H
