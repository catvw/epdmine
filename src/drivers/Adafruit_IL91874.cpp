#include "Adafruit_IL91874.h"
#include "Adafruit_EPD.h"

#define EPD_RAM_BW 0x10
#define EPD_RAM_RED 0x13

#define BUSY_WAIT 500

// clang-format off


/**************************************************************************/
/*!
    @brief constructor if using external SRAM chip and software SPI
    @param width the width of the display in pixels
    @param height the height of the display in pixels
    @param SID the SID pin to use
    @param SCLK the SCLK pin to use
    @param DC the data/command pin to use
    @param RST the reset pin to use
    @param CS the chip select pin to use
    @param SRCS the SRAM chip select pin to use
    @param MISO the MISO pin to use
    @param BUSY the busy pin to use
*/
/**************************************************************************/
Adafruit_IL91874::Adafruit_IL91874(int width, int height, int16_t SID,
                                   int16_t SCLK, int16_t DC, int16_t RST,
                                   int16_t CS, int16_t SRCS, int16_t MISO,
                                   int16_t BUSY)
    : Adafruit_EPD(width, height, SID, SCLK, DC, RST, CS, SRCS, MISO, BUSY) {

  buffer1_size = ((uint32_t)width * (uint32_t)height) / 8;
  buffer2_size = buffer1_size;

  if (SRCS >= 0) {
    use_sram = true;
    buffer1_addr = 0;
    buffer2_addr = buffer1_size;
    buffer1 = buffer2 = NULL;
  } else {
    buffer1 = (uint8_t *)malloc(buffer1_size);
    buffer2 = (uint8_t *)malloc(buffer2_size);
  }
}

// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset
/**************************************************************************/
/*!
    @brief constructor if using on-chip RAM and hardware SPI
    @param width the width of the display in pixels
    @param height the height of the display in pixels
    @param DC the data/command pin to use
    @param RST the reset pin to use
    @param CS the chip select pin to use
    @param SRCS the SRAM chip select pin to use
    @param BUSY the busy pin to use
*/
/**************************************************************************/
Adafruit_IL91874::Adafruit_IL91874(int width, int height, int16_t DC,
                                   int16_t RST, int16_t CS, int16_t SRCS,
                                   int16_t BUSY, SPIClass *spi)
    : Adafruit_EPD(width, height, DC, RST, CS, SRCS, BUSY, spi) {

  buffer1_size = ((uint32_t)width * (uint32_t)height) / 8;
  buffer2_size = buffer1_size;

  if (SRCS >= 0) {
    use_sram = true;
    buffer1_addr = 0;
    buffer2_addr = buffer1_size;
    buffer1 = buffer2 = NULL;
  } else {
    buffer1 = (uint8_t *)malloc(buffer1_size);
    buffer2 = (uint8_t *)malloc(buffer2_size);
  }
}

/**************************************************************************/
/*!
    @brief wait for busy signal to end
*/
/**************************************************************************/
void Adafruit_IL91874::busy_wait(void) {
  if (_busy_pin >= 0) {
    while (!digitalRead(_busy_pin)) {
      delay(1); // wait for busy low
    }
  } else {
    delay(BUSY_WAIT);
  }
}

/**************************************************************************/
/*!
    @brief begin communication with and set up the display.
    @param reset if true the reset pin will be toggled.
*/
/**************************************************************************/
void Adafruit_IL91874::begin(bool reset) {
  singleByteTxns = true;
  Adafruit_EPD::begin(reset);

  setBlackBuffer(0, true); // black defaults to inverted
  setColorBuffer(1, true); // red defaults to not inverted

  powerDown();
}

/**************************************************************************/
/*!
    @brief signal the display to update
*/
/**************************************************************************/
void Adafruit_IL91874::update() {
  EPD_command(IL91874_DISPLAY_REFRESH);
  delay(100);
  busy_wait();
  if (_busy_pin <= -1) {
    delay(default_refresh_delay);
  }
}

/**************************************************************************/
/*!
    @brief start up the display
*/
/**************************************************************************/
void Adafruit_IL91874::powerUp() {
  uint8_t buf[5];

  hardwareReset();
  delay(200);
  const uint8_t *init_code = il91874_default_init_code;

  if (_epd_init_code != NULL) {
    init_code = _epd_init_code;
  }
  EPD_commandList(init_code);

  if (_epd_lut_code) {
    EPD_commandList(_epd_lut_code);
  }

  buf[0] = (HEIGHT >> 8) & 0xFF;
  buf[1] = HEIGHT & 0xFF;
  buf[2] = (WIDTH >> 8) & 0xFF;
  buf[3] = WIDTH & 0xFF;
  EPD_command(IL91874_RESOLUTION, buf, 4);

  buf[0] = 0x00;
  EPD_command(IL91874_PDRF, buf, 1);
}

/**************************************************************************/
/*!
    @brief wind down the display
*/
/**************************************************************************/
void Adafruit_IL91874::powerDown() {
  uint8_t buf[1];

  buf[0] = 0xF7;
  EPD_command(IL91874_CDI, buf, 1);

  // power off
  EPD_command(IL91874_POWER_OFF);
  busy_wait();

  // Only deep sleep if we can get out of it
  if (_reset_pin >= 0) {
    buf[0] = 0xA5;
    EPD_command(IL91874_DEEP_SLEEP, buf, 1);
  }
}

/**************************************************************************/
/*!
    @brief Send the specific command to start writing to EPD display RAM
    @param index The index for which buffer to write (0 or 1 or tri-color
   displays) Ignored for monochrome displays.
    @returns The byte that is read from SPI at the same time as sending the
   command
*/
/**************************************************************************/
uint8_t Adafruit_IL91874::writeRAMCommand(uint8_t index) {
  if (index == 0) {
    return EPD_command(EPD_RAM_BW, false);
  }
  if (index == 1) {
    return EPD_command(EPD_RAM_RED, false);
  }
  return 0;
}

/**************************************************************************/
/*!
    @brief Some displays require setting the RAM address pointer
    @param x X address counter value
    @param y Y address counter value
*/
/**************************************************************************/
void Adafruit_IL91874::setRAMAddress(uint16_t x, uint16_t y) {
  // on this chip we do nothing
  (void)x;
  (void)y;
}

void Adafruit_IL91874::displayPartial(uint16_t x, uint16_t y, uint16_t w,
                                      uint16_t h) {
  const int16_t eight_mask = 0xFFF8; // last three bits unset
  int16_t coords[4];
  coords[0] = (int16_t)y & eight_mask;
  coords[1] = width() - (int16_t)x - (int16_t)w;
  coords[2] = -(-((int16_t)y + (int16_t)h) & eight_mask) - coords[0]; // rounds up!
  coords[3] = (int16_t)w;

  // Serial.println("Partial update!");

  // backup & change init to the partial code
  const uint8_t *init_code_backup = _epd_init_code;
  const uint8_t *lut_code_backup = _epd_lut_code;
  _epd_init_code = _epd_partial_init_code;
  _epd_lut_code = _epd_partial_lut_code;

  // perform standard power up
  powerUp();

  // Set X & Y ram counters
  setRAMAddress(0, 0);

  if (use_sram) {
#ifdef EPD_DEBUG
    Serial.println("  Write SRAM buff to EPD");
#endif
    writeSRAMFramebufferToEPD(buffer1_addr, buffer1_size, 0);
  } else {
#ifdef EPD_DEBUG
    Serial.println("  Write RAM buff to EPD");
#endif
    writeRAMFramebufferToEPD(buffer1, buffer1_size, 0);
  }

  if (buffer2_size != 0) {
    // oh there's another buffer eh?
    delay(2);

    // Set X & Y ram counters
    setRAMAddress(0, 0);

    if (use_sram) {
      writeSRAMFramebufferToEPD(buffer2_addr, buffer2_size, 1);
    } else {
      writeRAMFramebufferToEPD(buffer2, buffer2_size, 1);
    }
  }

  // actually send the partial refresh command
  EPD_command(IL91874_PDRF, true);
  for (size_t i = 0; i < 4; ++i) {
    EPD_data((coords[i] >> 8) & 0x01);
    EPD_data(coords[i] & 0xFF);
  }
  delay(100);
  busy_wait();

#ifdef EPD_DEBUG
  Serial.println("  Powering Down");
#endif

  powerDown();
  // change init back
  _epd_lut_code = lut_code_backup;
  _epd_init_code = init_code_backup;
}
