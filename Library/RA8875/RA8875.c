/*!
 * @file     Adafruit_RA8875.cpp
 *
 * @mainpage Adafruit RA8875 TFT Driver
 *
 * @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries
 *
 * @section intro_sec Introduction
 *
 * This is the library for the Adafruit RA8875 Driver board for TFT displays
 * ---------------> http://www.adafruit.com/products/1590
 * The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
 * It is tested to work with displays in the Adafruit shop. Other displays
 * may need timing adjustments and are not guanteed to work.
 *
 * Adafruit invests time and resources providing this open
 * source code, please support Adafruit and open-source hardware
 * by purchasing products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, check license.txt for more information.
 * All text above must be included in any redistribution.
 *
 * @section  HISTORY
 *
 * v1.0 - First release
 *
 */
/*!
 * Modified by Ademola Adedeji
 * Rewritten in C for the TM4C123
 */

#define SSI_MODULE 0

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "RA8875.h"
#include "SSI/SSI0.h"
#include "SysTick/SysTick.h"

#include "PORT_BASE.h"
#include "tm4c123gh6pm.h"

#define RST_PIN    (unsigned)(1 << 6) // PA6
#define RST_PCTL_M (unsigned)(GPIO_PCTL_PA6_M)
#define RST_ADDR   (*((volatile uint32_t *)(PORTA_BASE | (RST_PIN << 2))))

// Seems to be the closest max
#define SSI_SPEED 5e6 /*!< 5MHz */

static void RA8875_PLLinit(void);
static void RA8875_initialize(void);

/* GFX Helper Functions */
static void RA8875_circleHelper(int16_t x, int16_t y, int16_t r, uint16_t color, bool filled);
static void RA8875_rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool filled);
static void RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, bool filled);
static void RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, bool filled);
static void RA8875_curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, bool filled);
static void RA8875_roundRectHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, bool filled);

/* Rotation Functions */
static int16_t RA8875_applyRotationX(int16_t x);
static int16_t RA8875_applyRotationY(int16_t y);

static void swap(int16_t *x, int16_t *y);

/**************************************************************************/
/*!
      Initialises the LCD driver and any HW required by the display

      @param s The display size, which can be either:
                  'RA8875_480x80'  (3.8" displays) or
                  'RA8875_480x128' (3.9" displays) or
                  'RA8875_480x272' (4.3" displays) or
                  'RA8875_800x480' (5" and 7" displays)

      @return True if we reached the end
*/
/**************************************************************************/
bool RA8875_begin(const uint32_t SYS_CLOCK, enum RA8875sizes s) {
  _size = s;

  if (_size == RA8875_480x80) {
    _width = 480;
    _height = 80;
  } else if (_size == RA8875_480x128) {
    _width = 480;
    _height = 128;
  } else if (_size == RA8875_480x272) {
    _width = 480;
    _height = 272;
  } else if (_size == RA8875_800x480) {
    _width = 800;
    _height = 480;
  } else {
    return false;
  }
  _rotation = 0;

  SysTick_Init();
  // Arduino SSI0_MODE0 with LOW idle clock and RISING edge Data capture
  SSI0_Init(SYS_CLOCK, SSI_SPEED, SSI_MODE0, SSI_DATA_16);

  GPIO_PORTA_DEN_R |= RST_PIN;      // Digital enable
  GPIO_PORTA_DIR_R |= RST_PIN;      // Set as output
  GPIO_PORTA_AMSEL_R &= ~RST_PIN;   // Disable analog mode
  GPIO_PORTA_PCTL_R &= ~RST_PCTL_M; // Deselect alternate function on pin
  GPIO_PORTA_AFSEL_R &= ~RST_PIN;   // Not using digital alternate functions

  /** Reset Device - Start */
  RST_ADDR = RST_PIN;
  SysTick_Wait10ms(10);

  RST_ADDR = 0;
  SysTick_Wait10ms(10);

  RST_ADDR = RST_PIN;
  SysTick_Wait10ms(10);
  /** Reset Device - End */

uint8_t x;

do {
  // Invalid ID
  x = RA8875_readReg(0);
  break;
} while (x != 0x75); 

  RA8875_initialize();

  RA8875_displayOn(true);   // Turn on the display
  RA8875_EnableGPIOX(true); // Enable TFT - display enable tied to GPIOX

  RA8875_PWM1out(255);
  RA8875_PWM1config(true, RA8875_PWM_CLK_DIV2048); // PWM output for backlight

  RA8875_fillScreen(RA8875_BLACK);

  SysTick_WaitCustom(500, -3);

  return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
      Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void RA8875_softReset(void) {
  RA8875_writeCommand(RA8875_PWRR);
  RA8875_writeData(RA8875_PWRR_SOFTRESET);
  RA8875_writeData(RA8875_PWRR_NORMAL);
  SysTick_WaitCustom(1, -3);
}

/**************************************************************************/
/*!
      Initialise the PLL
*/
/**************************************************************************/
void RA8875_PLLinit(void) {
  if (_size == RA8875_480x80 || _size == RA8875_480x128 || _size == RA8875_480x272) {
    RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    SysTick_WaitCustom(1, -3);
    RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    SysTick_WaitCustom(1, -3);
  } else /* (_size == RA8875_800x480) */ {
    RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
    SysTick_WaitCustom(1, -3);
    RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    SysTick_WaitCustom(1, -3);
  }
}

/**************************************************************************/
/*!
      Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void RA8875_initialize(void) {
  /* Timing values */
  uint8_t  pixclk;
  uint8_t  hsync_start;
  uint8_t  hsync_pw;
  uint8_t  hsync_finetune;
  uint8_t  hsync_nondisp;
  uint8_t  vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

  RA8875_PLLinit();
  RA8875_writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  /* Set the correct values for the display being used */
  if (_size == RA8875_480x80) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
  } else if (_size == RA8875_480x128 || _size == RA8875_480x272) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 0;
  } else // (_size == RA8875_800x480)
  {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    _voffset = 0;
  }

  RA8875_writeReg(RA8875_PCSR, pixclk);
  SysTick_WaitCustom(1, -3);

  /* Horizontal settings registers */
  RA8875_writeReg(RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
  RA8875_writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  RA8875_writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) / 8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  RA8875_writeReg(RA8875_HSTR, hsync_start / 8 - 1);                      // Hsync start: (HSTR + 1)*8
  RA8875_writeReg(RA8875_HPWR,
                  RA8875_HPWR_LOW + (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

  /* Vertical settings registers */
  RA8875_writeReg(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
  RA8875_writeReg(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
  RA8875_writeReg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
  RA8875_writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
  RA8875_writeReg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
  RA8875_writeReg(RA8875_VSTR1, vsync_start >> 8);
  RA8875_writeReg(RA8875_VPWR,
                  RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

  /* Set active window X */
  RA8875_writeReg(RA8875_HSAW0, 0); // horizontal start point
  RA8875_writeReg(RA8875_HSAW1, 0);
  RA8875_writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF); // horizontal end point
  RA8875_writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);

  /* Set active window Y */
  RA8875_writeReg(RA8875_VSAW0, 0 + _voffset); // vertical start point
  RA8875_writeReg(RA8875_VSAW1, 0 + _voffset);
  RA8875_writeReg(RA8875_VEAW0,
                  (uint16_t)(_height - 1 + _voffset) & 0xFF); // vertical end point
  RA8875_writeReg(RA8875_VEAW1, (uint16_t)(_height - 1 + _voffset) >> 8);

  /* ToDo: Setup touch panel? */

  /* Clear the entire window */
  RA8875_writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  SysTick_WaitCustom(500, -3);
}

/**************************************************************************/
/*!
      Returns the display width in pixels

      @return  The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t RA8875_width(void) {
  return _width;
}

/**************************************************************************/
/*!
      Returns the display height in pixels

      @return  The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t RA8875_height(void) {
  return _height;
}

/**************************************************************************/
/*!
 Returns the current rotation (0-3)

 @return  The Rotation Setting
 */
/**************************************************************************/
int8_t RA8875_getRotation(void) {
  return _rotation;
}

/**************************************************************************/
/*!
 Sets the current rotation (0-3)

 @param rotation The Rotation Setting
 */
/**************************************************************************/
void RA8875_setRotation(int8_t rotation) {
  switch (rotation) {
    case 2:
      _rotation = rotation;
      break;
    default:
      _rotation = 0;
      break;
  }
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void RA8875_textMode(void) {
  /* Set text mode */
  RA8875_writeCommand(RA8875_MWCR0);
  uint8_t temp = RA8875_readData();
  temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
  RA8875_writeData(temp);

  /* Select the internal (ROM) font */
  RA8875_writeCommand(0x21);
  temp = RA8875_readData();
  temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
  RA8875_writeData(temp);
}

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)

      @param x The x position of the cursor (in pixels, 0..1023)
      @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void RA8875_textSetCursor(uint16_t x, uint16_t y) {
  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);

  /* Set cursor location */
  RA8875_writeCommand(0x2A);
  RA8875_writeData(x & 0xFF);
  RA8875_writeCommand(0x2B);
  RA8875_writeData(x >> 8);
  RA8875_writeCommand(0x2C);
  RA8875_writeData(y & 0xFF);
  RA8875_writeCommand(0x2D);
  RA8875_writeData(y >> 8);
}

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)

      @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void RA8875_textSetYCursor(uint16_t y) {
  y = RA8875_applyRotationY(y);

  /* Set cursor location */
  RA8875_writeCommand(0x2C);
  RA8875_writeData(y & 0xFF);
  RA8875_writeCommand(0x2D);
  RA8875_writeData(y >> 8);
}

/**************************************************************************/
/*!
      Sets the fore and background color when rendering text

      @param foreColor The RGB565 color to use when rendering the text
      @param bgColor   The RGB565 colot to use for the background
*/
/**************************************************************************/
void RA8875_textColor(uint16_t foreColor, uint16_t bgColor) {
  /* Set Fore Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((foreColor & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((foreColor & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(foreColor & 0x001f);

  /* Set Background Color */
  RA8875_writeCommand(0x60);
  RA8875_writeData((bgColor & 0xf800) >> 11);
  RA8875_writeCommand(0x61);
  RA8875_writeData((bgColor & 0x07e0) >> 5);
  RA8875_writeCommand(0x62);
  RA8875_writeData(bgColor & 0x001f);

  /* Clear transparency flag */
  RA8875_writeCommand(0x22);
  uint8_t temp = RA8875_readData();
  temp &= ~(1 << 6); // Clear bit 6
  RA8875_writeData(temp);
}

/**************************************************************************/
/*!
      Sets the fore color when rendering text with a transparent bg

      @param foreColor The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void RA8875_textTransparent(uint16_t foreColor) {
  uint8_t temp;

  /* Set Fore Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((foreColor & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((foreColor & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(foreColor & 0x001f);

  /* Set transparency flag */
  RA8875_writeCommand(0x22);
  temp = RA8875_readData() | (1 << 6); // Set bit 6
  RA8875_writeData(temp);
}

/**************************************************************************/
/*!
      Sets the text enlarge settings, using one of the following values:

      0 = 1x zoom
      1 = 2x zoom
      2 = 3x zoom
      3 = 4x zoom

      @param scale   The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void RA8875_textEnlarge(uint8_t scale) {
  uint8_t temp;

  if (scale > 3)
    scale = 3; // highest setting is 3

  /* Set font size flags */
  RA8875_writeCommand(0x22);
  temp = RA8875_readData() & ~0xF; // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;

  RA8875_writeData(temp);

  _textScale = scale;
}

/**************************************************************************/
/*!
     Enable Cursor Visibility and Blink
     Here we set bits 6 and 5 in 40h
     As well as the set the blink rate in 44h
     The rate is 0 through max 255
     the lower the number the faster it blinks (00h is 1 frame time,
     FFh is 256 Frames time.
     Blink Time (sec) = BTCR[44h]x(1/Frame_rate)

     @param rate The frame rate to blink
 */
/**************************************************************************/
void RA8875_cursorBlink(uint8_t rate) {
  uint8_t temp;

  RA8875_writeCommand(RA8875_MWCR0);
  temp = RA8875_readData() | RA8875_MWCR0_CURSOR;
  RA8875_writeData(temp);

  RA8875_writeCommand(RA8875_MWCR0);
  temp = RA8875_readData() | RA8875_MWCR0_BLINK;
  RA8875_writeData(temp);

  // if (rate > 255) rate = 255;
  RA8875_writeCommand(RA8875_BTCR);
  RA8875_writeData(rate);
}

/**************************************************************************/
/*!
      Renders some text on the screen when in text mode

      @param buffer    The buffer containing the characters to render
      @param len       The size of the buffer in bytes
*/
/**************************************************************************/
void RA8875_textWrite(const char *buffer, uint16_t len) {
  if (len == 0)
    len = (uint16_t)strlen(buffer);
  RA8875_writeCommand(RA8875_MRWC);
  for (uint16_t i = 0; i < len; i++) {
    RA8875_writeData(buffer[i]);
/// @cond DISABLE
#if defined(__arm__)
    /// @endcond
    // This delay is needed with textEnlarge(1) because
    // Teensy 3.X is much faster than Arduino Uno
    if (_textScale > 0)
      SysTick_WaitCustom(1, -3);
/// @cond DISABLE
#else
    /// @endcond
    // For others, delay starting with textEnlarge(2)
    if (_textScale > 1)
      SysTick_WaitCustom(1, -3);
/// @cond DISABLE
#endif
    /// @endcond
  }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
      Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void RA8875_graphicsMode(void) {
  uint8_t temp;

  RA8875_writeCommand(RA8875_MWCR0);
  temp = RA8875_readData() & ~RA8875_MWCR0_TXTMODE; // bit #7

  RA8875_writeData(temp);
}

/**************************************************************************/
/*!
      Waits for screen to finish by polling the status!

      @param regname The register name to check
      @param waitflag The value to wait for the status register to match

      @return True if the expected status has been reached
*/
/**************************************************************************/
bool RA8875_waitPoll(uint8_t regname, uint8_t waitflag) {
  uint8_t temp;

  /* Wait for the command to finish */
  while (1) {
    temp = RA8875_readReg(regname);
    if (!(temp & waitflag))
      return true;
  }

  return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
      Sets the current X/Y position on the display before drawing

      @param x The 0-based x location
      @param y The 0-base y location
*/
/**************************************************************************/
void RA8875_setXY(uint16_t x, uint16_t y) {
  RA8875_writeReg(RA8875_CURH0, x & 0xFF);
  RA8875_writeReg(RA8875_CURH1, (x >> 8) & 0xFF);
  RA8875_writeReg(RA8875_CURV0, y & 0xFF);
  RA8875_writeReg(RA8875_CURV1, (y >> 8) & 0xFF);
}

/**************************************************************************/
/*!
      HW accelerated function to push a chunk of raw pixel data

      @param num The number of pixels to push
      @param p   The pixel color to use
*/
/**************************************************************************/
void RA8875_pushPixels(uint32_t num, uint16_t p) {
  uint16_t byte = RA8875_DATAWRITE;
  SSI0_StartTransmission();

  SSI0_Write(&byte, 1);
  while (num--) {
    SSI0_Write(&p, 1);
  }

  SSI0_EndTransmission();
}

/**************************************************************************/
/*!
    Fill the screen with the current color
*/
/**************************************************************************/
void RA8875_fillScreenWithCurrentColor(void) {
  RA8875_writeCommand(RA8875_DCR);
  RA8875_writeData(RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
  RA8875_writeData(RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL | RA8875_DCR_DRAWSQUARE);
}

/**************************************************************************/
/*!
    Apply current rotation in the X direction

    @return the X value with current rotation applied
 */
/**************************************************************************/
int16_t RA8875_applyRotationX(int16_t x) {
  switch (_rotation) {
    case 2:
      x = _width - 1 - x;
      break;
  }

  return x;
}

/**************************************************************************/
/*!
    Apply current rotation in the Y direction

    @return the Y value with current rotation applied
 */
/**************************************************************************/
int16_t RA8875_applyRotationY(int16_t y) {
  switch (_rotation) {
    case 2:
      y = _height - 1 - y;
      break;
  }

  return y + _voffset;
}

/**************************************************************************/
/*!
      Draws a single pixel at the specified location

      @param x     The 0-based x location
      @param y     The 0-base y location
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawPixel(int16_t x, int16_t y, uint16_t color) {
  uint16_t byte[] = {(RA8875_DATAWRITE << 8) | (color >> 8), (color & 0xFF) << 8};

  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);

  RA8875_writeReg(RA8875_CURH0, x & 0xFF);
  RA8875_writeReg(RA8875_CURH1, (x >> 8) & 0xFF);
  RA8875_writeReg(RA8875_CURV0, y & 0xFF);
  RA8875_writeReg(RA8875_CURV1, (y >> 8) & 0xFF);
  RA8875_writeCommand(RA8875_MRWC);

  SSI0_StartTransmission();
  SSI0_Write(byte, 2);
  SSI0_EndTransmission();
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead

 @param p     An array of RGB565 color pixels
 @param num   The number of the pixels to draw
 @param x     The 0-based x location
 @param y     The 0-base y location
 */
/**************************************************************************/
void RA8875_drawPixels(uint16_t *p, uint32_t num, int16_t x, int16_t y) {
  uint8_t  dir = RA8875_MWCR0_LRTD;
  uint16_t byte = RA8875_DATAWRITE << 8;

  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);

  RA8875_writeReg(RA8875_CURH0, x & 0xFF);
  RA8875_writeReg(RA8875_CURH1, (x >> 8) & 0xFF);
  RA8875_writeReg(RA8875_CURV0, y & 0xFF);
  RA8875_writeReg(RA8875_CURV1, (y >> 8) & 0xFF);

  if (_rotation == 2) {
    dir = RA8875_MWCR0_RLTD;
  }
  RA8875_writeReg(RA8875_MWCR0, (RA8875_readReg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | dir);

  RA8875_writeCommand(RA8875_MRWC);

  SSI0_StartTransmission();

  SSI0_Write(&byte, 1);
  SSI0_Write(p, num);

  SSI0_EndTransmission();
}

/**************************************************************************/
/*!
      Draws a HW accelerated line on the display

      @param x0    The 0-based starting x location
      @param y0    The 0-base starting y location
      @param x1    The 0-based ending x location
      @param y1    The 0-base ending y location
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  x0 = RA8875_applyRotationX(x0);
  y0 = RA8875_applyRotationY(y0);
  x1 = RA8875_applyRotationX(x1);
  y1 = RA8875_applyRotationY(y1);

  /* Set X */
  RA8875_writeCommand(0x91);
  RA8875_writeData((x0) & 0xFF);
  RA8875_writeCommand(0x92);
  RA8875_writeData((x0 >> 8) & 0xFF);

  /* Set Y */
  RA8875_writeCommand(0x93);
  RA8875_writeData((y0) & 0xFF);
  RA8875_writeCommand(0x94);
  RA8875_writeData((y0 >> 8) & 0xFF);

  /* Set X1 */
  RA8875_writeCommand(0x95);
  RA8875_writeData((x1) & 0xFF);
  RA8875_writeCommand(0x96);
  RA8875_writeData(((x1) >> 8) & 0xFF);

  /* Set Y1 */
  RA8875_writeCommand(0x97);
  RA8875_writeData((y1) & 0xFF);
  RA8875_writeCommand(0x98);
  RA8875_writeData(((y1) >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(RA8875_DCR);
  RA8875_writeData(0x80);

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
    Draw a vertical line

    @param x The X position
    @param y The Y position
    @param h Height
    @param color The color
*/
/**************************************************************************/
void RA8875_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  RA8875_drawLine(x, y, x, y + h, color);
}

/**************************************************************************/
/*!
     Draw a horizontal line

     @param x The X position
     @param y The Y position
     @param w Width
     @param color The color
*/
/**************************************************************************/
void RA8875_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  RA8875_drawLine(x, y, x + w, y, color);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  RA8875_rectHelper(x, y, x + w - 1, y + h - 1, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  RA8875_rectHelper(x, y, x + w - 1, y + h - 1, color, true);
}

/**************************************************************************/
/*!
      Fills the screen with the spefied RGB565 color

      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillScreen(uint16_t color) {
  RA8875_rectHelper(0, 0, _width - 1, _height - 1, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
  RA8875_circleHelper(x, y, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
  RA8875_circleHelper(x, y, r, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color) {
  RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color) {
  RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color) {
  RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color) {
  RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
  RA8875_roundRectHelper(x, y, x + w, y + h, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
  RA8875_roundRectHelper(x, y, x + w, y + h, r, color, true);
}

/**************************************************************************/
/*!
      Helper function for higher level circle drawing code
*/
/**************************************************************************/
void RA8875_circleHelper(int16_t x, int16_t y, int16_t r, uint16_t color, bool filled) {
  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);

  /* Set X */
  RA8875_writeCommand(0x99);
  RA8875_writeData((x) & 0xFF);
  RA8875_writeCommand(0x9a);
  RA8875_writeData((x >> 8) & 0xFF);

  /* Set Y */
  RA8875_writeCommand(0x9b);
  RA8875_writeData((y) & 0xFF);
  RA8875_writeCommand(0x9c);
  RA8875_writeData((y >> 8) & 0xFF);

  /* Set Radius */
  RA8875_writeCommand(0x9d);
  RA8875_writeData((r) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(RA8875_DCR);
  if (filled) {
    RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void RA8875_rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool filled) {
  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);
  w = RA8875_applyRotationX(w);
  h = RA8875_applyRotationY(h);

  /* Set X */
  RA8875_writeCommand(0x91);
  RA8875_writeData((x) & 0xFF);
  RA8875_writeCommand(0x92);
  RA8875_writeData((x >> 8) & 0xFF);

  /* Set Y */
  RA8875_writeCommand(0x93);
  RA8875_writeData((y) & 0xFF);
  RA8875_writeCommand(0x94);
  RA8875_writeData((y >> 8) & 0xFF);

  /* Set X1 */
  RA8875_writeCommand(0x95);
  RA8875_writeData((w) & 0xFF);
  RA8875_writeCommand(0x96);
  RA8875_writeData(((w) >> 8) & 0xFF);

  /* Set Y1 */
  RA8875_writeCommand(0x97);
  RA8875_writeData((h) & 0xFF);
  RA8875_writeCommand(0x98);
  RA8875_writeData(((h) >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(RA8875_DCR);
  if (filled) {
    RA8875_writeData(0xB0);
  } else {
    RA8875_writeData(0x90);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, bool filled) {
  x0 = RA8875_applyRotationX(x0);
  y0 = RA8875_applyRotationY(y0);
  x1 = RA8875_applyRotationX(x1);
  y1 = RA8875_applyRotationY(y1);
  x2 = RA8875_applyRotationX(x2);
  y2 = RA8875_applyRotationY(y2);

  /* Set Point 0 */
  RA8875_writeCommand(0x91);
  RA8875_writeData((x0) & 0xFF);
  RA8875_writeCommand(0x92);
  RA8875_writeData((x0 >> 8) & 0xFF);
  RA8875_writeCommand(0x93);
  RA8875_writeData((y0) & 0xFF);
  RA8875_writeCommand(0x94);
  RA8875_writeData((y0 >> 8) & 0xFF);

  /* Set Point 1 */
  RA8875_writeCommand(0x95);
  RA8875_writeData((x1) & 0xFF);
  RA8875_writeCommand(0x96);
  RA8875_writeData((x1 >> 8) & 0xFF);
  RA8875_writeCommand(0x97);
  RA8875_writeData((y1) & 0xFF);
  RA8875_writeCommand(0x98);
  RA8875_writeData((y1 >> 8) & 0xFF);

  /* Set Point 2 */
  RA8875_writeCommand(0xA9);
  RA8875_writeData((x2) & 0xFF);
  RA8875_writeCommand(0xAA);
  RA8875_writeData((x2 >> 8) & 0xFF);
  RA8875_writeCommand(0xAB);
  RA8875_writeData((y2) & 0xFF);
  RA8875_writeCommand(0xAC);
  RA8875_writeData((y2 >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(RA8875_DCR);
  if (filled) {
    RA8875_writeData(0xA1);
  } else {
    RA8875_writeData(0x81);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, bool filled) {
  xCenter = RA8875_applyRotationX(xCenter);
  yCenter = RA8875_applyRotationY(yCenter);

  /* Set Center Point */
  RA8875_writeCommand(0xA5);
  RA8875_writeData((xCenter) & 0xFF);
  RA8875_writeCommand(0xA6);
  RA8875_writeData((xCenter >> 8) & 0xFF);
  RA8875_writeCommand(0xA7);
  RA8875_writeData((yCenter) & 0xFF);
  RA8875_writeCommand(0xA8);
  RA8875_writeData((yCenter >> 8) & 0xFF);

  /* Set Long and Short Axis */
  RA8875_writeCommand(0xA1);
  RA8875_writeData((longAxis) & 0xFF);
  RA8875_writeCommand(0xA2);
  RA8875_writeData((longAxis >> 8) & 0xFF);
  RA8875_writeCommand(0xA3);
  RA8875_writeData((shortAxis) & 0xFF);
  RA8875_writeCommand(0xA4);
  RA8875_writeData((shortAxis >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(0xA0);
  if (filled) {
    RA8875_writeData(0xC0);
  } else {
    RA8875_writeData(0x80);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level curve drawing code
*/
/**************************************************************************/
void RA8875_curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, bool filled) {
  xCenter = RA8875_applyRotationX(xCenter);
  yCenter = RA8875_applyRotationY(yCenter);
  curvePart = (curvePart + _rotation) % 4;

  /* Set Center Point */
  RA8875_writeCommand(0xA5);
  RA8875_writeData((xCenter) & 0xFF);
  RA8875_writeCommand(0xA6);
  RA8875_writeData((xCenter >> 8) & 0xFF);
  RA8875_writeCommand(0xA7);
  RA8875_writeData((yCenter) & 0xFF);
  RA8875_writeCommand(0xA8);
  RA8875_writeData((yCenter >> 8) & 0xFF);

  /* Set Long and Short Axis */
  RA8875_writeCommand(0xA1);
  RA8875_writeData((longAxis) & 0xFF);
  RA8875_writeCommand(0xA2);
  RA8875_writeData((longAxis >> 8) & 0xFF);
  RA8875_writeCommand(0xA3);
  RA8875_writeData((shortAxis) & 0xFF);
  RA8875_writeCommand(0xA4);
  RA8875_writeData((shortAxis >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(0xA0);
  if (filled) {
    RA8875_writeData(0xD0 | (curvePart & 0x03));
  } else {
    RA8875_writeData(0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rounded rectangle drawing code
 */
/**************************************************************************/
void RA8875_roundRectHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, bool filled) {
  x = RA8875_applyRotationX(x);
  y = RA8875_applyRotationY(y);
  w = RA8875_applyRotationX(w);
  h = RA8875_applyRotationY(h);
  if (x > w)
    swap(&x, &w);
  if (y > h)
    swap(&y, &h);

  /* Set X */
  RA8875_writeCommand(0x91);
  RA8875_writeData((x) & 0xFF);
  RA8875_writeCommand(0x92);
  RA8875_writeData((x >> 8) & 0xFF);

  /* Set Y */
  RA8875_writeCommand(0x93);
  RA8875_writeData((y) & 0xFF);
  RA8875_writeCommand(0x94);
  RA8875_writeData((y >> 8) & 0xFF);

  /* Set X1 */
  RA8875_writeCommand(0x95);
  RA8875_writeData((w) & 0xFF);
  RA8875_writeCommand(0x96);
  RA8875_writeData((w >> 8) & 0xFF);

  /* Set Y1 */
  RA8875_writeCommand(0x97);
  RA8875_writeData((h) & 0xFF);
  RA8875_writeCommand(0x98);
  RA8875_writeData((h >> 8) & 0xFF);

  RA8875_writeCommand(0xA1);
  RA8875_writeData((r) & 0xFF);
  RA8875_writeCommand(0xA2);
  RA8875_writeData((r >> 8) & 0xFF);

  RA8875_writeCommand(0xA3);
  RA8875_writeData((r) & 0xFF);
  RA8875_writeCommand(0xA4);
  RA8875_writeData((r >> 8) & 0xFF);

  /* Set Color */
  RA8875_writeCommand(0x63);
  RA8875_writeData((color & 0xf800) >> 11);
  RA8875_writeCommand(0x64);
  RA8875_writeData((color & 0x07e0) >> 5);
  RA8875_writeCommand(0x65);
  RA8875_writeData(color & 0x001f);

  /* Draw! */
  RA8875_writeCommand(RA8875_ELLIPSE);
  if (filled) {
    RA8875_writeData(0xE0);
  } else {
    RA8875_writeData(0xA0);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}
/**************************************************************************/
/*!
      Set the scroll window

      @param x  X position of the scroll window
      @param y  Y position of the scroll window
      @param w  Width of the Scroll Window
      @param h  Height of the Scroll window
      @param mode Layer to Scroll

 */
/**************************************************************************/
void RA8875_setScrollWindow(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t mode) {
  // Horizontal Start point of Scroll Window
  RA8875_writeCommand(0x38);
  RA8875_writeData(x & 0xFF);
  RA8875_writeCommand(0x39);
  RA8875_writeData((x >> 8) & 0xFF);

  // Vertical Start Point of Scroll Window
  RA8875_writeCommand(0x3a);
  RA8875_writeData(y & 0xFF);
  RA8875_writeCommand(0x3b);
  RA8875_writeData((y >> 8) & 0xFF);

  // Horizontal End Point of Scroll Window
  RA8875_writeCommand(0x3c);
  RA8875_writeData((x + w) & 0xFF);
  RA8875_writeCommand(0x3d);
  RA8875_writeData(((x + w) >> 8) & 0xFF);

  // Vertical End Point of Scroll Window
  RA8875_writeCommand(0x3e);
  RA8875_writeData((y + h) & 0xFF);
  RA8875_writeCommand(0x3f);
  RA8875_writeData(((y + h) >> 8) & 0xFF);

  // Scroll function setting
  RA8875_writeCommand(0x52);
  RA8875_writeData(mode);
}

/**************************************************************************/
/*!
    Scroll in the X direction

    @param dist The distance to scroll

 */
/**************************************************************************/
void RA8875_scrollX(int16_t dist) {
  RA8875_writeCommand(0x24);
  RA8875_writeData(dist & 0xFF);
  RA8875_writeCommand(0x25);
  RA8875_writeData((dist >> 8) & 0xFF);
}

/**************************************************************************/
/*!
     Scroll in the Y direction

     @param dist The distance to scroll

 */
/**************************************************************************/
void RA8875_scrollY(int16_t dist) {
  RA8875_writeCommand(0x26);
  RA8875_writeData(dist & 0xFF);
  RA8875_writeCommand(0x27);
  RA8875_writeData((dist >> 8) & 0xFF);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
    Set the Extra General Purpose IO Register

    @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void RA8875_EnableGPIOX(bool on) {
  if (on)
    RA8875_writeReg(RA8875_GPIOX, 1);
  else
    RA8875_writeReg(RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
    Set the duty cycle of the PWM 1 Clock

    @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void RA8875_PWM1out(uint8_t p) {
  RA8875_writeReg(RA8875_P1DCR, p);
}

/**************************************************************************/
/*!
     Set the duty cycle of the PWM 2 Clock

     @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void RA8875_PWM2out(uint8_t p) {
  RA8875_writeReg(RA8875_P2DCR, p);
}

/**************************************************************************/
/*!
    Configure the PWM 1 Clock

    @param on Whether to enable the clock
    @param clock The Clock Divider
*/
/**************************************************************************/
void RA8875_PWM1config(bool on, uint8_t clock) {
  if (on) {
    RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
     Configure the PWM 2 Clock

     @param on Whether to enable the clock
     @param clock The Clock Divider
*/
/**************************************************************************/
void RA8875_PWM2config(bool on, uint8_t clock) {
  if (on) {
    RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  } else {
    RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
      Enables or disables the on-chip touch screen controller

      @param on Whether to turn touch sensing on or not
*/
/**************************************************************************/
void RA8875_touchEnable(bool on) {
  uint8_t adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV4;

  if (_size == RA8875_800x480) // match up touch size with LCD size
    adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV16;

  if (on) {
    /* Enable Touch Panel (Reg 0x70) */
    RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK | RA8875_TPCR0_WAKEENABLE | adcClk); // 10mhz max!
    /* Set Auto Mode      (Reg 0x71) */
    RA8875_writeReg(RA8875_TPCR1, RA8875_TPCR1_AUTO |
                                      // RA8875_TPCR1_VREFEXT |
                                      RA8875_TPCR1_DEBOUNCE);
    /* Enable TP INT */
    RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) | RA8875_INTC1_TP);
  } else {
    /* Disable TP INT */
    RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) & ~RA8875_INTC1_TP);
    /* Disable Touch Panel (Reg 0x70) */
    RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_DISABLE);
  }
}

/**************************************************************************/
/*!
      Checks if a touch event has occured

      @return  True is a touch event has occured (reading it via
               touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
bool RA8875_touched(void) {
  if (RA8875_readReg(RA8875_INTC2) & RA8875_INTC2_TP)
    return true;
  return false;
}

/**************************************************************************/
/*!
      Reads the last touch event

      @param x  Pointer to the uint16_t field to assign the raw X value
      @param y  Pointer to the uint16_t field to assign the raw Y value

      @return True if successful

      @note Calling this function will clear the touch panel interrupt on
            the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
bool RA8875_touchRead(uint16_t *x, uint16_t *y) {
  uint16_t tx, ty;
  uint8_t  temp;

  tx = RA8875_readReg(RA8875_TPXH);
  ty = RA8875_readReg(RA8875_TPYH);
  temp = RA8875_readReg(RA8875_TPXYL);
  tx <<= 2;
  ty <<= 2;
  tx |= temp & 0x03;        // get the bottom x bits
  ty |= (temp >> 2) & 0x03; // get the bottom y bits

  *x = tx;
  *y = ty;

  /* Clear TP INT Status */
  RA8875_writeReg(RA8875_INTC2, RA8875_INTC2_TP);

  return true;
}

/**************************************************************************/
/*!
      Turns the display on or off

      @param on Whether to turn the display on or not
*/
/**************************************************************************/
void RA8875_displayOn(bool on) {
  if (on)
    RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  else
    RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
    Puts the display in sleep mode, or disables sleep mode if enabled

    @param sleep Whether to sleep or not
*/
/**************************************************************************/
void RA8875_sleep(bool sleep) {
  if (sleep)
    RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
  else
    RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!
    Write data to the specified register

    @param reg Register to write to
    @param val Value to write
*/
/**************************************************************************/
void RA8875_writeReg(uint8_t reg, uint8_t val) {
  RA8875_writeCommand(reg);
  RA8875_writeData(val);
}

/**************************************************************************/
/*!
    Set the register to read from

    @param reg Register to read

    @return The value
*/
/**************************************************************************/
uint8_t RA8875_readReg(uint8_t reg) {
  uint8_t res = 0;

  RA8875_writeCommand(reg);
  res = RA8875_readData();

  return res;
}

/**************************************************************************/
/*!
    Write data to the current register

    @param d Data to write
*/
/**************************************************************************/
void RA8875_writeData(uint8_t d) {
  uint16_t byte = (RA8875_DATAWRITE << 8) | d;

  SSI0_StartTransmission();
  SSI0_Write(&byte, 1);
  SSI0_EndTransmission();
}

/**************************************************************************/
/*!
    Read the data from the current register

    @return The Value
*/
/**************************************************************************/
uint8_t RA8875_readData(void) {
  uint16_t result = 0x00;

  SSI0_StartTransmission();
  SSI0_Read(RA8875_DATAREAD << 8, &result, 1);
  SSI0_EndTransmission();

  return result & 0xFF;
}

/**************************************************************************/
/*!
    Write a command to the current register

    @param d The data to write as a command
 */
/**************************************************************************/
void RA8875_writeCommand(uint8_t d) {
  uint16_t byte = (RA8875_CMDWRITE << 8) | d;

  SSI0_StartTransmission();
  SSI0_Write(&byte, 1);
  SSI0_EndTransmission();
}

/**************************************************************************/
/*!
    Read the status from the current register

    @return The value
 */
/**************************************************************************/
uint8_t RA8875_readStatus(void) {
  uint16_t result = 0x00;

  SSI0_StartTransmission();
  SSI0_Read(RA8875_CMDREAD << 8, &result, 1);
  SSI0_EndTransmission();

  return result & 0xFF;
}

/**************************************************************************/
/*!
     Alias of textWrite to Play nice with Arduino's Print class

     @param buffer The buffer to write
     @param size The size of the buffer

     @return The number of bytes written
 */
/**************************************************************************/
uint32_t RA8875_write(const uint8_t *buffer, uint32_t size) {
  RA8875_textWrite((const char *)buffer, size);
  return size;
}

static void swap(int16_t *x, int16_t *y) {
  int16_t temp = *x;
  *x = *y;
  *y = temp;
}
