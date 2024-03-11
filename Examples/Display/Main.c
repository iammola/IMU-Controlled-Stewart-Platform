#include "PLL/PLL.h"
#include "RA8875/RA8875.h"

#define SYS_CLOCK 80e6

static void TextMode(void);

int main(void) {
  PLL_Init();
  RA8875_begin(SYS_CLOCK, RA8875_800x480);

  TextMode();
}

static void TextMode(void) {
  char string[15] = "Hello, World! ";

  /* Switch to text mode */
  RA8875_textMode();
  RA8875_cursorBlink(32);

  /* Set a solid for + bg color ... */

  /* ... or a fore color plus a transparent background */

  /* Set the cursor location (in pixels) */
  RA8875_textSetCursor(10, 10);

  /* Render some text! */
  RA8875_textTransparent(RA8875_WHITE);
  RA8875_textWrite(string, 0);
  RA8875_textColor(RA8875_WHITE, RA8875_RED);
  RA8875_textWrite(string, 0);
  RA8875_textTransparent(RA8875_CYAN);
  RA8875_textWrite(string, 0);
  RA8875_textTransparent(RA8875_GREEN);
  RA8875_textWrite(string, 0);
  RA8875_textColor(RA8875_BLACK, RA8875_CYAN);
  RA8875_textWrite(string, 0);
  RA8875_textColor(RA8875_BLACK, RA8875_YELLOW);
  RA8875_textWrite(string, 0);
  RA8875_textTransparent(RA8875_MAGENTA);
  RA8875_textWrite(string, 0);

  /* Change the cursor location and color ... */
  RA8875_textSetCursor(100, 100);
  RA8875_textTransparent(RA8875_RED);
  /* If necessary, enlarge the font */
  RA8875_textEnlarge(1);
  /* ... and render some more text! */
  RA8875_textWrite(string, 0);
  RA8875_textSetCursor(100, 150);
  RA8875_textEnlarge(2);
  RA8875_textWrite(string, 0);
}
