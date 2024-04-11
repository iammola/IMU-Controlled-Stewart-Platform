/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.2 - Added Graphic Mode function to draw a maze
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdbool.h>
#include <stdint.h>

#include "PLL/PLL.h"
#include "RA8875/RA8875.h"

#define SYS_CLOCK 80e6

static void TextMode(void);
static void DrawMaze(void);

int main(void) {
  PLL_Init();
  RA8875_begin(SYS_CLOCK, RA8875_800x480);

  TextMode();
  DrawMaze();
}

static void DrawMaze(void) {
#define MAZE_X             400
#define MAZE_Y             50
#define MAZE_CELL_SIZE     40
#define MAZE_COLUMNS_COUNT 9
#define MAZE_ROWS_COUNT    9
#define MAZE_WIDTH         (MAZE_COLUMNS_COUNT * MAZE_CELL_SIZE)
#define MAZE_HEIGHT        (MAZE_ROWS_COUNT * MAZE_CELL_SIZE)

  uint8_t  lineIdx;
  uint16_t color;
  int16_t  startX, startY;

  static const struct {
    uint32_t col;
    uint32_t row;
    bool     isHorizontal;
    bool     isGate;
  } lines[] = {
  // Start Gate
      {.col = 5, .row = 1,  .isHorizontal = true,  .isGate = true },
 // 1st row
      {.col = 2, .row = 1,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 1,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 1,  .isHorizontal = false, .isGate = false},
 // 2nd row
      {.col = 3, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 2,  .isHorizontal = false, .isGate = false},
      {.col = 4, .row = 2,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 2,  .isHorizontal = false, .isGate = false},
 // 3rd row
      {.col = 1, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 3,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 3,  .isHorizontal = true,  .isGate = false},
 // 4th row
      {.col = 2, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 9, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 4,  .isHorizontal = false, .isGate = false},
 // 5th row
      {.col = 1, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 5,  .isHorizontal = false, .isGate = false},
 // 6th row
      {.col = 2, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 6,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 6,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 6,  .isHorizontal = false, .isGate = false},
 // 7th row
      {.col = 1, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 4, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 6, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 7,  .isHorizontal = false, .isGate = false},
 // 8th row
      {.col = 3, .row = 8,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 8,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 6, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 8,  .isHorizontal = false, .isGate = false},
 // 9th row
      {.col = 1, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 9,  .isHorizontal = true,  .isGate = false},
 //  End Gate
      {.col = 5, .row = 10, .isHorizontal = true,  .isGate = true },
  };
  const uint8_t linesCount = sizeof(lines) / sizeof(lines[0]);

  /* Clear section */
  RA8875_graphicsMode();

  RA8875_fillRect(MAZE_X, MAZE_Y, MAZE_WIDTH, MAZE_HEIGHT, RA8875_BLACK); // Clear Screen

  RA8875_drawRect(MAZE_X, MAZE_Y, MAZE_WIDTH, MAZE_HEIGHT, RA8875_WHITE); // Maze Box
  for (lineIdx = 0; lineIdx < linesCount; lineIdx++) {                    // Draw lines
    if (lines[lineIdx].row > MAZE_ROWS_COUNT) {
      startY = MAZE_HEIGHT - 1 + MAZE_Y;
    } else {
      startY = (int16_t)(((lines[lineIdx].row - 1) * MAZE_CELL_SIZE) + MAZE_Y);
    }

    startX = (int16_t)(((lines[lineIdx].col - 1) * MAZE_CELL_SIZE) + MAZE_X);
    color = lines[lineIdx].isGate ? RA8875_BLACK : RA8875_WHITE; // Draw walls in white, gates in black for "transparent"

    if (lines[lineIdx].isHorizontal) {
      RA8875_drawFastHLine(startX, startY, MAZE_CELL_SIZE, color);
    } else {
      RA8875_drawFastVLine(startX, startY, MAZE_CELL_SIZE, color);
    }
  }
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
