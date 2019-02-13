#include "display.h"

void updateDisplay() {
    for(uint16_t y = 0; y < PIXEL_HEIGHT; y++) {
    uint16_t rowIndex = y * PIXEL_WIDTH;
    for(uint16_t x = 0; x < PIXEL_WIDTH; x++) {
      uint16_t pixelIndex = rowIndex + x;
      uint32_t pixelColor = frameBuf[pixelIndex];

      // does not differentiate between upper and lower half
      uint16_t matrixIndex = ((x / 4) * 16) + ((y % 4) * 4) + (x % 4); 
      matrixIndex += 64 * (y / 8); //offset for next panel

      bool half = (y / 4) % 2; // 0 = upper half, 1 = lower half

      for(uint8_t colorIdx = 0; colorIdx < 4; colorIdx++) {
        uint16_t brightness = (pixelColor >> (8 * colorIdx)) & 0xFF;
        brightness = gammaTable[brightness];
        uint8_t bitToSet =  1 << (half * 4 + (3 - colorIdx));

        for(uint16_t pwmStep = 0; pwmStep < PWM_RESOLUTION; pwmStep++) {
          uint16_t pwmIndex = pwmStep * NUM_PIXELS / 2; 

          if(brightness & (1 << pwmStep)) {
            dmaBuf[pwmIndex + matrixIndex] |= bitToSet;
          }
          else {
            dmaBuf[pwmIndex + matrixIndex] &= ~bitToSet;
          }
        }
      }
    }
  }
}

void clearDisplay() {
    memset(frameBuf, 0, sizeof(frameBuf));
}

void drawPixel(int8_t x, int8_t y, uint32_t rrgb) {
    if(x >= 0 && x < PIXEL_WIDTH && y >= 0 && y < PIXEL_HEIGHT) {
        frameBuf[y * 16 + x] = rrgb;
    }
}

void drawRect(int8_t x, int8_t y, int8_t w, int8_t h, uint32_t rrgb) {
    for(uint8_t xPos = x; x < x + w; x++) {
        for(uint8_t yPos = y; y < y + h; y++) {
            drawPixel(xPos, yPos, rrgb);
        }
    }
}

void dispPrint(int8_t x, int8_t y, uint32_t rrgbColor, const char* text) {
    // loop through all characters in string
    for(uint8_t i = 0; i < strlen(text); i++) {
        char c = text[i];                                       // get character from text
        // loop through all x positions to draw on
        for(int8_t xPos = x; (xPos < x + FONT_WIDTH) && (xPos < PIXEL_WIDTH); xPos++) {
            uint8_t xIdx = xPos - x;                            // calculate index (counting from 0)
            uint8_t col = DEFAULT_FONT[c - FONT_OFFSET][xIdx];  // get the font column from the array
            // loop through all pixels in column, from bottom to top
            for(int8_t yPos = y; (yPos > y - FONT_HEIGHT) && (yPos >= 0); yPos--) {
                uint8_t yIdx = y - yPos;                        // calculate index (counting from 0)
                if(col & (1 << yIdx)) {                         // check if pixel has to be set by font
                    drawPixel(xPos, yPos, rrgbColor);
                }
            }
        }
        x += 4;                                                 // progress to next character x position
    }
}

void dispPrintf(int8_t x, int8_t y, uint32_t rrgbColor, const char* fmt, ...) {
    char buf[16];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    dispPrint(x, y, rrgbColor, buf);
    va_end(args);
}