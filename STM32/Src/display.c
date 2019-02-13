#include "display.h"

void clearDisplay() {
    memset(frameBuf, 0, sizeof(frameBuf));
}

void setPixel(int8_t x, int8_t y, uint32_t rrgb) {
    if(x >= 0 && x < PIXEL_WIDTH && y >= 0 && y < PIXEL_HEIGHT) {
        frameBuf[y * 16 + x] = rrgb;
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
                    setPixel(xPos, yPos, rrgbColor);
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