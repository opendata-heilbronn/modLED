#pragma once

#include "globals.h"
#include "font.h"
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

void updateDisplay();
void clearDisplay();
void dispPrint(int8_t x, int8_t y, uint32_t rrgbColor, const char* text);
void dispPrintf(int8_t x, int8_t y, uint32_t rrgbColor, const char* fmt, ...);
void drawRect(int8_t x, int8_t y, int8_t w, int8_t h, uint32_t rrgb);
void drawPixel(int8_t x, int8_t y, uint32_t rrgb);

#define BLACK   0x00000000
#define WHITE   0xFFFFFFFF
#define RED     0xFFFF0000
#define GREEN   0x0000FF00
#define BLUE    0x000000FF

