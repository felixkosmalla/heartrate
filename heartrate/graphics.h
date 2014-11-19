#ifndef _H_GRAPHICS_
#define _H_GRAPHICS_

#include "Arduino.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"



#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#define STATUS_BAR_HEIGHT 20
#define MM2PX 6.53

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF


#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
extern Adafruit_ILI9341 tft;


void graphics_setup(void);

void graphics_loop(void);





#endif



