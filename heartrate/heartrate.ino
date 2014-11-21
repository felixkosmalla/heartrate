#include "SPI.h"          /* SPI library is used to communicate with the
                           * ILI9341_t3 display and the SD card reader. */
#include <SdFat.h>        // SdFat library is used to access the SD card.
#include <Adafruit_GFX.h> // Adafruit GFX library is used for the user interface.
#include <ILI9341_t3.h>   // ILI9341_t3 library defines the display functions.

#include "signal.h"
#include "graphics.h"

void setup()
{
	// Initialize serial port with baud rate 9600bips.
    Serial.begin(9600);
    while (!Serial) {}

    Serial.println("test");

	// setup the heart rate signale aquision
	signal_setup();

	// setup the heart rate display
    //graphics_setup();
}

void loop()
{
    //graphics_loop();
}